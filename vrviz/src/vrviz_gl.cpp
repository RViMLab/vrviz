//========= Copyright Valve Corporation ============//


/// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <urdf/model.h>

/// Used to broadcast information about the VR system (positions, buttons)
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

/// Used to render ros messages in the VR scene
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>

/// Needed for rendering image to overlay
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

/// PCL Bridge to/from ROS
#include <pcl_conversions/pcl_conversions.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

/// Inheret everything useful from the openvr example class
#ifdef USE_VULKAN
#include "openvr_vk.h"
#else
#include "openvr_gl.h"
#endif


struct tf_obj{
    Matrix4 transform;
    std::string frame_id;
};



/// ROS Objects
ros::NodeHandle* nh;
ros::Publisher controller_pub[3];
ros::Publisher twist_pub;
tf::TransformBroadcaster* broadcaster;
tf::TransformListener* listener;

/// Parameters
std::string vrviz_include_path;
std::string texture_filename = "/texture_map.png";
std::string texture_character_map=" !\"#$%&\'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_";
std::string base_frame = "vive";
std::string frame_prefix = base_frame;
float hud_dist=1.0;///!< meters; distance that the head's up display will appear away
float hud_size=2.0;///!< Radians; How much
float scaling_factor=1.0f;///!< Unitless; for values >1.0 this will make the scene bigger, relative to the person in VR
int point_size=1;
bool sbs_image=true;///!< If true, render the left half of the image to the left eye, the right half to the right eye. If false, render whole image to both eyes
bool show_tf=false;
bool load_robot=false;
bool show_grid=true;

/// This is a flag that tells the VR code that we have new ROS data
/// \todo This should be a semaphore or mutex
volatile bool scene_update_needed=false;

#ifdef USE_VULKAN
#else
/// Variables for rendering image to overlay
GLuint textureFromImage;
#endif
cv_bridge::CvImagePtr cv_ptr_raw;
bool received_image=false;
cv::Mat image_received;


/// Arrays of objects to be rendered. These have been converted into VR space, and are in a format easily rendered by the VR code.
/// We do this so that the maximum amount of work can be done by the ROS spinner thread, and the VR code can run as fast as possible
/// \warning These arrays are edited by the ROS callback, and read by the VR code! This is probably NOT THREADSAFE!

std::vector<float> color_points_vertdataarray;
std::vector<float> textured_tris_vertdataarray;
std::vector<float> color_tris_vertdataarray;


/*!
 * \brief The VRVizApplication class is overloaded from the example openvr code
 *
 * This code has been coded to stay as close as possible to the CMainApplication()
 * class provided in the examples in openvr. This (potentially) allows us to extend
 * either the OpenGL version or the Vulkan version. However, the VULKAN version is
 * currently not fully implemented, so the flexibility is not really being used.
 *
 */
class VRVizApplication: public CMainApplication {
private:
    tf::Transform previous_trans;
    int pressed_id;
    std::vector<tf_obj> tf_cache;

   public:

    int m_iCylinderNumFacets;
    float m_fAngle;

    VRVizApplication( int argc, char *argv[] ): CMainApplication(argc,argv),
        m_iCylinderNumFacets(12),
        m_fAngle(1.f),
        pressed_id(-1)
    {
        tf_obj base_frame_obj;
        base_frame_obj.frame_id=base_frame;
        base_frame_obj.transform=Matrix4().identity();
        tf_cache.push_back(base_frame_obj);
    }
#ifdef USE_VULKAN
/// \todo Setup overlay
#else
    bool BInitGL()
    {
        /// Call the normal part of initializing OpenGL stuff
        bool bSuccess = CMainApplication::BInitGL();

        /// Now set up the Overlay
        vr::HmdError m_eOverlayError;
        vr::Compositor_OverlaySettings m_overlaySettings;
        std::string m_strName="systemoverlay";
        if( vr::VROverlay() )
        {
            std::string sKey = std::string( "sample." ) + m_strName;
            vr::VROverlayError overlayError = vr::VROverlay()->CreateOverlay( sKey.c_str(), m_strName.c_str(), &m_ulOverlayHandle );
            bSuccess = bSuccess && (overlayError == vr::VROverlayError_None);
        }

        if( bSuccess )
        {

            vr::VROverlay()->SetOverlayFlag(m_ulOverlayHandle, vr::VROverlayFlags_SideBySide_Parallel, sbs_image );
            vr::VROverlay()->SetOverlayWidthInMeters(m_ulOverlayHandle, hud_size*hud_dist);
            OverlayTransform.m[0][3]=0.0;
            OverlayTransform.m[1][3]=0.0;
            OverlayTransform.m[2][3]=-hud_dist;
            OverlayTransform.m[0][0]=1.0;
            OverlayTransform.m[0][1]=0.0;
            OverlayTransform.m[0][2]=0.0;
            OverlayTransform.m[1][0]=0.0;
            OverlayTransform.m[1][1]=1.0;
            OverlayTransform.m[1][2]=0.0;
            OverlayTransform.m[2][0]=0.0;
            OverlayTransform.m[2][1]=0.0;
            OverlayTransform.m[2][2]=1.0;
            vr::VROverlay()->SetOverlayTransformTrackedDeviceRelative(m_ulOverlayHandle, vr::k_unTrackedDeviceIndex_Hmd, &OverlayTransform);
            vr::VROverlay()->SetOverlayInputMethod( m_ulOverlayHandle, vr::VROverlayInputMethod_Mouse );
        }else{
            ROS_ERROR("Failed to setup VR Overlay");
        }

        return bSuccess;
    }
#endif


    /*!
     * \brief RunMainLoop additionally checks that ROS is okay
     */
    void RunMainLoop()
    {
        bool bQuit = false;

        SDL_StartTextInput();
        SDL_ShowCursor( SDL_DISABLE );

        while ( !bQuit )
        {
            bQuit = HandleInput();

            RenderFrame();

            if(scene_update_needed){
                SetupScene();
            }

            bQuit = bQuit || !ros::ok();
        }

        SDL_StopTextInput();
    }

#ifndef USE_VULKAN
    /*!
     * \brief RenderControllerAxes
     *
     * This function has been changed to render things other than the controller axes
     *
     * This renders lines with RGB, which we are using for tf rendering and a grid
     *
     */
    void RenderControllerAxes()
    {
        std::vector<float> vertdataarray;
        if(show_tf){
            for(int ii=0;ii<tf_cache.size();ii++){
                add_frame_to_scene(tf_cache[ii].transform,vertdataarray,0.1);
            }
        }
        if(show_grid){
            add_grid_to_scene(vertdataarray);
        }


        // Setup the VAO the first time through.
        if ( m_unControllerVAO == 0 )
        {
            glGenVertexArrays( 1, &m_unControllerVAO );
            glBindVertexArray( m_unControllerVAO );

            glGenBuffers( 1, &m_glControllerVertBuffer );
            glBindBuffer( GL_ARRAY_BUFFER, m_glControllerVertBuffer );

            GLuint stride = 2 * 3 * sizeof( float );
            uintptr_t offset = 0;

            glEnableVertexAttribArray( 0 );
            glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

            offset += sizeof( Vector3 );
            glEnableVertexAttribArray( 1 );
            glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

            glBindVertexArray( 0 );
        }

        glBindBuffer( GL_ARRAY_BUFFER, m_glControllerVertBuffer );

        // set vertex data if we have some
        if( vertdataarray.size() > 0 )
        {
            //$ TODO: Use glBufferSubData for this...
            glBufferData( GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STREAM_DRAW );
        }

    }
#endif

    /*!
     * \brief set scaling factor for scene
     * \param scale desiring scaling factor
     */
    void setScale(float scale)
    {
        m_fScale = scale;
    }

    /*!
     * \brief set full path for texture file
     * \param path desired path for texture file
     */
    void setTextPath(std::string path)
    {
        m_strTextPath = path;
    }

    /*!
     * \brief set point size in pixels
     * \param point_size desired point size
     */
    void setPointSize(float point_size)
    {
        m_unPointSize=point_size;
    }

    //-----------------------------------------------------------------------------
    // Purpose: This function is intended to set up semi-perminant aspects of the
    //          scene, which for our purposes consists of ROS messages which should
    //          be arriving at ~10hz, whereas we hope that RenderScene will run at
    //          ~90hz or more to avoid nausea.
    //-----------------------------------------------------------------------------
    void SetupScene()
    {
        if ( !m_pHMD )
            return;


        if(received_image){
#ifdef USE_VULKAN
            /// \todo Bind texture
#else
            // Bind texture
            vr::Texture_t texture = {(void*)(uintptr_t)textureFromImage, vr::TextureType_OpenGL, vr::ColorSpace_Auto };
            vr::VROverlay()->SetOverlayTexture( m_ulOverlayHandle, &texture );
            vr::VROverlay()->ShowOverlay(m_ulOverlayHandle);
#endif
        }else{
            //ROS_ERROR("no image as of yet");
        }

        m_uiVertcount = textured_tris_vertdataarray.size()/5;

#ifdef USE_VULKAN

        // Create the vertex buffer and fill with data
        if ( !CreateVulkanBuffer( m_pDevice, m_physicalDeviceMemoryProperties, &textured_tris_vertdataarray[ 0 ], textured_tris_vertdataarray.size() * sizeof( float ),
                                  VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, &m_pSceneVertexBuffer, &m_pSceneVertexBufferMemory ) )
        {
            return;
        }

        // Create constant buffer to hold the per-eye CB data
        for ( uint32_t nEye = 0; nEye < 2; nEye++ )
        {
            VkBufferCreateInfo bufferCreateInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
            bufferCreateInfo.size = sizeof( Matrix4 );
            bufferCreateInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            vkCreateBuffer( m_pDevice, &bufferCreateInfo, nullptr, &m_pSceneConstantBuffer[ nEye ] );

            VkMemoryRequirements memoryRequirements = { };
            vkGetBufferMemoryRequirements( m_pDevice, m_pSceneConstantBuffer[ nEye ], &memoryRequirements );

            VkMemoryAllocateInfo allocInfo = { VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
            MemoryTypeFromProperties( m_physicalDeviceMemoryProperties, memoryRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT, &allocInfo.memoryTypeIndex );
            allocInfo.allocationSize = memoryRequirements.size;

            vkAllocateMemory( m_pDevice, &allocInfo, nullptr, &m_pSceneConstantBufferMemory[ nEye ] );
            vkBindBufferMemory( m_pDevice, m_pSceneConstantBuffer[ nEye ], m_pSceneConstantBufferMemory[ nEye ], 0 );

            // Map and keep mapped persistently
            vkMapMemory( m_pDevice, m_pSceneConstantBufferMemory[ nEye ], 0, VK_WHOLE_SIZE, 0, &m_pSceneConstantBufferData[ nEye ] );
        }
#else
        if(textured_tris_vertdataarray.size()>0){
            glGenVertexArrays( 1, &m_unSceneVAO );
            glBindVertexArray( m_unSceneVAO );

            glGenBuffers( 1, &m_glSceneVertBuffer );
            glBindBuffer( GL_ARRAY_BUFFER, m_glSceneVertBuffer );
            glBufferData( GL_ARRAY_BUFFER, sizeof(float) * textured_tris_vertdataarray.size(), &textured_tris_vertdataarray[0], GL_STATIC_DRAW);

            GLsizei stride = sizeof(VertexDataScene);
            uintptr_t offset = 0;

            glEnableVertexAttribArray( 0 );
            glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride , (const void *)offset);

            offset += sizeof(Vector3);
            glEnableVertexAttribArray( 1 );
            glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, stride, (const void *)offset);


            glBindVertexArray( 0 );
            glDisableVertexAttribArray(0);
            glDisableVertexAttribArray(1);
        }



        // Setup the VAO the first time through.
        if ( m_unPointCloudVAO == 0 )
        {
            glGenVertexArrays( 1, &m_unPointCloudVAO );
            glBindVertexArray( m_unPointCloudVAO );

            glGenBuffers( 1, &m_glPointCloudVertBuffer );
            glBindBuffer( GL_ARRAY_BUFFER, m_glPointCloudVertBuffer );

            GLuint stride = 2 * 3 * sizeof( float );
            uintptr_t offset = 0;

            glEnableVertexAttribArray( 0 );
            glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

            offset += sizeof( Vector3 );
            glEnableVertexAttribArray( 1 );
            glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);

            glBindVertexArray( 0 );
        }

        glBindBuffer( GL_ARRAY_BUFFER, m_glPointCloudVertBuffer );

        // set vertex data if we have some
        if( color_points_vertdataarray.size() > 0 )
        {
            //$ TODO: Use glBufferSubData for this...
            m_uiPointCloudVertcount = color_points_vertdataarray.size()/6;
            glBufferData( GL_ARRAY_BUFFER, sizeof(float) * color_points_vertdataarray.size(), &color_points_vertdataarray[0], GL_STREAM_DRAW );
        }

#endif
        scene_update_needed=false;
    }

    Vector4 sphere2cart(float azimuth, float elevation, float radius)
    {
        Vector4 cart(radius*sin(elevation)*cos(azimuth),
                     radius*sin(elevation)*sin(azimuth),
                     radius*cos(elevation), 1.0);
        return cart;
    }

    void add_char_quad(Matrix4 mat, std::vector<float> &vertdata, char character,int char_num=0,float quad_size=0.0005){

        Vector2 text_global_start;
        text_global_start.x=0.0;
        text_global_start.y=0.0;
        Vector2 text_global_stop;
        text_global_stop.x=1.0;
        text_global_stop.y=1.0;
        for(int idx=0;idx<texture_character_map.size();idx++){
            if(character==texture_character_map[idx]){
                int row=idx%8;
                int col=idx/8;
                Vector2 text_start,text_stop;
                text_start.x=text_global_start.x+(text_global_stop.x-text_global_start.x)*row/8;
                text_stop.x=text_global_start.x+(text_global_stop.x-text_global_start.x)*(row+1)/8;
                text_start.y=text_global_start.y+(text_global_stop.y-text_global_start.y)*col/8;
                text_stop.y=text_global_start.y+(text_global_stop.y-text_global_start.y)*(col+1)/8;
                /// Do something with u1,u2,v1,v2
                ///
                ///

                Vector4 B = mat * Vector4( char_num*quad_size, 0, 0, 1 );
                Vector4 A = mat * Vector4( char_num*quad_size+quad_size, 0, 0, 1 );
                Vector4 D = mat * Vector4( char_num*quad_size+quad_size, quad_size, 0, 1 );
                Vector4 C = mat * Vector4( char_num*quad_size, quad_size, 0, 1 );

                // triangles instead of quads

                AddCubeVertex( B.x, B.y, B.z, text_start.x, text_stop.y, vertdata ); //Back
                AddCubeVertex( A.x, A.y, A.z, text_stop.x, text_stop.y, vertdata );
                AddCubeVertex( D.x, D.y, D.z, text_stop.x, text_start.y, vertdata );
                AddCubeVertex( D.x, D.y, D.z, text_stop.x, text_start.y, vertdata );
                AddCubeVertex( C.x, C.y, C.z, text_start.x, text_start.y, vertdata );
                AddCubeVertex( B.x, B.y, B.z, text_start.x, text_stop.y, vertdata );
                return;
            }
        }
        std::cout << "Error! character \"" << character << "\" not recognized" << std::endl;
    }

    void AddTextToScene( Matrix4 mat, std::vector<float> &vertdata, std::string text,float size=0.010)
    {

        for(int ii=0;ii<text.size();ii++){
            add_char_quad(mat,vertdata,std::toupper(text[ii]),ii-text.size()/2,size);
        }
    }

    //-----------------------------------------------------------------------------
    // Purpose:
    //-----------------------------------------------------------------------------
    void AddSphereToScene( Matrix4 mat, std::vector<float> &vertdata, float radius)
    {

        Vector2 text_start(0.0,0.0);
        Vector2 text_stop(1.0,1.0);

        int num_lat=16;
        int num_lon=num_lat*2;
        for(int lat=0;lat<num_lat;lat++)
        {
            for(int lon=0;lon<num_lon;lon++)
            {
                Vector2 texture_1,texture_2;
                texture_1.x=float(lon  )/num_lon*(text_stop.x-text_start.x)+text_start.x;
                texture_2.x=float(lon+1)/num_lon*(text_stop.x-text_start.x)+text_start.x;
                texture_1.y=float(lat  )/num_lat*(text_stop.y-text_start.y)+text_start.y;
                texture_2.y=float(lat+1)/num_lat*(text_stop.y-text_start.y)+text_start.y;
                float azimuth1=(lon  )/float(num_lon)*M_PI*2;
                float azimuth2=(lon+1)/float(num_lon)*M_PI*2;
                float elevation1=(lat  )/float(num_lat)*M_PI;
                float elevation2=(lat+1)/float(num_lat)*M_PI;

                Vector4 p1=mat*sphere2cart(azimuth1,elevation1,radius);
                Vector4 p2=mat*sphere2cart(azimuth2,elevation1,radius);
                Vector4 p3=mat*sphere2cart(azimuth2,elevation2,radius);
                Vector4 p4=mat*sphere2cart(azimuth1,elevation2,radius);
                Vector2 t1=Vector2(texture_1.x, texture_1.y);
                Vector2 t2=Vector2(texture_2.x, texture_1.y);
                Vector2 t3=Vector2(texture_2.x, texture_2.y);
                Vector2 t4=Vector2(texture_1.x, texture_2.y);

                AddCubeVertex( p1.x, p1.y, p1.z, t1.x, t1.y, vertdata );
                AddCubeVertex( p2.x, p2.y, p2.z, t2.x, t2.y, vertdata );
                AddCubeVertex( p3.x, p3.y, p3.z, t3.x, t3.y, vertdata );
                AddCubeVertex( p3.x, p3.y, p3.z, t3.x, t3.y, vertdata );
                AddCubeVertex( p1.x, p1.y, p1.z, t1.x, t1.y, vertdata );
                AddCubeVertex( p4.x, p4.y, p4.z, t4.x, t4.y, vertdata );
            }
        }
    }

    //-----------------------------------------------------------------------------
    // Purpose:
    //-----------------------------------------------------------------------------
    void AddCylinderToScene( Matrix4 mat, std::vector<float> &vertdata)
    {
        Vector2 text_start(0.0,0.0);
        Vector2 text_stop(1.0,1.0);

        Vector4 Top = mat * Vector4( 0, 0, 0, 1 );
        Vector4 Bot = mat * Vector4( 0, 0, 1, 1 );
        std::vector<Vector4> Top_Ring;
        //std::vector<Vector2> Top_Ring_Text;
        std::vector<Vector4> Bot_Ring;
        //std::vector<Vector2> Bot_Ring_Text;
        for(int ii=0;ii<m_iCylinderNumFacets;ii++){
            float angle = ii*M_PI*2.0/m_iCylinderNumFacets;
            Vector4 top_vert = mat * Vector4( sin(angle), cos(angle), 0, 1 );
            Top_Ring.push_back(top_vert);
            //Top_Ring_Text.push_back(Vector2((cos(angle)*0.5)/(text_stop.x-text_start.x)+text_start.x,(sin(angle)*0.5)/(text_stop.y-text_start.y)+text_start.y));
            Vector4 bot_vert = mat * Vector4( sin(angle), cos(angle), 1, 1 );
            Bot_Ring.push_back(bot_vert);
            //Bot_Ring_Text.push_back(Vector2((cos(angle)*0.5)/(text_stop.x-text_start.x)+text_start.x,(sin(angle)*0.5)/(text_stop.y-text_start.y)+text_start.y));
        }


        for(int ii=0;ii<m_iCylinderNumFacets;ii++){
            int idx1=ii;
            int idx2=(ii+1)%m_iCylinderNumFacets;

            //Top Pinwheel
            AddCubeVertex( Top.x, Top.y, Top.z, text_start.x, text_start.y, vertdata );
            AddCubeVertex( Top_Ring[idx1].x, Top_Ring[idx1].y, Top_Ring[idx1].z, text_start.x, text_stop.y, vertdata );
            AddCubeVertex( Top_Ring[idx2].x, Top_Ring[idx2].y, Top_Ring[idx2].z, text_stop.x, text_stop.y, vertdata );

            //Bottom Pinwheel
            AddCubeVertex( Bot.x, Bot.y, Bot.z, text_start.x, text_start.y, vertdata );
            AddCubeVertex( Bot_Ring[idx1].x, Bot_Ring[idx1].y, Bot_Ring[idx1].z, text_start.x, text_stop.y, vertdata );
            AddCubeVertex( Bot_Ring[idx2].x, Bot_Ring[idx2].y, Bot_Ring[idx2].z, text_stop.x, text_stop.y, vertdata );

            //Side
            AddCubeVertex( Bot_Ring[idx1].x, Bot_Ring[idx1].y, Bot_Ring[idx1].z, text_start.x, text_stop.y, vertdata ); //Front
            AddCubeVertex( Bot_Ring[idx2].x, Bot_Ring[idx2].y, Bot_Ring[idx2].z, text_stop.x, text_stop.y, vertdata );
            AddCubeVertex( Top_Ring[idx2].x, Top_Ring[idx2].y, Top_Ring[idx2].z, text_stop.x, text_start.y, vertdata );
            AddCubeVertex( Top_Ring[idx2].x, Top_Ring[idx2].y, Top_Ring[idx2].z, text_stop.x, text_start.y, vertdata );
            AddCubeVertex( Top_Ring[idx1].x, Top_Ring[idx1].y, Top_Ring[idx1].z, text_start.x, text_start.y, vertdata );
            AddCubeVertex( Bot_Ring[idx1].x, Bot_Ring[idx1].y, Bot_Ring[idx1].z, text_start.x, text_stop.y, vertdata );

        }
    }



    void add_frame_to_scene( Matrix4 mat, std::vector<float> &vertdataarray, float radius, int num_dof=3)
    {

        Vector4 center = mat * Vector4( 0, 0, 0, 1 );

        for ( int i = 0; i < num_dof; ++i )
        {
            Vector3 color( 0, 0, 0 );
            Vector4 point( 0, 0, 0, 1 );
            point[i] += radius*scaling_factor;  // offset in X, Y, Z
            color[i] = 1.0;  // R, G, B
            point = mat * point;
            vertdataarray.push_back( center.x );
            vertdataarray.push_back( center.y );
            vertdataarray.push_back( center.z );

            vertdataarray.push_back( color.x );
            vertdataarray.push_back( color.y );
            vertdataarray.push_back( color.z );

            vertdataarray.push_back( point.x );
            vertdataarray.push_back( point.y );
            vertdataarray.push_back( point.z );

            vertdataarray.push_back( color.x );
            vertdataarray.push_back( color.y );
            vertdataarray.push_back( color.z );

            m_uiControllerVertcount += 2;
        }
    }

    void add_grid_to_scene( std::vector<float> &vertdataarray, float cell_size=1.0, int plane_cell_count=10, Vector3 color=Vector3(0.627,0.627,0.627) ){
        cell_size*=scaling_factor;
        for(int jj=0;jj<3;jj++){
            if(jj==1){jj++;}
            for(int ii=0;ii<plane_cell_count+1;ii++){
                Vector4 point1( -cell_size*plane_cell_count/2.0, 0, -cell_size*plane_cell_count/2.0, 1 );
                Vector4 point2(  cell_size*plane_cell_count/2.0, 0, cell_size*plane_cell_count/2.0, 1 );
                point1[jj]=-cell_size*plane_cell_count/2.0+ii*cell_size;
                point2[jj]=-cell_size*plane_cell_count/2.0+ii*cell_size;

                vertdataarray.push_back( point1.x );
                vertdataarray.push_back( point1.y );
                vertdataarray.push_back( point1.z );

                vertdataarray.push_back( color.x );
                vertdataarray.push_back( color.y );
                vertdataarray.push_back( color.z );

                vertdataarray.push_back( point2.x );
                vertdataarray.push_back( point2.y );
                vertdataarray.push_back( point2.z );

                vertdataarray.push_back( color.x );
                vertdataarray.push_back( color.y );
                vertdataarray.push_back( color.z );

                m_uiControllerVertcount += 2;
            }
        }
    }


    void add_point_to_scene( Matrix4 mat, std::vector<float> &vertdata, Vector4 pt, Vector3 colour)
    {

        Vector4 point = mat * pt;

        vertdata.push_back( point.x );
        vertdata.push_back( point.y );
        vertdata.push_back( point.z );

        vertdata.push_back( colour.x );
        vertdata.push_back( colour.y );
        vertdata.push_back( colour.z );
    }

    /*!
     * \brief update_tf_cache
     *
     * This function is connected to a timer
     *
     * One task it does is to update all the TF's in the cache
     *
     * Another task is to publish the TF's and buttons of the controllers
     *
     */
    void update_tf_cache(const ros::TimerEvent&){
        /// Go through the cache and get updated TF's
        for(int ii=0;ii<tf_cache.size();ii++){
            tf::StampedTransform transform;
            try{
              listener->lookupTransform(base_frame, tf_cache[ii].frame_id,
                                       ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
              ROS_ERROR_THROTTLE(2,"%s",ex.what());
              break;
              /// \todo We should probably remove this TF from the cache
            }
            tf_cache[ii].transform=VrTransform(transform);
        }

        /// Also, publish transforms for things like the HMD and the controllers
        /// (Could publish the transforms for the HMD -> Eyes, the camera, the Lighthouse base stations, etc.)

        /// Broadcast the transform of the HMD relative to the base
        /// \note we invert this because in the GL code, the headset is the 'root' transform, since that's the render target, whereas for ROS we want the 'ground' to be the root
        broadcaster->sendTransform(tf::StampedTransform(TfTransform(m_mat4HMDPose).inverse(), ros::Time::now(), base_frame, frame_prefix + "_hmd" ));

        /// Publish the transform of the eyes relative to the base (I'm not sure why anyone would want them?)
        /// \todo these don't change, so we could store the transforms to not recalculate every timestep
//        broadcaster->sendTransform(tf::StampedTransform(TfTransform(m_mat4eyePosLeft).inverse(), ros::Time::now(), frame_prefix + "_hmd", frame_prefix + "eye_left" ));
//        broadcaster->sendTransform(tf::StampedTransform(TfTransform(m_mat4eyePosRight).inverse(), ros::Time::now(), frame_prefix + "_hmd", frame_prefix + "eye_right" ));


        geometry_msgs::Twist twist_msg;

        int controller_id=0;
        for ( vr::TrackedDeviceIndex_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; ++unTrackedDevice )
        {
            if ( !m_pHMD->IsTrackedDeviceConnected( unTrackedDevice ) )
                continue;

            if( m_pHMD->GetTrackedDeviceClass( unTrackedDevice ) != vr::TrackedDeviceClass_Controller )
                continue;

            if( !m_rTrackedDevicePose[ unTrackedDevice ].bPoseIsValid )
                continue;

            const Matrix4 & mat = m_rmat4DevicePose[unTrackedDevice];



            controller_id++;
            std::stringstream ss;
            ss << frame_prefix << "_controller_" << controller_id;
            /// Publish the transform of the end effector relative to the base
            tf::Transform current_trans=TfTransform(mat);
            broadcaster->sendTransform(tf::StampedTransform(current_trans, ros::Time::now(), base_frame, ss.str() ));

            if(controller_id<3){
                sensor_msgs::Joy joy_msg;
                joy_msg.header.frame_id=ss.str();
                joy_msg.header.stamp=ros::Time::now();

                vr::VRControllerState_t state;
                if( m_pHMD->GetControllerState( unTrackedDevice, &state, sizeof(state) ) )
                {
                    u_int64_t buttons[2];
                    buttons[0]=state.ulButtonPressed;
                    buttons[1]=state.ulButtonTouched;

                    int idx=0; /// These are cast to bool so that they are 0 or 1, and because bits past 32 would be ignored by the message
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_System)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_Grip)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_DPad_Left)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_DPad_Up)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_DPad_Right)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_DPad_Down)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_A)));

                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_ProximitySensor)));

                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_Axis0)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_Axis1)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_Axis2)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_Axis3)));
                    joy_msg.buttons.push_back( bool ( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_Axis4)));

                    if(( buttons[idx] & vr::ButtonMaskFromId(vr::k_EButton_Axis1))){
                        if(pressed_id==controller_id){
                            /// We already knew they pressed this trigger
                            /// So check to see how much has changed since then.
                            /// \note we are implicitly setting Kp=1.0
                            tf::Transform trans = previous_trans.inverseTimes(current_trans);
                            tf::Matrix3x3 r(trans.getRotation());
                            double roll,pitch,yaw;
                            r.getEulerYPR(yaw,pitch,roll);
                            twist_msg.angular.x=roll;
                            twist_msg.angular.z=pitch;
                            twist_msg.angular.y=yaw;
                            twist_msg.linear.x=trans.getOrigin().getX();
                            twist_msg.linear.y=trans.getOrigin().getY();
                            twist_msg.linear.z=trans.getOrigin().getZ();
                        }else if(pressed_id==-1){
                            /// This is the first we have heard about this
                            pressed_id=controller_id;
                            previous_trans=current_trans;
                        }else{
                            ROS_WARN_THROTTLE(2.0,"Please don't press both triggers simultaneously...");
                        }
                    }else{
                        if(pressed_id==controller_id){
                            /// We thought they pressed this trigger, but it appears not anymore
                            pressed_id=-1;
                        }
                    }
                }

                controller_pub[controller_id].publish(joy_msg);
            }
        }
        twist_pub.publish(twist_msg);

    }


    /// Convert to ROS transform, rigid 6DOF 3D transform
    tf::Transform TfTransform(Matrix4 trans)
    {

        tf::Transform transform;
        /// We divide by m_fScale to get back into real world units
        transform.setOrigin(tf::Vector3(trans[12]/m_fScale,
                                        trans[13]/m_fScale,
                                        trans[14]/m_fScale));
        tf::Quaternion q;
        tf::Matrix3x3 m;
        m.setValue( trans[0],trans[4],trans[8],
                    trans[1],trans[5],trans[9],
                    trans[2],trans[6],trans[10]);
        m.getRotation(q);
        transform.setRotation(q);

        return transform;
    }


    /*!
     * \brief Convert from, ROS transform, rigid 6DOF 3D transform
     *
     * \todo This really should be optimized, it's not very efficient right now
     *
     * \param trans ROS tf transform
     * \return internal Matrix version of trasform
     */
    Matrix4 VrTransform(tf::StampedTransform trans)
    {
        Matrix4 mat1,mat2;
        mat1=Matrix4().identity();

        mat1.translate(trans.getOrigin().getX()*m_fScale,
                       trans.getOrigin().getY()*m_fScale,
                       trans.getOrigin().getZ()*m_fScale);

        mat2=Matrix4().identity();
        tf::Matrix3x3 m = trans.getBasis();
        mat2[0]=float(m[0].x());
        mat2[4]=float(m[0].y());
        mat2[8]=float(m[0].z());
        mat2[1]=float(m[1].x());
        mat2[5]=float(m[1].y());
        mat2[9]=float(m[1].z());
        mat2[2]=float(m[2].x());
        mat2[6]=float(m[2].y());
        mat2[10]=float(m[2].z());

        return mat1 * mat2;
    }

    Matrix4 GetRobotMatrixPose( std::string frame_name ){
        /// First, look to see if we have it in the cache
        for(int ii=0;ii<tf_cache.size();ii++){
            if(tf_cache[ii].frame_id==frame_name){
                return tf_cache[ii].transform;
            }
        }
        tf::StampedTransform transform;
        try{
          listener->lookupTransform(base_frame, frame_name,
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR_THROTTLE(2,"%s",ex.what());
          return Matrix4().identity();
        }

        /// We can't find it in our cache, so let's add it
        tf_obj trans;
        trans.transform=VrTransform(transform);
        trans.frame_id=frame_name;
        tf_cache.push_back(trans);
        return trans.transform;
    }

};


VRVizApplication *pVRVizApplication;

/*!
 * \brief Callback for an array of Visualization Markers
 *
 * \warning This currently only works with spheres!
 *
 * For now, it checks to see if something is 'mostly red' 'mostly green'
 * or 'mostly blue' and grabs one of those textures, otherwise it just applies
 * a bland texture.
 *
 * \todo Allow standard RGB color
 * \todo Allow specification of texture coords from texture map?
 * \todo Allow cubes, cylinders, lines, etc.
 *
 * \param msg
 */
void markers_Callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    std::vector<float> vertdataarray;

    for(int ii=0;ii<msg->markers.size();ii++)
    {
        visualization_msgs::Marker marker=msg->markers[ii];
        if(marker.type==visualization_msgs::Marker::SPHERE){
//            Vector4 pt;
//            pt.x=marker.pose.position.x;
//            pt.y=marker.pose.position.y;
//            pt.z=marker.pose.position.z;
//            pt.w=1.f;

//            Matrix4 mat = pVRVizApplication->GetRobotMatrixPose(marker.header.frame_id);
//            sphere_object sphere;

//            if(marker.color.r>0.5 && marker.color.g<0.5 && marker.color.b<0.5){
//                /// It is reddish
//                sphere.texture_id=texture_leather_3;
//            }else if(marker.color.r<0.5 && marker.color.g>0.5 && marker.color.b<0.5){
//                /// It is greenish
//                sphere.texture_id=texture_leather_2;
//            }else if(marker.color.r<0.5 && marker.color.g<0.5 && marker.color.b>0.5){
//                /// It is blueish
//                sphere.texture_id=texture_leather_1;
//            }else{
//                /// It is something else. Make it brown.
//                sphere.texture_id=texture_leather_4;
//            }

//            sphere.location = mat * pt;
//            sphere.radius = marker.scale.x/2.f;
//            spheres.push_back(sphere);
        }else if(marker.type==visualization_msgs::Marker::TEXT_VIEW_FACING){
            ROS_INFO_ONCE("Received Text Message");

            Vector4 pt;
            /// We scale up from real world units to 'vr units'
            pt.x=marker.pose.position.x*scaling_factor;
            pt.y=marker.pose.position.y*scaling_factor;
            pt.z=marker.pose.position.z*scaling_factor;
            pt.w=1.f;

            Matrix4 mat = pVRVizApplication->GetRobotMatrixPose(marker.header.frame_id);

            Matrix4 matTransform4;
            matTransform4.translate(pt.x,pt.y,pt.z);
            Matrix4 mat4 = mat * matTransform4;
            pVRVizApplication->AddTextToScene(mat4,vertdataarray,marker.text,marker.scale.z);

        }
    }
    /// Copy the data over to the shared data
    /// \todo This should be protected with a mutex of sorts!
    textured_tris_vertdataarray=vertdataarray;
    scene_update_needed=true;
}


/*!
 * \brief Callback for a point cloud with color
 *
 * \param msg ROS PointCloud2 Message
 */
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
    ROS_INFO_ONCE("Received Point Cloud 2 Message");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_with_nan(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_in, *cloud_with_nan);
    std::vector<int> indices;
    /// Avoid NAN points, since they would not render well
    pcl::removeNaNFromPointCloud(*cloud_with_nan,*cloud, indices);

    Matrix4 mat = pVRVizApplication->GetRobotMatrixPose(cloud_in->header.frame_id);

    std::vector<float> vertdataarray;

    for(size_t i = 0; i<cloud->points.size(); i++)
    {
        Vector4 pt;
        /// We scale up from real world units to 'vr units'
        pt.x=cloud->points[i].x*scaling_factor;
        pt.y=cloud->points[i].y*scaling_factor;
        pt.z=cloud->points[i].z*scaling_factor;
        pt.w=1.0;

        Vector3 color;
        color.x = cloud->points[i].r/255.0;
        color.y = cloud->points[i].g/255.0;
        color.z = cloud->points[i].b/255.0;

        pVRVizApplication->add_point_to_scene(mat,vertdataarray,pt,color);

        //ROS_INFO("Processed Point %f,%f,%f",pt.x,pt.y,pt.z);
    }

    /// Copy the data over to the shared data
    /// \todo This should be protected with a mutex of sorts!
    color_points_vertdataarray=vertdataarray;
    scene_update_needed=true;
}

#ifdef USE_VULKAN

/// \todo Convert texture

#else
/*!
 * \brief Convert an OpenCV image mat to OpenGL
 *
 * \note This was adapted from https://stackoverflow.com/a/16815662 and may not be optimal
 *
 * \param image
 * \param imageTexture
 */
void BindCVMat2GLTexture(cv::Mat& image, GLuint& imageTexture)
{
   if(image.empty()){
      std::cout << "image empty" << std::endl;
  }else{
      //glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
      glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
      if(!received_image){
          /// This allocates memory for the texture, so we do it the first time
          /// If the image you are subscribing to changes resolution THINGS WILL BREAK
          glGenTextures(1, &imageTexture);
      }
      glBindTexture(GL_TEXTURE_2D, imageTexture);

      cv::cvtColor(image, image, CV_BGR2RGB);
      cv::Mat image_flipped;
      cv::flip(image, image_flipped, 0);

      glTexImage2D(GL_TEXTURE_2D,         // Type of texture
                        0,                   // Pyramid level (for mip-mapping) - 0 is the top level
            GL_RGBA,              // Internal colour format to convert to
                        image_flipped.cols,          // Image width  i.e. 640 for Kinect in standard mode
                        image_flipped.rows,          // Image height i.e. 480 for Kinect in standard mode
                        0,                   // Border width in pixels (can either be 1 or 0)
            GL_RGB,              // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
            GL_UNSIGNED_BYTE,    // Image data type
            image_flipped.ptr());        // The actual image data itself


      // If this renders black ask McJohn what's wrong.
      glGenerateMipmap(GL_TEXTURE_2D);

      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
      glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );

      GLfloat fLargest;
      glGetFloatv( GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest );
      glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest );

      glBindTexture( GL_TEXTURE_2D, 0 );
    }
}
#endif

/*!
 * \brief rawImageCallback
 *
 * \note This callback converts to OpenCV only to then convert to OpenGL, which is probably inefficient
 * \todo See if there is an easy way to pass the raw_image_msg->data directly into glTexImage2D to avoid OpenCV
 *
 * \param raw_image_msg
 */
void rawImageCallback(const sensor_msgs::Image::ConstPtr& raw_image_msg){

    try
    {
        cv_ptr_raw = cv_bridge::toCvCopy(raw_image_msg,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& error)
    {
        ROS_ERROR("cv_bridge exception: %s", error.what());
        return;
    }

#ifdef USE_VULKAN
    /// \todo Convert to texture
    return;
#else
    BindCVMat2GLTexture(cv_ptr_raw->image,textureFromImage);
#endif
    received_image=true;
    scene_update_needed=true;
}

#ifndef USE_VULKAN
/*!
 * \brief load a mesh model with assimp
 * \param mod_url
 * \return
 */
bool loadModel(std::string mod_url,std::string name,Matrix4 trans,Vector3 scale)
{
    if (mod_url.find("package://") == 0)
    {
        mod_url.erase(0, strlen("package://"));
        size_t pos = mod_url.find("/");
        if (pos == std::string::npos)
        {
            ROS_ERROR("Could not parse package:// format into file:// format");
        }

        std::string package = mod_url.substr(0, pos);
        mod_url.erase(0, pos);
        std::string package_path = ros::package::getPath(package);

        if (package_path.empty())
        {
            ROS_ERROR("Package [%s] does not exist",package.c_str());
        }

        mod_url = package_path + mod_url;
    }

    Mesh* myMesh = new Mesh;
    myMesh->name=name;
    myMesh->scale=scale;
    myMesh->trans=trans;

    bool verbose=false;

    /// Assimp doesn't load the 'units' attribute of Collada files
    /// https://github.com/assimp/assimp/issues/165
    /// According to someone on the internet it should be "4 lines of code" to extract it with BOOST.
    /// Unfortunately, it took me signficantly more lines than that...
    /// https://github.com/assimp/assimp/issues/165#issuecomment-47468897
    if(mod_url.substr(mod_url.length()-4,4)==".dae"){

        boost::property_tree::ptree pt;
        boost::property_tree::xml_parser::read_xml(mod_url, pt );

        boost::property_tree::ptree collada=pt.get_child("COLLADA");
        boost::property_tree::ptree asset=collada.get_child("asset");

        if( asset.count("up_axis") != 0 ){
            std::string up_axis_str=asset.get<std::string>("up_axis");
            if(up_axis_str=="Z_UP"){
                ROS_INFO_COND(verbose,"Found Z_UP attribute, compensating");
                myMesh->Z_UP=true;
            }else{
                ROS_WARN("up_axis unexpected! up_axis=%s",up_axis_str.c_str());
            }
        }else{
            ROS_INFO_COND(verbose,"up_axis normal");
        }

        if( asset.count("unit") != 0 ){
            float meter=asset.get<float>("unit.<xmlattr>.meter");
            ROS_INFO_COND(verbose,"Found units, 1 unit=%f meters",meter);
            myMesh->scale.x*=meter;
            myMesh->scale.y*=meter;
            myMesh->scale.z*=meter;
        }else{
            ROS_WARN("no units found");
        }
    }


    if(myMesh->LoadMesh(mod_url)){
        ROS_INFO("Loaded %s's mesh:%s",name.c_str(),mod_url.c_str());
        pVRVizApplication->robot_meshes.push_back(myMesh);
    }else{
        ROS_ERROR("Could not load mesh file %s",mod_url.c_str());
    }
}

/*!
 * \brief load's a robot model from the parameter server
 *
 * We scale the model when we load it, since for now scale is only set at startup
 *
 * \todo Add support for geometric primatives
 *
 * \param vr_scale
 * \return success
 */
bool loadRobot(float vr_scale=1.f){

    urdf::Model model;
    if (!model.initParamWithNodeHandle("robot_description",*nh)){
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }
    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);
    for(int idx=0;idx<links.size();idx++){
        urdf::Link l;
        if(links[idx]->visual && links[idx]->visual->geometry && links[idx]->visual->geometry->type==urdf::Geometry::MESH){
            const urdf::Mesh& mesh = static_cast<const urdf::Mesh&>(*(links[idx]->visual->geometry));

            if ( !mesh.filename.empty() ){

                /// Convert the origin transform to a TF transform as an intermediate
                /// (This is inefficient, but it only gets run once per link)
                tf::StampedTransform trans;
                double x,y,z,w;
                links[idx]->visual->origin.rotation.getQuaternion(x,y,z,w);
                trans.setRotation(tf::Quaternion(x,y,z,w));
                double px=links[idx]->visual->origin.position.x;
                double py=links[idx]->visual->origin.position.y;
                double pz=links[idx]->visual->origin.position.z;
                trans.setOrigin(tf::Vector3(px,py,pz));

                /// Start by assuming a scale of the ROS->VR ratio
                Vector3 scale;
                scale.x=vr_scale;
                scale.y=vr_scale;
                scale.z=vr_scale;
                /// One option for scaling is from the scale attribute of the mesh xml tag
                scale.x*=mesh.scale.x;
                scale.y*=mesh.scale.y;
                scale.z*=mesh.scale.z;

                Matrix4 VRtrans=pVRVizApplication->VrTransform(trans);

                loadModel(mesh.filename,links[idx]->name,VRtrans,scale);

            }
        }
    }
    return true;
}
#endif

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int main(int argc, char *argv[])
{


#ifdef USE_VULKAN
    std::string node_name="vrviz_vk";
#else
    std::string node_name="vrviz_gl";
#endif
    ros::init(argc, argv, node_name);
    nh = new ros::NodeHandle("~");


    ros::Subscriber sub_markers = nh->subscribe("/markers", 1, markers_Callback);
    ros::Subscriber sub_image = nh->subscribe("/rgb/image_raw", 1, rawImageCallback);
    ros::Subscriber sub_cloud = nh->subscribe("/cloud", 1, pointCloudCallback);

    /// For now we only expect to see two controllers.
    controller_pub[1] = nh->advertise<sensor_msgs::Joy>("/controller_left",1);
    controller_pub[2] = nh->advertise<sensor_msgs::Joy>("/controller_right",1);
    twist_pub = nh->advertise<geometry_msgs::Twist>("/controller_twist",1);
    /// The texture file is used for texturing some things
    vrviz_include_path = ros::package::getPath("vrviz")+"/include/vrviz/";

    broadcaster = new tf::TransformBroadcaster;
    listener = new tf::TransformListener;

    /// Pass the command line args to the app
    pVRVizApplication = new VRVizApplication( argc, argv );


    /// This callback will update the transforms both in and out
    ros::Timer timer = nh->createTimer(ros::Duration(0.033), &VRVizApplication::update_tf_cache,pVRVizApplication);

    /// These params should probably be made dynamic?
    nh->getParam("scaling_factor", scaling_factor);
    nh->getParam("hud_dist", hud_dist);
    nh->getParam("hud_size", hud_size);
    nh->getParam("point_size", point_size);
    nh->getParam("load_robot", load_robot);
    nh->getParam("show_tf", show_tf);
    nh->getParam("show_grid", show_grid);


    /// The scaling factor allows us to render large or small things in 'VR world'
    /// A value <1.0 would be for large scenes, and a value >1.0 would be for small scenes
    pVRVizApplication->setScale(scaling_factor);
    pVRVizApplication->setPointSize(point_size);
    pVRVizApplication->setTextPath(vrviz_include_path + texture_filename);

    /// Try initializing the application - this will try to connect to a VR headset
    if (!pVRVizApplication->BInit())
    {
        pVRVizApplication->Shutdown();
        return 1;
    }
    pVRVizApplication->setScale(scaling_factor);

    /// We spawn a spinner to look for callbacks
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();

#ifndef USE_VULKAN
    /// If desired, load a robot model from the parameter server
    if(load_robot){
        loadRobot(scaling_factor);
    }
#endif

    /// This call will run unil the window is closed
    pVRVizApplication->RunMainLoop();

    /// Cleanup
    pVRVizApplication->Shutdown();

	return 0;
}
