/*

	Copyright 2011 Etay Meiri

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/



#include "mesh.h"
#include <tf/transform_broadcaster.h>

Mesh::MeshEntry::MeshEntry()
{
    VB = INVALID_OGL_VALUE;
    IB = INVALID_OGL_VALUE;
    NumIndices  = 0;
    MaterialIndex = INVALID_MATERIAL;
};

Mesh::MeshEntry::~MeshEntry()
{
    if (VB != INVALID_OGL_VALUE)
    {
        glDeleteBuffers(1, &VB);
    }

    if (IB != INVALID_OGL_VALUE)
    {
        glDeleteBuffers(1, &IB);
    }
}

void Mesh::MeshEntry::Init(const std::vector<vr::RenderModel_Vertex_t>& Vertices,
                          const std::vector<u_int32_t>& Indices)
{
    NumIndices = Indices.size();

    // create and bind a VAO to hold state for this model
    glGenVertexArrays( 1, &VA );
    glBindVertexArray( VA );

    // Populate a vertex buffer
    glGenBuffers( 1, &VB );
    glBindBuffer( GL_ARRAY_BUFFER, VB );
    glBufferData( GL_ARRAY_BUFFER, sizeof( vr::RenderModel_Vertex_t ) * Vertices.size(), &Vertices[0], GL_STATIC_DRAW );

    // Identify the components in the vertex buffer
    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vPosition ) );
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, vNormal ) );
    glEnableVertexAttribArray( 2 );
    glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t ), (void *)offsetof( vr::RenderModel_Vertex_t, rfTextureCoord ) );

    // Create and populate the index buffer
    glGenBuffers( 1, &IB );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, IB );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( u_int32_t ) * NumIndices, &Indices[0], GL_STATIC_DRAW );

    glBindVertexArray( 0 );

}


void Mesh::MeshEntry::Init(const std::vector<vr::RenderModel_Vertex_t_rgb>& Vertices,
                          const std::vector<u_int32_t>& Indices)
{
    NumIndices = Indices.size();

    // create and bind a VAO to hold state for this model
    glGenVertexArrays( 1, &VA );
    glBindVertexArray( VA );

    // Populate a vertex buffer
    glGenBuffers( 1, &VB );
    glBindBuffer( GL_ARRAY_BUFFER, VB );
    glBufferData( GL_ARRAY_BUFFER, sizeof( vr::RenderModel_Vertex_t_rgb ) * Vertices.size(), &Vertices[0], GL_STATIC_DRAW );

    // Identify the components in the vertex buffer
    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t_rgb ), (void *)offsetof( vr::RenderModel_Vertex_t_rgb, vPosition ) );
    glEnableVertexAttribArray( 1 );
    glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t_rgb ), (void *)offsetof( vr::RenderModel_Vertex_t_rgb, vNormal ) );
    glEnableVertexAttribArray( 2 );
    glVertexAttribPointer( 2, 3, GL_FLOAT, GL_FALSE, sizeof( vr::RenderModel_Vertex_t_rgb ), (void *)offsetof( vr::RenderModel_Vertex_t_rgb, vColor ) );

    // Create and populate the index buffer
    glGenBuffers( 1, &IB );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, IB );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, sizeof( u_int32_t ) * NumIndices, &Indices[0], GL_STATIC_DRAW );

    glBindVertexArray( 0 );

}


Mesh::Mesh()
{
    trans=Matrix4().identity();
    scale.x=1.0;
    scale.y=1.0;
    scale.z=1.0;
    Z_UP=false;
    load_mesh=false;
}


Mesh::~Mesh()
{
    Clear();
}


void Mesh::Clear()
{
    for (unsigned int i = 0 ; i < m_Textures.size() ; i++) {
        SAFE_DELETE(m_Textures[i]);
    }
}


bool Mesh::LoadMesh(const std::string& Filename)
{
    // Release the previously loaded mesh (if it exists)
    Clear();
    
    bool Ret = false;
    Assimp::Importer Importer;

    const aiScene* pScene = Importer.ReadFile(Filename.c_str(), ASSIMP_LOAD_FLAGS);
    if (pScene) {
        Ret = InitFromScene(pScene, Filename);
    }
    else {
        printf("Error parsing '%s': '%s'\n", Filename.c_str(), Importer.GetErrorString());
    }

    return Ret;
}

bool Mesh::InitFromScene(const aiScene* pScene, const std::string& Filename)
{  
    m_Entries.resize(pScene->mNumMeshes);
    m_Textures.resize(pScene->mNumMaterials);

    InitMaterials(pScene, Filename);

    // Initialize the meshes in the scene one by one
    for (unsigned int i = 0 ; i < m_Entries.size() ; i++) {
        const aiMesh* paiMesh = pScene->mMeshes[i];
        InitMesh(i, paiMesh, pScene->mRootNode);
    }

    return true;
}

Vector4 Mesh::sphere2cart(float azimuth, float elevation, float radius)
{
    Vector4 cart(radius*sin(elevation)*cos(azimuth),
                 radius*sin(elevation)*sin(azimuth),
                 radius*cos(elevation), 1.0);
    return cart;
}

void Mesh::AddColorVertex(Vector4 pt,Vector4 normal,Vector3 color, std::vector<vr::RenderModel_Vertex_t_rgb> &Vertices, std::vector<u_int32_t> &Indices)
{
    vr::RenderModel_Vertex_t_rgb v;
    v.vPosition.v[0]=pt.x;
    v.vPosition.v[1]=pt.y;
    v.vPosition.v[2]=pt.z;
    v.vColor.v[0]=color.x;
    v.vColor.v[1]=color.y;
    v.vColor.v[2]=color.z;
    v.vNormal.v[0]=normal.x;
    v.vNormal.v[1]=normal.y;
    v.vNormal.v[2]=normal.z;

    /// We are being inefficient and adding indices redundantly.
    Indices.push_back(Vertices.size());
    Vertices.push_back(v);

}

void Mesh::AddColorTri(Vector4 pt1, Vector4 pt2, Vector4 pt3, Vector3 color, std::vector<vr::RenderModel_Vertex_t_rgb> &Vertices, std::vector<u_int32_t> &Indices)
{
    Vector3 pt1_b(pt1.x-pt2.x,pt1.y-pt2.y,pt1.z-pt2.z);
    Vector3 pt2_b(pt2.x-pt3.x,pt2.y-pt3.y,pt2.z-pt3.z);
    Vector3 nm = pt1_b.cross(pt2_b);
    Vector4 normal(nm.x,nm.y,nm.z,0.0);

    AddColorVertex(pt1,normal,color,Vertices,Indices);
    AddColorVertex(pt2,normal,color,Vertices,Indices);
    AddColorVertex(pt3,normal,color,Vertices,Indices);
}

geometry_msgs::Quaternion Mesh::quatPoint2Point(Vector4 p1,Vector4 p2,float distance)
{
    float x = p1.x;
    float y = p1.y;
    float z = p1.z;
    float x_n = p2.x;
    float y_n = p2.y;
    float z_n = p2.z;

    geometry_msgs::Quaternion q;

    /// Ensure that there is a nonzero distance
    if(distance < 1e-6){distance=1e-6;}

    /// We need a quaternion from the vector change https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
    /// We take a cross product of the vector from the first point to the second point, and the natural orientation of the cyliner:(0,0,1)
    q.x=-(y-y_n);
    q.y=(x-x_n);
    q.z=0.0;
    /// Then since we used a unit vector for the cylinder, w is just the distance plus the dot product of the vector from the first point to the second point, and the natural orientation of the cyliner:(0,0,1)
    q.w=distance + (z-z_n);
    /// Normalize the quaternion
    float quaternion_norm = sqrt(pow(q.x,2)+
                                 pow(q.y,2)+
                                 pow(q.z,2)+
                                 pow(q.w,2));
    if(quaternion_norm>0.000001){
        /// The quaternion isn't close to zero, so it isn't singular.
        q.x = q.x / quaternion_norm;
        q.y = q.y / quaternion_norm;
        q.z = q.z / quaternion_norm;
        q.w = q.w / quaternion_norm;
    }else{
        /// The quaternion is close to singular! Just default to identity quat
        q.x = 0.0;
        q.y = 0.0;
        q.z = 0.0;
        q.w = 1.0;

    }
    return q;
}

Matrix4 Mesh::quat2mat(geometry_msgs::Quaternion quat)
{
    Matrix4 mat5;
    tf::Quaternion q(quat.x,
                     quat.y,
                     quat.z,
                     quat.w);
    tf::Matrix3x3 m(q);
    mat5.set(m.getColumn(0).getX(),
             m.getColumn(0).getY(),
             m.getColumn(0).getZ(),0,
             m.getColumn(1).getX(),
             m.getColumn(1).getY(),
             m.getColumn(1).getZ(),0,
             m.getColumn(2).getX(),
             m.getColumn(2).getY(),
             m.getColumn(2).getZ(),0,
             0,0,0,1);
    return mat5;
}

void Mesh::InitMarker(float scaling_factor)
{
    m_Entries.resize(1);
    m_Entries[0].MaterialIndex=NO_TEXTURE;
    std::vector<vr::RenderModel_Vertex_t_rgb> Vertices;
    std::vector<u_int32_t> Indices;

    Vector4 pt;
    /// We scale up from real world units to 'vr units'
    pt.x=marker.pose.position.x*scaling_factor;
    pt.y=marker.pose.position.y*scaling_factor;
    pt.z=marker.pose.position.z*scaling_factor;
    pt.w=1.f;

    Matrix4 mat4;
    mat4.translate(pt.x,pt.y,pt.z);

    Vector3 color(marker.color.r,
                  marker.color.g,
                  marker.color.b);

    Matrix4 mat5 = quat2mat(marker.pose.orientation);

    Matrix4 mat6 = mat4*mat5;

    Vector3 radius(marker.scale.x/2.0*scaling_factor,marker.scale.y/2.0*scaling_factor,marker.scale.z/2.0*scaling_factor);

    if(marker.type==visualization_msgs::Marker::ARROW){
        /// scale.x is length
        /// \warning there's a custom arrow type in rviz where you can use points to define the shape. That isn't implemented here.
        float length=marker.scale.x*scaling_factor; /// Use scale.x to specify the height.
        InitArrow(Vertices,Indices,mat6,radius.y,radius.z,length,color);
    }else if(marker.type==visualization_msgs::Marker::CUBE){
        /// scale x,y,z all used
        InitCube(Vertices,Indices,radius,color,mat6);
    }else if(marker.type==visualization_msgs::Marker::SPHERE){
        /// scale.x should be diameter, so radius.x is radius
        InitSphere(Vertices,Indices,radius.x,color,pt);
    }else if(marker.type==visualization_msgs::Marker::CYLINDER){
        /// scale.x is diameter in x direction (currently don't support ellipse)
        float length=marker.scale.z*scaling_factor; /// Use scale.z to specify the height.
        InitCylinder(Vertices,Indices,mat6,radius.x,length,color);
    }else if(marker.type==visualization_msgs::Marker::LINE_STRIP || marker.type==visualization_msgs::Marker::LINE_LIST){
        /// The only difference between them is that a strip goes 0->1->2->3, while a list goes 0->1  2->3
        /// \warning This is not properly implimented, but it may be functional.
        int increment = 1;
        if(marker.type==visualization_msgs::Marker::LINE_LIST){
            increment = 2;
        }
        Matrix4 mat7,mat8,mat9;
        for(int idx=0;idx<((int) marker.points.size())-1;idx+=increment)
        {
            /// Get the points in global coords
            Vector4 pos1(marker.points[idx].x*scaling_factor,
                        marker.points[idx].y*scaling_factor,
                        marker.points[idx].z*scaling_factor,1);
            Vector4 gpos1 = mat6*pos1;
            Vector4 pos2(marker.points[idx+1].x*scaling_factor,
                        marker.points[idx+1].y*scaling_factor,
                        marker.points[idx+1].z*scaling_factor,1);
            Vector4 gpos2 = mat6*pos2;

            float distance = std::sqrt((marker.points[idx].x-marker.points[idx+1].x)*(marker.points[idx].x-marker.points[idx+1].x)+
                                       (marker.points[idx].y-marker.points[idx+1].y)*(marker.points[idx].y-marker.points[idx+1].y)+
                                       (marker.points[idx].z-marker.points[idx+1].z)*(marker.points[idx].z-marker.points[idx+1].z))*scaling_factor;

            /// Translate to the average of the two points, since cylinder & cube are both centered.
            mat7.identity();
            mat7.translate((gpos1.x+gpos2.x)/2.0,
                           (gpos1.y+gpos2.y)/2.0,
                           (gpos1.z+gpos2.z)/2.0);
            /// Also get rotation to point from one point towards the other
            mat8 = quat2mat(quatPoint2Point(gpos1,gpos2,distance));
            mat9 = mat7 * mat8;

            if(idx<((int) marker.colors.size())-1){
                /// \warning this is supposed to blend colors from colors[idx] at points[idx] to colors[idx+1] at points[idx+1], but we don't have that ability so we just average them.
                /// \todo figure out if we can easily do per-vertex color, since that would fix this.
                color.x = (marker.colors[idx].r+marker.colors[idx+1].r)/2.0;
                color.y = (marker.colors[idx].g+marker.colors[idx+1].g)/2.0;
                color.z = (marker.colors[idx].b+marker.colors[idx+1].b)/2.0;
            }
            /// Not properly implimented. This should be a simple camera-facing quad.
            /// We could use a cylinder because making things face the camera is annoying to impliment using this code.
//            InitCylinder(Vertices,Indices,mat9,radius.x,distance,color,6);
            /// We could also use a box, it's less verts, but also a bit silly looking from certain angles.
            radius.z = distance/2.0;
            radius.y = radius.x;
            InitCube(Vertices,Indices,radius,color,mat9);

        }
    }else if(marker.type==visualization_msgs::Marker::CUBE_LIST){
        Matrix4 mat7,mat8;
        for(int idx=0;idx<marker.points.size();idx++)
        {
            /// Make simple translation matrix
            mat7.identity();
            mat7.translate(marker.points[idx].x*scaling_factor,
                           marker.points[idx].y*scaling_factor,
                           marker.points[idx].z*scaling_factor);
            mat8 = mat6*mat7;
            if(idx<marker.colors.size()){
                /// If there are per-cube colors, use those
                color.x = marker.colors[idx].r;
                color.y = marker.colors[idx].g;
                color.z = marker.colors[idx].b;
            }
            InitCube(Vertices,Indices,radius,color,mat8);
        }
    }else if(marker.type==visualization_msgs::Marker::SPHERE_LIST){
        for(int idx=0;idx<marker.points.size();idx++)
        {
            /// Only bother getting the position, it's a sphere so who cares about orientation
            Vector4 pos(marker.points[idx].x*scaling_factor,
                        marker.points[idx].y*scaling_factor,
                        marker.points[idx].z*scaling_factor,1);
            Vector4 gpos = mat6*pos;
            if(idx<marker.colors.size()){
                /// If there are per-sphere colors, use those
                color.x = marker.colors[idx].r;
                color.y = marker.colors[idx].g;
                color.z = marker.colors[idx].b;
            }
            /// scale.x should be diameter, so radius.x is radius
            InitSphere(Vertices,Indices,radius.x,color,gpos);
        }
    }else if(marker.type==visualization_msgs::Marker::POINTS){
        Matrix4 mat7,mat8;
        for(int idx=0;idx<marker.points.size();idx++)
        {
            /// Make simple translation matrix
            mat7.identity();
            mat7.translate(marker.points[idx].x*scaling_factor,
                           marker.points[idx].y*scaling_factor,
                           marker.points[idx].z*scaling_factor);
            mat8 = mat6*mat7;
            if(idx<marker.colors.size()){
                /// If there are per-point colors, use those
                color.x = marker.colors[idx].r;
                color.y = marker.colors[idx].g;
                color.z = marker.colors[idx].b;
            }
            /// scale.x should be diameter, so radius.x is radius
            /// We use a reduced number of facets, since in rviz this is just 1 quad facing the camera anyways.
            /// We could use a cube too if we wanted.
            InitCube(Vertices,Indices,Vector3(radius.x,radius.x,radius.x),color,mat8);
        }
    }else if(marker.type==visualization_msgs::Marker::TEXT_VIEW_FACING){
        /// Implimented in the main loop, since it uses the textured pipeline
    }else if(marker.type==visualization_msgs::Marker::TRIANGLE_LIST){
        InitTriangles(Vertices,Indices,mat6,radius,marker.points,marker.colors,color);
    }

    if(marker.type==visualization_msgs::Marker::MESH_RESOURCE){
        /// This is a separate if, since we don't want to call Init for meshes, we want to call LoadMesh
        /// \todo load_mesh and initialized are probably redundant, so they could probably be simplified.
        /// \warning the mesh is only loaded once, so if the pose/orientation of a MESH_RESOURCE changes this won't detect that.
        if(load_mesh){
            if(LoadMesh(filename)){
                initialized=true;
                needs_update=false;
            }else{
                initialized=false;
            }
            /// I don't know why it would succeed on further attempts, so don't keep trying?
            load_mesh=false;
        }
    }else{
        m_Entries[0].Init(Vertices,Indices);
        initialized=true;
        needs_update=false;
    }
}

void Mesh::InitCube(std::vector<vr::RenderModel_Vertex_t_rgb> &Vertices, std::vector<u_int32_t> &Indices, Vector3 radius, Vector3 color, Matrix4 mat )
{
    // The eight corners of the cube
    Vector4 A = mat * Vector4( -radius.x, -radius.y, -radius.z, 1 );
    Vector4 B = mat * Vector4(  radius.x, -radius.y, -radius.z, 1 );
    Vector4 C = mat * Vector4(  radius.x,  radius.y, -radius.z, 1 );
    Vector4 D = mat * Vector4( -radius.x,  radius.y, -radius.z, 1 );
    Vector4 E = mat * Vector4( -radius.x, -radius.y,  radius.z, 1 );
    Vector4 F = mat * Vector4(  radius.x, -radius.y,  radius.z, 1 );
    Vector4 G = mat * Vector4(  radius.x,  radius.y,  radius.z, 1 );
    Vector4 H = mat * Vector4( -radius.x,  radius.y,  radius.z, 1 );

    // triangles instead of quads
    AddColorTri(E,F,G,color,Vertices,Indices); //Front
    AddColorTri(G,H,E,color,Vertices,Indices);

    AddColorTri(B,A,D,color,Vertices,Indices); //Back
    AddColorTri(D,C,B,color,Vertices,Indices);

    AddColorTri(H,G,C,color,Vertices,Indices); //Top
    AddColorTri(C,D,H,color,Vertices,Indices);

    AddColorTri(A,B,F,color,Vertices,Indices); //Bottom
    AddColorTri(F,E,A,color,Vertices,Indices);

    AddColorTri(A,E,H,color,Vertices,Indices);
    AddColorTri(H,D,A,color,Vertices,Indices);

    AddColorTri(F,B,C,color,Vertices,Indices);
    AddColorTri(C,G,F,color,Vertices,Indices);
}

void Mesh::InitSphere(std::vector<vr::RenderModel_Vertex_t_rgb> &Vertices, std::vector<u_int32_t> &Indices, float radius, Vector3 color, Vector4 center, int num_lat, int num_lon )
{
    bool smooth=false; //!< If we compute normals per vert, we could get nice smooth shapes.
    if(num_lon<=0){
        /// Default to twice the latitudes, since longitude goes -180 to +180 and latitude only goes -90 to +90
        num_lon=num_lat*2;
    }
    for(int lat=0;lat<num_lat;lat++)
    {
        for(int lon=0;lon<num_lon;lon++)
        {
            float azimuth1=(lon  )/float(num_lon)*M_PI*2;
            float azimuth2=(lon+1)/float(num_lon)*M_PI*2;
            float elevation1=(lat  )/float(num_lat)*M_PI;
            float elevation2=(lat+1)/float(num_lat)*M_PI;

            Vector4 p1=center+sphere2cart(azimuth1,elevation1,radius);
            Vector4 p2=center+sphere2cart(azimuth2,elevation1,radius);
            Vector4 p3=center+sphere2cart(azimuth2,elevation2,radius);
            Vector4 p4=center+sphere2cart(azimuth1,elevation2,radius);

            if(smooth){
                /// The normal could actually just be the point (normalized). Weird, huh?
                Vector4 n1=sphere2cart(azimuth1,elevation1,1.0);
                Vector4 n2=sphere2cart(azimuth2,elevation1,1.0);
                Vector4 n3=sphere2cart(azimuth2,elevation2,1.0);
                Vector4 n4=sphere2cart(azimuth1,elevation2,1.0);
                /// The top and bottom ring have zero size elements. Skip those.
                if(lat!=0){
                    AddColorVertex( p1, n1, color, Vertices, Indices );
                    AddColorVertex( p2, n2, color, Vertices, Indices );
                    AddColorVertex( p3, n3, color, Vertices, Indices );
                }
                if(lat!=num_lat-1){
                    AddColorVertex( p3, n3, color, Vertices, Indices );
                    AddColorVertex( p1, n1, color, Vertices, Indices );
                    AddColorVertex( p4, n4, color, Vertices, Indices );
                }
            }else{
                /// The top and bottom ring have zero size elements. Skip those.
                if(lat!=0){
                    AddColorTri( p3,p2,p1,color,Vertices,Indices);
                }
                if(lat!=num_lat-1){
                    AddColorTri( p3,p1,p4,color,Vertices,Indices);
                }
            }


        }
    }
}

//-----------------------------------------------------------------------------
// Purpose: Create an arror marker
// http://wiki.ros.org/rviz/DisplayTypes/Marker#Arrow_.28ARROW.3D0.29
//-----------------------------------------------------------------------------
void Mesh::InitArrow( std::vector<vr::RenderModel_Vertex_t_rgb> &Vertices, std::vector<u_int32_t> &Indices, Matrix4 mat, float radius_y,float radius_z, float length, Vector3 color, int num_facets )
{
    /// We have the head take up 23.2% of the length. This is what rviz does, and I've no clue why.
    float shaft_len_pct = 0.768;
    /// We have the shaft take up 50% of the radius. This is what rviz does, and it seems sensible.
    float shaft_rad_pct = 0.5;
    /// Arrow goes from 0 to length in X.
    /// Y & Z allow different radii to allow ellipical arrows, even though that's silly.
    Vector4 Top = mat * Vector4( length, 0, 0, 1 );
    Vector4 Bot = mat * Vector4( 0, 0, 0, 1 );
    std::vector<Vector4> Head_Outer_Ring;
    std::vector<Vector4> Head_Inner_Ring;
    std::vector<Vector4> Bot_Ring;
    for(int ii=0;ii<num_facets;ii++){
        float angle = ii*M_PI*2.0/num_facets;
        Vector4 head_outer_vert = mat * Vector4( shaft_len_pct * length,               radius_y*sin(angle),               radius_z*cos(angle), 1 );
        Head_Outer_Ring.push_back(head_outer_vert);
        Vector4 head_inner_vert = mat * Vector4( shaft_len_pct * length, shaft_rad_pct*radius_y*sin(angle), shaft_rad_pct*radius_z*cos(angle), 1 );
        Head_Inner_Ring.push_back(head_inner_vert);
        Vector4 bot_vert        = mat * Vector4(                    0.0, shaft_rad_pct*radius_y*sin(angle), shaft_rad_pct*radius_z*cos(angle), 1 );
        Bot_Ring.push_back(bot_vert);
    }


    for(int ii=0;ii<num_facets;ii++){
        int idx1=ii;
        int idx2=(ii+1)%num_facets;

        //Pointy part of head
        AddColorTri( Top,Head_Outer_Ring[idx1],Head_Outer_Ring[idx2],color,Vertices,Indices);

        //Back of head
        AddColorTri( Head_Inner_Ring[idx1],Head_Inner_Ring[idx2],Head_Outer_Ring[idx2],color,Vertices,Indices);
        AddColorTri( Head_Outer_Ring[idx2],Head_Outer_Ring[idx1],Head_Inner_Ring[idx1],color,Vertices,Indices);

        //Shaft
        AddColorTri( Bot_Ring[idx1],Bot_Ring[idx2],Head_Inner_Ring[idx2],color,Vertices,Indices);
        AddColorTri( Head_Inner_Ring[idx2],Head_Inner_Ring[idx1],Bot_Ring[idx1],color,Vertices,Indices);

        //Bottom Pinwheel
        AddColorTri( Bot,Bot_Ring[idx1],Bot_Ring[idx2],color,Vertices,Indices);

    }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void Mesh::InitCylinder( std::vector<vr::RenderModel_Vertex_t_rgb> &Vertices, std::vector<u_int32_t> &Indices, Matrix4 mat, float radius, float length, Vector3 color, int num_facets )
{

    Vector4 Top = mat * Vector4( 0, 0, -length/2, 1 );
    Vector4 Bot = mat * Vector4( 0, 0,  length/2, 1 );
    std::vector<Vector4> Top_Ring;
    std::vector<Vector4> Bot_Ring;
    for(int ii=0;ii<num_facets;ii++){
        float angle = ii*M_PI*2.0/num_facets;
        Vector4 top_vert = mat * Vector4( radius*sin(angle), radius*cos(angle), -length/2, 1 );
        Top_Ring.push_back(top_vert);
        Vector4 bot_vert = mat * Vector4( radius*sin(angle), radius*cos(angle),  length/2, 1 );
        Bot_Ring.push_back(bot_vert);
    }


    for(int ii=0;ii<num_facets;ii++){
        int idx1=ii;
        int idx2=(ii+1)%num_facets;

        //Top Pinwheel
        AddColorTri( Top,Top_Ring[idx1],Top_Ring[idx2],color,Vertices,Indices);

        //Bottom Pinwheel
        AddColorTri( Bot,Bot_Ring[idx1],Bot_Ring[idx2],color,Vertices,Indices);

        //Side
        AddColorTri( Bot_Ring[idx1],Bot_Ring[idx2],Top_Ring[idx2],color,Vertices,Indices);
        AddColorTri( Top_Ring[idx2],Top_Ring[idx1],Bot_Ring[idx1],color,Vertices,Indices);

    }
}

void Mesh::InitTriangles(std::vector<vr::RenderModel_Vertex_t_rgb> &Vertices, std::vector<u_int32_t> &Indices,Matrix4 mat, Vector3 radius,std::vector<geometry_msgs::Point> &points,std::vector<std_msgs::ColorRGBA> &colors, Vector3 default_color){
    /// If the points aren't a multiple of 3, something is wrong
    assert(points.size()%3==0);
    Vector4 scale(radius.x*2.0,radius.y*2.0,radius.z*2.0,1.0);
    for(int idx=0;idx<points.size()/3;idx++){
        Vector3 color=default_color;
//        if(colors.size()>idx*3){
//            /// Only grabbing one color
//            color.x=colors[idx*3].r;
//            color.y=colors[idx*3].g;
//            color.z=colors[idx*3].b;
//        }
        Vector4 p1 = mat * (Vector4( points[idx*3+0].x, points[idx*3+0].y, points[idx*3+0].z, 1.0 )*scale);
        Vector4 p2 = mat * (Vector4( points[idx*3+1].x, points[idx*3+1].y, points[idx*3+1].z, 1.0 )*scale);
        Vector4 p3 = mat * (Vector4( points[idx*3+2].x, points[idx*3+2].y, points[idx*3+2].z, 1.0 )*scale);

        AddColorTri( p1,p2,p3,color,Vertices,Indices);
    }
}


void Mesh::InitMesh(unsigned int Index, const aiMesh* paiMesh, const aiNode* node)
{

    // Make sure we have a root node
    if (!node) {
      return;
    }

    // We need to fix the orientation
    aiMatrix4x4 transform = node->mTransformation;
    aiNode *pnode = node->mParent;
    while (pnode) {
      // Don't convert to y-up orientation, which is what the root node in
      // Assimp does
      if (pnode->mParent != NULL) {
        transform = pnode->mTransformation * transform;
      }
      pnode = pnode->mParent;
    }
    // Get just the rotation, for transforming the normals
    aiMatrix3x3 rotation(transform);




    m_Entries[Index].MaterialIndex = paiMesh->mMaterialIndex;

    std::vector<u_int32_t> Indices;


    bool use_texture = (paiMesh->mMaterialIndex < m_Textures.size() && m_Textures[paiMesh->mMaterialIndex]);

    if(use_texture){
        const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
        std::vector<vr::RenderModel_Vertex_t> Vertices;

        for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
            aiVector3D pos = paiMesh->mVertices[i];
            aiVector3D n = paiMesh->mNormals[i];



            Vector4 pt;
            if(Z_UP){
                pt.x= pos.x*scale.x;
                pt.z= pos.y*scale.y;
                pt.y=-pos.z*scale.z;
            }else{
                pt.x=pos.x*scale.x;
                pt.y=pos.y*scale.y;
                pt.z=pos.z*scale.z;
            }
            pt.w=1;

            Vector4 pt_trans = trans * pt;

            Vector4 nm;
            if(Z_UP){
                nm.x= n.x;
                nm.z= n.y;
                nm.y=-n.z;
            }else{
                nm.x=n.x;
                nm.y=n.y;
                nm.z=n.z;
            }
            nm.w=0; // Normals are vectors in free space, so w=0 keeps it from being affected by the translation part

            Vector4 nm_trans = trans * nm;


            const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;
            vr::RenderModel_Vertex_t v;
            v.vPosition.v[0]=pt_trans.x;
            v.vPosition.v[1]=pt_trans.y;
            v.vPosition.v[2]=pt_trans.z;
            v.rfTextureCoord[0]=pTexCoord->x;
            v.rfTextureCoord[1]=pTexCoord->y;
            v.vNormal.v[0]=nm_trans.x;
            v.vNormal.v[1]=nm_trans.y;
            v.vNormal.v[2]=nm_trans.z;

            Vertices.push_back(v);
        }
        for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
            const aiFace& Face = paiMesh->mFaces[i];
            if(Face.mNumIndices != 3){
                std::cout << "numfaces not 3, it is " << Face.mNumIndices << std::endl;
            }else{
                Indices.push_back(Face.mIndices[0]);
                Indices.push_back(Face.mIndices[1]);
                Indices.push_back(Face.mIndices[2]);
            }
        }

        m_Entries[Index].Init(Vertices, Indices);

    }else{
        const aiColor4D Zero4D(0.0f, 0.0f, 0.0f, 0.0f);
        std::vector<vr::RenderModel_Vertex_t_rgb> Vertices;

        for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
            aiVector3D pos = paiMesh->mVertices[i];
            aiVector3D n = paiMesh->mNormals[i];



            Vector4 pt;
            if(Z_UP){
                pt.x= pos.x*scale.x;
                pt.z= pos.y*scale.y;
                pt.y=-pos.z*scale.z;
            }else{
                pt.x=pos.x*scale.x;
                pt.y=pos.y*scale.y;
                pt.z=pos.z*scale.z;
            }
            pt.w=1;

            Vector4 pt_trans = trans * pt;

            Vector4 nm;
            if(Z_UP){
                nm.x= n.x;
                nm.z= n.y;
                nm.y=-n.z;
            }else{
                nm.x=n.x;
                nm.y=n.y;
                nm.z=n.z;
            }
            nm.w=0; // Normals are vectors in free space, so w=0 keeps it from being affected by the translation part

            Vector4 nm_trans = trans * nm;


            const aiColor4D* pVertColor = paiMesh->HasVertexColors(0) ? &(paiMesh->mColors[0][i]) : &Zero4D;
            vr::RenderModel_Vertex_t_rgb v;
            v.vPosition.v[0]=pt_trans.x;
            v.vPosition.v[1]=pt_trans.y;
            v.vPosition.v[2]=pt_trans.z;
            v.vColor.v[0]=pVertColor->r;
            v.vColor.v[1]=pVertColor->g;
            v.vColor.v[2]=pVertColor->b;
            v.vNormal.v[0]=nm_trans.x;
            v.vNormal.v[1]=nm_trans.y;
            v.vNormal.v[2]=nm_trans.z;

            Vertices.push_back(v);
        }
        for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
            const aiFace& Face = paiMesh->mFaces[i];
            if(Face.mNumIndices != 3){
                std::cout << "numfaces not 3, it is " << Face.mNumIndices << std::endl;
            }else{
                Indices.push_back(Face.mIndices[0]);
                Indices.push_back(Face.mIndices[1]);
                Indices.push_back(Face.mIndices[2]);
            }
        }

        m_Entries[Index].MaterialIndex=NO_TEXTURE;
        m_Entries[Index].Init(Vertices, Indices);
    }

}

bool Mesh::InitMaterials(const aiScene* pScene, const std::string& Filename)
{
    // Extract the directory part from the file name
    std::string::size_type SlashIndex = Filename.find_last_of("/");
    std::string Dir;

    if (SlashIndex == std::string::npos) {
        Dir = ".";
    }
    else if (SlashIndex == 0) {
        Dir = "/";
    }
    else {
        Dir = Filename.substr(0, SlashIndex);
    }

    bool Ret = true;

    // Initialize the materials
    for (unsigned int i = 0 ; i < pScene->mNumMaterials ; i++) {
        const aiMaterial* pMaterial = pScene->mMaterials[i];

        m_Textures[i] = NULL;

        if (pMaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
            aiString Path;

            if (pMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &Path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS) {
                std::string FullPath = Dir + "/" + Path.data;
                m_Textures[i] = new Texture(GL_TEXTURE_2D, FullPath.c_str());

                if (!m_Textures[i]->Load()) {
                    printf("Error loading texture '%s'\n", FullPath.c_str());
                    delete m_Textures[i];
                    m_Textures[i] = NULL;
                    Ret = false;
                }
                else {
                    printf("Loaded texture '%s'\n", FullPath.c_str());
                }
            }
        }
    }

    return Ret;
}

void Mesh::Render()
{

    for (unsigned int i = 0 ; i < m_Entries.size() ; i++) {

        glBindVertexArray( m_Entries[i].VA );


        const unsigned int MaterialIndex = m_Entries[i].MaterialIndex;

        if (MaterialIndex < m_Textures.size() && m_Textures[MaterialIndex]) {
            m_Textures[MaterialIndex]->Bind(GL_TEXTURE0);
        }

        glDrawElements(GL_TRIANGLES, m_Entries[i].NumIndices, GL_UNSIGNED_INT, 0);
    }

}
