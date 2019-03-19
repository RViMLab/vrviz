#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>

ros::Publisher g_marker_pub;

/// Params
bool silly_shapes = false;
std::string shape_frame = "/vrviz_base";

std::vector<std::string> tf_cache;

visualization_msgs::Marker frame_label(std::string frame_id){
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_text";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.scale.z = 0.015;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.text=frame_id;
    return marker;
}

void publishCallback(const ros::TimerEvent&)
{
    visualization_msgs::MarkerArray msg;
    {
        /// Add some instructions for the demo
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "marker_text";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.7;
        marker.pose.position.z =-0.5;
        marker.scale.z = 0.020;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.text="USE WAND TRIGGER TO DRIVE";
        msg.markers.push_back(marker);
    }
    if(silly_shapes)
    {
        /// Add some silly shapes
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "sphere";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.2;
        marker.pose.position.y = 0.7;
        marker.pose.position.z =-0.2;
        marker.scale.x = 0.10;
        marker.scale.y = 0.10;
        marker.scale.z = 0.10;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r=1.0;
        marker.color.g=0.0;
        marker.color.b=0.0;
        marker.color.a=1.0;
        msg.markers.push_back(marker);
    }
    if(silly_shapes)
    {
        /// Add some silly shapes
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "cube";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x =-0.5;
        marker.pose.position.y = 0.7;
        marker.pose.position.z =-0.5;
        marker.scale.x = 0.3;
        marker.scale.y = 0.5;
        marker.scale.z = 0.7;
        marker.pose.orientation.x = 0.146629;
        marker.pose.orientation.y = 0.311454;
        marker.pose.orientation.z = 0.733143;
        marker.pose.orientation.w = 0.586514;
        marker.color.r=0.0;
        marker.color.g=1.0;
        marker.color.b=0.0;
        marker.color.a=1.0;
        msg.markers.push_back(marker);
    }
    if(silly_shapes)
    {
        /// Add some silly shapes
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.5;
        marker.pose.position.y = 0.7;
        marker.pose.position.z =-0.5;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.5;
        marker.pose.orientation.x = 0.443047;
        marker.pose.orientation.y = 0.235269;
        marker.pose.orientation.z = 0.553809;
        marker.pose.orientation.w = 0.664570;
        marker.color.r=0.0;
        marker.color.g=0.0;
        marker.color.b=1.0;
        marker.color.a=1.0;
        msg.markers.push_back(marker);
    }
    if(silly_shapes)
    {
        /// Add some silly shapes
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "arrow";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = .1;
        marker.pose.position.y = .2;
        marker.pose.position.z =-.7;
        marker.scale.x = 1.0/3.0;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.pose.orientation.x = 0.443047;
        marker.pose.orientation.y = 0.235269;
        marker.pose.orientation.z = 0.553809;
        marker.pose.orientation.w = 0.664570;
        marker.color.r=1.0;
        marker.color.g=0.0;
        marker.color.b=1.0;
        marker.color.a=1.0;
        msg.markers.push_back(marker);
    }
    if(silly_shapes)
    {
        /// Add some silly shapes
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "line";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.5;
        marker.pose.position.y = 0.7;
        marker.pose.position.z =-0.5;
        marker.scale.x = 0.01;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.pose.orientation.x = 0.443047;
        marker.pose.orientation.y = 0.235269;
        marker.pose.orientation.z = 0.553809;
        marker.pose.orientation.w = 0.664570;
        marker.color.r=1.0;
        marker.color.g=0.0;
        marker.color.b=0.0;
        marker.color.a=1.0;
        for(float angle=0.0;angle<2*M_PI;angle+=0.15){
            geometry_msgs::Point pt;
            pt.x = cos(angle)/3.0;
            pt.y = sin(angle)/3.0;
            pt.z = angle/10.0;
            marker.points.push_back(pt);
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = angle/(2*M_PI);
            color.g = 1.0 - angle/(2*M_PI);
            color.b = 0.0;
//            color.r = (sin(angle*234634623.)+1)/2.0;
//            color.g = (sin(angle*454737257.)+1)/2.0;
//            color.b = (sin(angle*372754645.)+1)/2.0;
            marker.colors.push_back(color);
        }
        msg.markers.push_back(marker);
    }
    if(silly_shapes)
    {
        /// Add some silly shapes
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "line";
        marker.id = 2;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.5;
        marker.pose.position.y = 0.7;
        marker.pose.position.z =-0.5;
        marker.scale.x = 0.01;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.pose.orientation.x = 0.443047;
        marker.pose.orientation.y = 0.235269;
        marker.pose.orientation.z = 0.553809;
        marker.pose.orientation.w = 0.664570;
        marker.color.r=1.0;
        marker.color.g=1.0;
        marker.color.b=0.0;
        marker.color.a=1.0;
        for(float angle=0.0;angle<2*M_PI;angle+=0.15){
            geometry_msgs::Point pt;
            pt.x = cos(angle)/4.0;
            pt.y = sin(angle)/4.0;
            pt.z = angle/10.0;
            marker.points.push_back(pt);
        }
        msg.markers.push_back(marker);
    }
    if(silly_shapes)
    {
        /// Add some silly shapes
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "lists";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.5;
        marker.pose.position.y = 0.7;
        marker.pose.position.z =-0.5;
        marker.scale.x = 0.01;
        marker.scale.y = 0.02;
        marker.scale.z = 0.03;
        marker.pose.orientation.x = 0.443047;
        marker.pose.orientation.y = 0.235269;
        marker.pose.orientation.z = 0.553809;
        marker.pose.orientation.w = 0.664570;
        marker.color.r=0.0;
        marker.color.g=1.0;
        marker.color.b=1.0;
        marker.color.a=1.0;
        for(float angle=0.0;angle<2*M_PI;angle+=0.15){
            geometry_msgs::Point pt;
            pt.x = cos(angle)/5.0;
            pt.y = sin(angle)/5.0;
            pt.z = angle/10.0;
            marker.points.push_back(pt);
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0 - angle/(2*M_PI);
            color.g = 1.0 - angle/(2*M_PI);
            color.b = 1.0;
            marker.colors.push_back(color);
        }
        msg.markers.push_back(marker);
    }
    if(silly_shapes)
    {
        /// Add some silly shapes
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "lists";
        marker.id = 2;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.5;
        marker.pose.position.y = 0.7;
        marker.pose.position.z =-0.5;
        marker.scale.x = 0.02;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.pose.orientation.x = 0.443047;
        marker.pose.orientation.y = 0.235269;
        marker.pose.orientation.z = 0.553809;
        marker.pose.orientation.w = 0.664570;
        marker.color.r=0.0;
        marker.color.g=1.0;
        marker.color.b=1.0;
        marker.color.a=1.0;
        for(float angle=0.0;angle<2*M_PI;angle+=0.15){
            geometry_msgs::Point pt;
            pt.x = cos(angle)/6.0;
            pt.y = sin(angle)/6.0;
            pt.z = angle/10.0;
            marker.points.push_back(pt);
        }
        msg.markers.push_back(marker);
    }
    if(silly_shapes)
    {
        /// Add some silly shapes
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "lists";
        marker.id = 3;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.5;
        marker.pose.position.y = 0.7;
        marker.pose.position.z =-0.5;
        marker.scale.x = 0.02;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.pose.orientation.x = 0.443047;
        marker.pose.orientation.y = 0.235269;
        marker.pose.orientation.z = 0.553809;
        marker.pose.orientation.w = 0.664570;
        marker.color.r=0.0;
        marker.color.g=1.0;
        marker.color.b=1.0;
        marker.color.a=1.0;
        for(float angle=0.0;angle<2*M_PI;angle+=0.15){
            geometry_msgs::Point pt;
            pt.x = cos(angle)/8.0;
            pt.y = sin(angle)/8.0;
            pt.z = angle/10.0;
            marker.points.push_back(pt);
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = angle/(2*M_PI);
            color.g = 1.0 - angle/(2*M_PI);
            color.b = 0.0;
            marker.colors.push_back(color);
        }
        msg.markers.push_back(marker);
    }
    /// Send a text marker at the origin of every frame we know about
    for(int idx=0;idx<tf_cache.size();idx++){
        msg.markers.push_back(frame_label(tf_cache[idx]));
    }
    if(silly_shapes)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = shape_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "marker_test_mesh";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.frame_locked = true;
        //marker.mesh_resource = "package://vrviz/meshes/flag.dae";
        marker.mesh_resource = "package://turtlebot_description/meshes/stacks/hexagons/plate_top.dae";
        marker.mesh_use_embedded_materials = true;
        msg.markers.push_back(marker);
    }
  g_marker_pub.publish(msg);

  static tf::TransformBroadcaster br;
  tf::Transform t;

  t.setOrigin(tf::Vector3(-0.1, 0.1, 0.6));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "my_tf_label"));

}

/*!
 * \brief add a frame to the tf cache
 * \param frame_id frame to add
 */
void add_tf_to_cache(std::string frame_id)
{
    if(frame_id.substr(0,10)=="vrviz_base"){return;}
    for(int idx=0;idx<tf_cache.size();idx++){
        if(frame_id==tf_cache[idx]){
            return;
        }
    }
    tf_cache.push_back(frame_id);
}

/*!
 * \brief tf_Callback for subscribing to TF messages
 *
 * We are just sniffing the /tf message and adding any frame_id's
 * we see to our cache.
 * \note that this will miss some frames, e.g. from robot_description
 *
 * \param msg Pointer to the message
 */
void tf_Callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    for(int idx=0;idx<msg->transforms.size();idx++){
        add_tf_to_cache(msg->transforms[idx].header.frame_id);
        add_tf_to_cache(msg->transforms[idx].child_frame_id);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_test");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  ros::Subscriber tf_sub = n.subscribe("/tf", 1, tf_Callback);
  g_marker_pub = n.advertise<visualization_msgs::MarkerArray> ("marker_array", 0);
  ros::Timer publish_timer = n.createTimer(ros::Duration(1), publishCallback);
  pnh.getParam("silly_shapes",silly_shapes);
  pnh.getParam("shape_frame",shape_frame);

  tf::TransformBroadcaster tf_broadcaster;

  ros::Duration(0.1).sleep();

  ros::spin();
}

