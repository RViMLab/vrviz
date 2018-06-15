#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

ros::Publisher g_marker_pub;

void publishCallback(const ros::TimerEvent&)
{
  visualization_msgs::MarkerArray msg;
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/vive";
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
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_tf_label";
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
    marker.text="my_tf_label";
    msg.markers.push_back(marker);
  }
//  {
//    visualization_msgs::Marker marker;
//    marker.header.frame_id = "/base_link";
//    marker.header.stamp = ros::Time::now();
//    marker.ns = "marker_test_mesh_color_change";
//    marker.id = 0;
//    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = 0.0;
//    marker.pose.position.y = 0.0;
//    marker.pose.position.z = 1.0;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 1.0;
//    marker.pose.position.x = x_pos;
//    marker.scale.x = 1.0;
//    marker.scale.y = 1.0;
//    marker.scale.z = 1.0;
//    marker.color.r = float(counter % 255) / 255;
//    marker.color.g = float((counter*3) % 255) / 255;
//    marker.color.b = float((counter*10) % 255) / 255;
//    marker.color.a = 1.0;
//    marker.frame_locked = true;
//    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
//    marker.mesh_use_embedded_materials = false;
//    msg.markers.push_back(marker);
//  }
  g_marker_pub.publish(msg);

  static tf::TransformBroadcaster br;
  tf::Transform t;

  t.setOrigin(tf::Vector3(-0.1, 0.1, 0.6));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "my_tf_label"));

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_test");
  ros::NodeHandle n;

  g_marker_pub = n.advertise<visualization_msgs::MarkerArray> ("marker_array", 0);
  ros::Timer publish_timer = n.createTimer(ros::Duration(1), publishCallback);

  tf::TransformBroadcaster tf_broadcaster;

  ros::Duration(0.1).sleep();

  ros::spin();
}

