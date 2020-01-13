#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

/*
 * This node publishes a pose arrow whose position is at the origin of the
 * upwards_looking_camera_link TF frame, but whose orientation is the same
 * as the base_link frame. In other words, this arrow will be visible in the
 * camera, but will point in the direction Kuri is facing. This allows users
 * know what direction Kuri will move when they press the teleop forward button,
 * independent of what direction the head is facing.
 */
int main(int argc, char** argv){
  ros::init(argc, argv, "display_heading_arrow");

  ros::NodeHandle nh;

  ros::Publisher publisher =
    nh.advertise<geometry_msgs::PoseStamped>("heading", 10);

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "/base_link";
  // The pose will always point in the direction of the base_link
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  tf::TransformListener listener;

  ros::Rate rate(1.0);
  while (ros::ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/base_link", "/upward_looking_camera_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Error during lookupTransform: %s",ex.what());
      ros::Duration(1.0).sleep();
    }

    msg.pose.position.x = transform.getOrigin().x();
    msg.pose.position.y = transform.getOrigin().y();
    msg.pose.position.z = transform.getOrigin().z();

    publisher.publish(msg);

    rate.sleep();
  }
  return 0;
};
