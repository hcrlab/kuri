#include <mobile_base_driver/Sensors.h>
#include <ros/ros.h>
#include <kuri_navigation/mobile_base_utils.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/publisher.h>
pcl_ros::Publisher<pcl::PointXYZ> pub;

void callback(mobile_base_driver::Sensors msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>};
  int num_bumps = kuri_navigation::bumperToPointCloud(msg, cloud);
  pub.publish(cloud);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "bumper_to_pointcloud");
  auto nh = ros::NodeHandle();
  auto sub = nh.subscribe("mobile_base/sensors", 1, callback);
  pub = pcl_ros::Publisher<pcl::PointXYZ>(nh, "mobile_base/sensors/bumper_cloud", 1);

  ros::spin();

  return(0);
}
