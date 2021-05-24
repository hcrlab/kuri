#ifndef MOBILE_BASE_UTILS_MOBILE_BASE_UTILS_H
#define MOBILE_BASE_UTILS_MOBILE_BASE_UTILS_H


#include <mobile_base_driver/Sensors.h>
#include <mobile_base_driver/Bumper.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace kuri_navigation {

  int bumperToPointCloud(const mobile_base_driver::Sensors& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    cloud->header.frame_id = "base_footprint";
    pcl_conversions::toPCL(msg.header.stamp, cloud->header.stamp);
    for (int i = 0; i < msg.bumper.size(); i++) {
      mobile_base_driver::Bumper bumper_state = msg.bumper[i];
      if (bumper_state.state == mobile_base_driver::Bumper::RELEASED) {
        continue;
      }
      float x, y;
      switch (bumper_state.bumper) {
        case mobile_base_driver::Bumper::LEFT:
          x = 0;
          y = 0.21;
          break;
        case mobile_base_driver::Bumper::CENTER:
          x = 0.21;
          y = 0;
          break;
        case mobile_base_driver::Bumper::RIGHT:
          x = 0;
          y = -0.21;
          break;
      }
      cloud->points.emplace_back(x, y , 0);
    }
    return cloud->points.size();
  }

  void growBumpCloudByRadius(double radius, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    int original_num = cloud->size();
  }
}


#endif
