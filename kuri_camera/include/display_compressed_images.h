#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CompressedImage.h"
#include <mutex>
#include <thread>
#include <boost/shared_ptr.hpp>

class DisplayCompressedImages {
  ros::NodeHandle nh;

  ros::Duration delay;

  ros::Subscriber image_sub;

  sensor_msgs::CompressedImage image;
  bool has_new_image;
  std::mutex image_mutex;

  std::thread control_loop_thread;

public:
  DisplayCompressedImages();
  ~DisplayCompressedImages();
  void controlLoop();
  void imageCallback(boost::shared_ptr<sensor_msgs::CompressedImage> msg);
};
