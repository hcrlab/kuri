#include <cassert>
#include <madmux/madmux.h>
#include <ros/ros.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <condition_variable>


struct KuriCameraROSPublisher {
  

  mdx_stream* stream;

  KuriCameraROSPublisher(ros::NodeHandle nh);
  ~KuriCameraROSPublisher();

};

  void stream_callback_thunk(uint8_t *buffer, uint32_t size, void *user_data) {
    ros::Time recv = ros::Time::now();
    ROS_INFO("Got Madmux Data Size %d, Time %f", size, recv.toSec());
    for (uint32_t i = 0; i < 20; i++)
    {
        if (i > 0) printf(":");
        printf("%02X", buffer[i]);
    }
    printf("\n");
    for (uint32_t i = size-20; i < size; i++)
    {
        if (i > size-20) printf(":");
        printf("%02X", buffer[i]);
    }
    printf("\n");
  };

  KuriCameraROSPublisher::KuriCameraROSPublisher(ros::NodeHandle nh) {
    stream = mdx_open("/var/run/madmux/ch1.sock");
    mdx_register_cb(stream, stream_callback_thunk, static_cast<void *>(this));
    //mdx_force_iframe(stream);
  };

  KuriCameraROSPublisher::~KuriCameraROSPublisher() {
    mdx_close(stream);
  };

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "kuri_camera_ros_publisher");
  ros::NodeHandle nh;

  KuriCameraROSPublisher kuriCameraROSPublisher(nh);
  
  ros::spin();
  return 0;
}
