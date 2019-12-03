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
#include <sensor_msgs/CompressedImage.h>

struct KuriCameraROSPublisher {

  ros::Publisher imagePublisher;
  mdx_stream *stream;

  std::mutex imageMutex;
  bool hasNewImage;
  cv::Mat imageMat;
  ros::Time recvTime;

  // You can read about this pattern in the boost docs:
  // http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?Generalizing_C-Style_Callbacks
  static void stream_callback_thunk(uint8_t *buffer, uint32_t size, void *self) {
    // Use the user data pointer as a pointer to an instance of the Publisher, then call
    // the nice callback on it.
    static_cast<KuriCameraROSPublisher *>(self)->data_callback(buffer, size);
  }

  KuriCameraROSPublisher(ros::NodeHandle &nh) {
    hasNewImage = false;
    imagePublisher = nh.advertise<sensor_msgs::CompressedImage>("upward_looking_camera/image_raw/compressed", 1);
    stream = mdx_open("/var/run/madmux/ch3.sock");

    mdx_register_cb(stream, stream_callback_thunk, static_cast<void *>(this));
  }

  ~KuriCameraROSPublisher() {
    mdx_close(stream);
  }

  void listen_publish_loop() {
    ros::Rate sub_poll(1);
    ros::Rate fps(60);
    cv::Mat image;
    ros::Time timeForMsg;
    while (ros::ok()) {
      ros::spinOnce();
      fps.sleep();
      if (imagePublisher.getNumSubscribers() == 0) {
        ROS_INFO("Waiting for subscribers");
        sub_poll.sleep();
        continue;
      }

      std::unique_lock<std::mutex> imageLock(imageMutex);
      if (hasNewImage) {
        hasNewImage = false;
        image = cv::imdecode(imageMat, CV_LOAD_IMAGE_UNCHANGED);
        timeForMsg = recvTime;
        imageLock.unlock();
        sensor_msgs::CompressedImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toCompressedImageMsg();
        msg->header.stamp = timeForMsg;
        msg->header.frame_id = "upward_looking_camera";
        imagePublisher.publish(msg);
      } else {
        imageLock.unlock();
      }
    }
  }

  void data_callback(uint8_t *buffer, uint32_t size) {
    std::unique_lock<std::mutex> imageLock(imageMutex);
    imageMat = cv::Mat(1, size, CV_8UC1, buffer).clone();
    recvTime = ros::Time::now();
    hasNewImage = true;
    imageLock.unlock();
  }
};

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "kuri_camera_ros_publisher");
  ros::NodeHandle nh;
  KuriCameraROSPublisher wrapper(nh);
  wrapper.listen_publish_loop();
}
