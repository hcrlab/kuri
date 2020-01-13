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

  ros::Publisher image_publisher;
  mdx_stream *stream;

  std::mutex image_mutex;
  bool has_new_image;
  cv::Mat image_mat;
  ros::Time recv_time;

  // You can read about this pattern in the boost docs:
  // http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?Generalizing_C-Style_Callbacks
  static void stream_callback_thunk(uint8_t *buffer, uint32_t size, void *self) {
    // Use the user data pointer as a pointer to an instance of the Publisher, then call
    // the nice callback on it.
    static_cast<KuriCameraROSPublisher *>(self)->data_callback(buffer, size);
  }

  KuriCameraROSPublisher(ros::NodeHandle &nh) {
    has_new_image = false;
    image_publisher = nh.advertise<sensor_msgs::CompressedImage>("upward_looking_camera/image_raw/compressed", 1);
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
    ros::Time time_for_msg;
    while (ros::ok()) {
      ros::spinOnce();
      fps.sleep();
      if (image_publisher.getNumSubscribers() == 0) {
        ROS_INFO("Waiting for subscribers");
        sub_poll.sleep();
        continue;
      }

      std::unique_lock<std::mutex> image_lock(image_mutex);
      if (has_new_image) {
        has_new_image = false;
        image = cv::imdecode(image_mat, CV_LOAD_IMAGE_UNCHANGED);
        time_for_msg = recv_time;
        image_lock.unlock();
        sensor_msgs::CompressedImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toCompressedImageMsg();
        msg->header.stamp = time_for_msg;
        msg->header.frame_id = "upward_looking_camera";
        image_publisher.publish(msg);
      } else {
        image_lock.unlock();
      }
    }
  }

  void data_callback(uint8_t *buffer, uint32_t size) {
    std::unique_lock<std::mutex> image_lock(image_mutex);
    image_mat = cv::Mat(1, size, CV_8UC1, buffer).clone();
    recv_time = ros::Time::now();
    has_new_image = true;
    image_lock.unlock();
  }
};

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "kuri_camera_ros_publisher");
  ros::NodeHandle nh;
  KuriCameraROSPublisher wrapper(nh);
  wrapper.listen_publish_loop();
}
