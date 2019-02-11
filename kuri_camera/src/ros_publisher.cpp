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

using std::string;
using std::vector;


struct KuriCameraROSPublisher {

  image_transport::ImageTransport transport;
  image_transport::CameraPublisher publisher;
  mdx_stream *stream;
  sensor_msgs::CameraInfo::Ptr camera_info;
  std::mutex m;
  std::condition_variable got_frame;
  bool frame_processed;

  // You can read about this pattern in the boost docs:
  // http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?Generalizing_C-Style_Callbacks
  static void stream_callback_thunk(uint8_t *buffer, uint32_t size, void *self) {
    // Use the user data pointer as a pointer to an instance of the Publisher, then call
    // the nice callback on it.
    static_cast<KuriCameraROSPublisher *>(self)->data_callback(buffer, size);
  }

  KuriCameraROSPublisher(const ros::NodeHandle &nh) : transport(nh), publisher(transport.advertiseCamera("/blah", 1)),
                                                      camera_info(new sensor_msgs::CameraInfo()), frame_processed(false), stream(nullptr) {
    stream = mdx_open("/var/run/madmux/ch4.sock");
    mdx_register_cb(stream, stream_callback_thunk, static_cast<void *>(this));
  }

  ~KuriCameraROSPublisher() {
    mdx_close(stream);
  }

  void listen_publish_loop() {
    ros::Rate sub_poll(1);
    while (ros::ok()) {
      ros::spinOnce();
      if (publisher.getNumSubscribers() == 0) {
        ROS_INFO("No subs");
        sub_poll.sleep();
        //continue;
      }
      ROS_INFO("Heard sub");
      // Someone is subscribed! Get a frame
      {
        std::unique_lock<std::mutex> lock(m);
        frame_processed = false;
        got_frame.wait(lock, [this]() { return frame_processed; });
        ROS_ERROR("LOOP SEES PROCESSED");
      }


    }
  }

  void data_callback(uint8_t *buffer, uint32_t size) {
    std::unique_lock<std::mutex> lock(m);
    std::cout << size << std::endl;
    std::cout << (void*)buffer << std::endl;
    std::cout << buffer[0] << std::endl;

    if (size < 1000) {
      return;
    }
    cv::Mat image_nv12(640, 480, CV_8UC1, buffer);
    cv::Mat image_bgr;
    cv::cvtColor(image_nv12, image_bgr, CV_YUV2BGR_YV12);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_bgr).toImageMsg();
    publisher.publish(msg, camera_info);
    frame_processed = true;
    lock.unlock();
    got_frame.notify_all();
    ROS_ERROR("CALLBACK RETURNING");
  }

};

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "kuri_camera_ros_publisher");
  ros::NodeHandle nh;
  KuriCameraROSPublisher wrapper(nh);
  wrapper.listen_publish_loop();
}