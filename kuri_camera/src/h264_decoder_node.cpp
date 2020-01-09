#include "ros/ros.h"
#include "h264_decoder.h"
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <boost/thread/shared_mutex.hpp>

struct H264DecoderNode {
  ros::Publisher image_publisher;

  cv::Mat cv_image;
  uint64_t cv_image_recv_timestamp;
  bool have_image;
  boost::shared_mutex cv_image_mutex;
  std::mutex wait_for_decoded_frame_mutex;
  std::mutex wait_for_cv_image_mutex;
  std::condition_variable got_new_cv_image;

  std::thread publish_image_thread;
  std::thread cv_display_thread;
  std::thread get_images_thread;
  // std::thread sample_function_thread;

  H264Decoder* decoder_object;

  H264DecoderNode(ros::NodeHandle &nh, H264Decoder* h264_decoder) {
    have_image = false;

    image_publisher = nh.advertise<sensor_msgs::Image>("upward_looking_camera/image_raw", 1);

    publish_image_thread = std::thread(&H264DecoderNode::publishImage, this);
    cv_display_thread = std::thread(&H264DecoderNode::cvDisplay, this);
    get_images_thread = std::thread(&H264DecoderNode::getImages, this);
    // sample_function_thread = std::thread(&H264DecoderNode::sampleFunctionForUsingImages, this);

    decoder_object = h264_decoder;
  }

  ~H264DecoderNode() {
    publish_image_thread.join();
    cv_display_thread.join();
    get_images_thread.join();
    // sample_function_thread.join();
  }

  void getImages() {
    std::unique_lock<std::mutex> wait_for_decoded_frame_lock(wait_for_decoded_frame_mutex);
    while (ros::ok()) {
      (decoder_object->decoded_new_frame).wait(wait_for_decoded_frame_lock);
      boost::unique_lock<boost::shared_mutex> cv_image_lock(cv_image_mutex);
      int go_error = decoder_object->getMostRecentFrame(cv_image, cv_image_recv_timestamp);

      /* // Uncomment these to print processing latency
      uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      ROS_INFO("From receiving the frame to having it availble as a cv::Mat, it took %lu ms", now-cv_image_recv_timestamp);
      */

      if (!go_error) {
        have_image = true;
      }
      cv_image_lock.unlock();
      got_new_cv_image.notify_all();
    }
  }


  void publishImage() {
    ros::Rate waiting_rate(0.3);

    std::unique_lock<std::mutex> wait_for_cv_image_lock(wait_for_cv_image_mutex);
    wait_for_cv_image_lock.unlock();

    while (ros::ok()) {

      // Because publishing ros Images is expensive, only do so when we have a
      // subscriber
      if (image_publisher.getNumSubscribers() > 0) {
        wait_for_cv_image_lock.lock();
        got_new_cv_image.wait(wait_for_cv_image_lock);
        wait_for_cv_image_lock.unlock();

        boost::shared_lock<boost::shared_mutex> cv_image_lock(cv_image_mutex);
        sensor_msgs::ImagePtr msg;
        if (have_image) {
          msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
          cv_image_lock.unlock();
        } else {
          cv_image_lock.unlock();
          continue;
        }

        msg->header.stamp = ros::Time(((double)cv_image_recv_timestamp)/1000.0); // When the H264 frame was received
        msg->header.frame_id = "upward_looking_camera";

        image_publisher.publish(msg);
      } else {
        ROS_INFO("No subscribers to upward_looking_camera/image_raw . Not published");
        waiting_rate.sleep();
      }
    }
  }

  void cvDisplay() {
    cv::namedWindow("view");

    std::unique_lock<std::mutex> wait_for_cv_image_lock(wait_for_cv_image_mutex);
    wait_for_cv_image_lock.unlock();

    while (ros::ok()) {

      wait_for_cv_image_lock.lock();
      got_new_cv_image.wait(wait_for_cv_image_lock);
      wait_for_cv_image_lock.unlock();

      boost::shared_lock<boost::shared_mutex> cv_image_lock(cv_image_mutex);
      if (have_image) {
        cv::imshow("view", cv_image);
        cv_image_lock.unlock();
      } else {
        cv_image_lock.unlock();
        continue;
      }
      cv::waitKey(1);
    }
  }

  void sampleFunctionForUsingImages() {
    // Create the lock that will be used with a condition_variable to determine
    // when the getImages() function has acquired a new image
    std::unique_lock<std::mutex> wait_for_cv_image_lock(wait_for_cv_image_mutex);
    wait_for_cv_image_lock.unlock();

    // Loop forever while ROS is running
    while (ros::ok()) {
      // Wait for getImages() to receive a new image
      wait_for_cv_image_lock.lock();
      got_new_cv_image.wait(wait_for_cv_image_lock);
      wait_for_cv_image_lock.unlock();

      // Put a *read lock* on the cv_image.
      // NOTE: If you will be writing to the cv_image, use a boost::unique_lock.
      // However, that is not recommended because then other threads will not be
      // able to use the unedited cv_image
      boost::shared_lock<boost::shared_mutex> cv_image_lock(cv_image_mutex);
      if (have_image) {

        // PROCESS and/or READ THE IMAGE HERE

        cv_image_lock.unlock();
      } else {
        cv_image_lock.unlock();
        continue;
      }
    }
  }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "h264_decoder_node");
  ros::Time::init();

  ros::NodeHandle nh;

  H264Decoder* h264_decoder = new H264Decoder();
  H264DecoderNode* node = new H264DecoderNode(nh, h264_decoder);

  std::string tcp_socket_hostname;
  int tc_socket_port;
  ros::param::param<std::string>("tcp_socket_hostname", tcp_socket_hostname, "cococutkuri.personalrobotics.cs.washington.edu");
  ros::param::param<int>("tc_socket_port", tc_socket_port, 1234);
  h264_decoder->load(tcp_socket_hostname, std::to_string(tc_socket_port));

  h264_decoder->startRead();
  ros::spin();
  h264_decoder->stopRead();

  free(h264_decoder);
  return 0;
}
