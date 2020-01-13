#include "ros/ros.h"
#include "h264_decoder.h"
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <boost/thread/shared_mutex.hpp>
#include <boost/range/algorithm.hpp>

struct H264DecoderNode {
  image_transport::ImageTransport transport;
  image_transport::CameraPublisher publisher;
  sensor_msgs::CameraInfo camera_info;

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

  H264DecoderNode(ros::NodeHandle &nh, H264Decoder* h264_decoder) : transport(ros::NodeHandle()) {
    have_image = false;

    // Load the camera_info parameters
    int camera_info_width, camera_info_height;
    std::string camera_info_distortion_model;
    std::vector<double> camera_info_D;
    std::vector<double> camera_info_K;
    std::vector<double> camera_info_R;
    std::vector<double> camera_info_P;
    if (!nh.getParam("image_width", camera_info_width)) ROS_WARN("Camera Calibration: image_width not set");
    if (!nh.getParam("image_height", camera_info_height)) ROS_WARN("Camera Calibration: image_height not set");
    if (!nh.getParam("camera_model", camera_info_distortion_model)) ROS_WARN("Camera Calibration: camera_model not set");
    if (!nh.getParam("distortion_coefficients/data", camera_info_D)) ROS_WARN("Camera Calibration: distortion_coefficients/data not set");
    if (!nh.getParam("rectification_matrix/data", camera_info_R)) ROS_WARN("Camera Calibration: rectification_matrix/data not set");
    if (!nh.getParam("projection_matrix/data", camera_info_P)) ROS_WARN("Camera Calibration: projection_matrix/data not set");
    // Construct K from P -- valid since the camera is monocular
    if (camera_info_P.size() == 12) {
      camera_info_K.insert(camera_info_K.end(), camera_info_P.begin(), camera_info_P.begin() + 3);
      camera_info_K.insert(camera_info_K.end(), camera_info_P.begin() + 4, camera_info_P.begin() + 7);
      camera_info_K.insert(camera_info_K.end(), camera_info_P.begin() + 8, camera_info_P.begin() + 11);

      // Only copy data over if the params were set
      camera_info.header.frame_id = "upward_looking_camera_optical_frame";
      camera_info.width = camera_info_width;
      camera_info.height = camera_info_height;
      camera_info.distortion_model = camera_info_distortion_model;
      camera_info.D = camera_info_D;
      boost::range::copy(camera_info_K, camera_info.K.begin());
      boost::range::copy(camera_info_R, camera_info.R.begin());
      boost::range::copy(camera_info_P, camera_info.P.begin());
    }

    publisher = transport.advertiseCamera("upward_looking_camera", 1);

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
      if (publisher.getNumSubscribers() > 0) {
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

        ros::Time frame_recv_time = ros::Time(((double)cv_image_recv_timestamp)/1000.0); // When the H264 frame was received;

        msg->header.stamp = frame_recv_time;
        msg->header.frame_id = "upward_looking_camera_optical_frame";

        publisher.publish(*msg, camera_info, frame_recv_time);
      } else {
        ROS_INFO("No subscribers to upward_looking_camera . Not published");
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

  ros::NodeHandle nh(ros::this_node::getName());

  H264Decoder* h264_decoder = new H264Decoder();
  H264DecoderNode* node = new H264DecoderNode(nh, h264_decoder);

  std::string tcp_socket_hostname;
  int tcp_socket_port;
  if (!nh.getParam("/tcp_socket_hostname", tcp_socket_hostname)) {
    ROS_ERROR("No hostname set at param /tcp_socket_hostname . Exiting.");
    return -1;
  }
  if (!nh.getParam("/tcp_socket_port", tcp_socket_port)) {
    ROS_ERROR("No port set at param /tcp_socket_port . Exiting.");
    return -1;
  }
  h264_decoder->load(tcp_socket_hostname, std::to_string(tcp_socket_port));

  h264_decoder->startRead();
  ros::spin();
  h264_decoder->stopRead();

  free(h264_decoder);
  return 0;
}
