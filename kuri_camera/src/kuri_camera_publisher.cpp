#include <madmux/madmux.h>
#include <ros/ros.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <thread>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/client.h>
#include <compressed_image_transport/CompressedPublisherConfig.h>
#include <boost/range/algorithm.hpp>


struct KuriCameraPublisher {

  // Image transport variables
  image_transport::ImageTransport transport;
  image_transport::CameraPublisher publisher;
  sensor_msgs::CameraInfo camera_info;

  // madmux variables
  mdx_stream *stream;

  // synchronization variables
  std::mutex image_mutex;
  bool has_new_image;
  cv::Mat image_mat;
  ros::Time recv_time;

  // dynamic reconfigure variables
  dynamic_reconfigure::Client<compressed_image_transport::CompressedPublisherConfig> dynamic_reconfigure_client;
  int initial_jpeg_quality;

  std::mutex jpeg_quality_mutex;
  int jpeg_quality;

  KuriCameraPublisher(ros::NodeHandle &nh) : transport(nh), dynamic_reconfigure_client("/upward_looking_camera/compressed") {

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
    if (!nh.getParam("camera_matrix/data", camera_info_K)) ROS_WARN("Camera Calibration: camera_matrix/data not set");
    if (!nh.getParam("distortion_coefficients/data", camera_info_D)) ROS_WARN("Camera Calibration: distortion_coefficients/data not set");
    if (!nh.getParam("rectification_matrix/data", camera_info_R)) ROS_WARN("Camera Calibration: rectification_matrix/data not set");
    if (!nh.getParam("projection_matrix/data", camera_info_P)) ROS_WARN("Camera Calibration: projection_matrix/data not set");
    // Construct K from P -- valid since the camera is monocular
    if (camera_info_P.size() == 12) {
      // Only copy data over if the params were set
      camera_info.header.frame_id = "upward_looking_camera_optical_frame";
      camera_info.width = camera_info_width;
      camera_info.height = camera_info_height;
      camera_info.distortion_model = camera_info_distortion_model;
      camera_info.D = camera_info_D; // D is variable length whereas the below ones are fixed length
      boost::range::copy(camera_info_K, camera_info.K.begin());
      boost::range::copy(camera_info_R, camera_info.R.begin());
      boost::range::copy(camera_info_P, camera_info.P.begin());
    }

    publisher = transport.advertiseCamera("upward_looking_camera", 1);

    // Set the initial jpeg quality
    nh.param<int>("initial_jpeg_quality", initial_jpeg_quality, 20);
    std::thread set_initial_jpeg_quality_thread = std::thread(&KuriCameraPublisher::setInitialJpegQuality, this);
    set_initial_jpeg_quality_thread.detach();

    has_new_image = false;

    // initialize the madmux stream
    stream = mdx_open("/var/run/madmux/ch3.sock");
    mdx_set_resolution(stream, 1280, 720);
    mdx_register_cb(stream, stream_callback_thunk, static_cast<void *>(this));
  }

  ~KuriCameraPublisher() {
    // close the madmux stream
    mdx_close(stream);
  }

  void setInitialJpegQuality() {
    // Set the initial jpeg quality to the value specified by the param
    compressed_image_transport::CompressedPublisherConfig config;
    config.jpeg_quality = initial_jpeg_quality;
    config.format = "jpeg";
    dynamic_reconfigure_client.setConfiguration(config);
  }

  // You can read about this pattern in the boost docs:
  // http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?Generalizing_C-Style_Callbacks
  static void stream_callback_thunk(uint8_t *buffer, uint32_t size, void *self) {
    // Use the user data pointer as a pointer to an instance of the Publisher,
    // then call the nice callback on it.
    static_cast<KuriCameraPublisher *>(self)->data_callback(buffer, size);
  }

  // receive the madmux byes and convert them to a cv::Mat
  void data_callback(uint8_t *buffer, uint32_t size) {
    ros::Time now = ros::Time::now();
    {
      std::unique_lock<std::mutex> image_lock(image_mutex);
      recv_time = now;
      image_mat = cv::Mat(1, size, CV_8UC1, buffer).clone();
      has_new_image = true;
    }
  }

  // take the madmux bytes, convert them to a ROS Image, and publish it
  void control_loop() {
    ros::Rate sub_poll(1); // rate to wait at if the camera has no subscribers
    ros::Rate fps(30); // fps of the loop

    cv::Mat image;
    ros::Time time_for_msg;

    while (ros::ok()) {
      ros::spinOnce();
      fps.sleep();
      if (publisher.getNumSubscribers() == 0) {
        ROS_INFO("Waiting for subscribers");
        sub_poll.sleep();
        continue;
      }

      {
        std::unique_lock<std::mutex> image_lock(image_mutex);
        if (has_new_image) {
          has_new_image = false;
          image = cv::imdecode(image_mat, CV_LOAD_IMAGE_UNCHANGED);
          time_for_msg = recv_time;
          image_lock.unlock();

          sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
          msg->header.frame_id = "upward_looking_camera_optical_frame";
          msg->header.stamp = time_for_msg;
          publisher.publish(*msg, camera_info, msg->header.stamp);
        }
      }
    }
  }
};

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "kuri_camera_ros_publisher");

  ros::NodeHandle nh;
  KuriCameraPublisher wrapper(nh);

  wrapper.control_loop();
}
