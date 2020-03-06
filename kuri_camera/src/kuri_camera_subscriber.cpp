#include "ros/ros.h"
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

struct KuriCameraSubscriber {
  image_transport::ImageTransport transport;
  image_transport::CameraPublisher publisher;
  sensor_msgs::CameraInfo camera_info;

  cv::Mat cv_image;
  ros::Time cv_image_timestamp;
  std::vector<bool> got_new_cv_image;
  boost::shared_mutex cv_image_mutex;

  ros::Subscriber compressed_image_subscriber;

  std::vector<void (*)(int)> workerFunctions;
  size_t numWorkerFunctions;
  std::vector<std::thread> workerThreads;

  int loop_rate_hz;

  KuriCameraSubscriber(ros::NodeHandle &nh) : transport(nh) {

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

    loop_rate_hz = 30; // how frequently to re-check if there is a new image

    publisher = transport.advertiseCamera("upward_looking_camera", 1);

    workerFunctions.push_back(&KuriCameraSubscriber::publishImage);
    workerFunctions.push_back(&KuriCameraSubscriber::cvDisplay);
    // workerFunctions.push_back(&KuriCameraSubscriber::sampleFunctionForUsingImages);

    numWorkerFunctions = workerFunctions.size();
    for (int threadI = 0; threadI < numWorkerFunctions; threadI++) {
      workerThreads.push_back(std::thread(workerFunctions[threadI], this, threadI));
      got_new_cv_image.push_back(false);
    }

    // We must initialize got_new_cv_image with the right size before subscribing
    compressed_image_subscriber = nh.subscribe("upward_looking_camera/compressed", 1, &KuriCameraSubscriber::compressedImageCallback, this);

  }

  ~KuriCameraSubscriber() {
    numWorkerFunctions = workerFunctions.size();
    for (int threadI = 0; threadI < numWorkerFunctions; threadI++) {
      workerThreads[threadI].join();
    }
  }

  void compressedImageCallback(const sensor_msgs::CompressedImage &msg) {
    {
      boost::unique_lock<boost::shared_mutex> cv_image_lock(cv_image_mutex);
      cv_image = cv::imdecode(cv::Mat(msg.data),cv::IMREAD_UNCHANGED); //convert compressed image data to cv::Mat
      cv_image_timestamp = msg.header.stamp;
      for (int i = 0; i < numWorkerFunctions; i++) {
        got_new_cv_image[i] = true;
      }
    }
  }


  void publishImage(int threadI) {
    ros::Rate waiting_rate(0.3); // If there are no subscribers
    ros::Rate loop_rate(loop_rate_hz); // If there are subscribers

    sensor_msgs::ImagePtr msg;

    while (ros::ok()) {

      // Because publishing ros Images is expensive, only do so when we have a
      // subscriber
      if (publisher.getNumSubscribers() > 0) {
        {
          boost::shared_lock<boost::shared_mutex> cv_image_lock(cv_image_mutex);
          if (got_new_cv_image[threadI]) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
            msg->header.stamp = cv_image_timestamp;
            got_new_cv_image[threadI] = false;
            cv_image_lock.unlock();

            msg->header.frame_id = "upward_looking_camera_optical_frame";
            publisher.publish(*msg, camera_info, msg->header.stamp);
          } else {
            cv_image_lock.unlock();
            loop_rate.sleep();
          }
        }
      } else {
        ROS_INFO("No subscribers to upward_looking_camera . Not published");
        waiting_rate.sleep();
      }
    }
  }

  void cvDisplay(int threadI) {
    ros::Rate loop_rate(loop_rate_hz);

    cv::namedWindow("view");

    while (ros::ok()) {
      {
        boost::shared_lock<boost::shared_mutex> cv_image_lock(cv_image_mutex);
        if (got_new_cv_image[threadI]) {
          cv::imshow("view", cv_image);
          got_new_cv_image[threadI] = false;
          cv_image_lock.unlock();
          cv::waitKey(1);
        } else {
          cv_image_lock.unlock();
          loop_rate.sleep()
        }
      }
    }
  }

  void sampleFunctionForUsingImages(int threadI) {
    ros::Rate loop_rate(loop_rate_hz);

    // Loop forever while ROS is running
    while (ros::ok()) {
      {
        // Put a *read lock* on the cv_image.
        // NOTE: If you will be writing to the cv_image, use a boost::unique_lock.
        // However, that is not recommended because then other threads will not be
        // able to use the unedited cv_image
        boost::shared_lock<boost::shared_mutex> cv_image_lock(cv_image_mutex);
        // Check if we have a new message available for this thread
        if (got_new_cv_image[threadI]) {

          // PROCESS and/or READ THE IMAGE HERE

          got_new_cv_image[threadI] = false;
          cv_image_lock.unlock();

          // DO ANY POST-READ PROCESSING HERE

        } else {
          cv_image_lock.unlock();
          loop_rate.sleep(); // Sleep before re-checking if an image is available
        }
      }
    }
  }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "kuri_camera_subscriber");
  ros::Time::init();

  ros::NodeHandle nh(ros::this_node::getName());

  KuriCameraSubscriber node(nh);

  ros::spin();

  return 0;
}
