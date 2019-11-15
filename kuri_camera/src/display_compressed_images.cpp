#include "display_compressed_images.h"

DisplayCompressedImages::DisplayCompressedImages() {

  image_sub = nh.subscribe("/upward_looking_camera/image_raw/compressed", 1, &DisplayCompressedImages::imageCallback, this);

  has_new_image = false;

  control_loop_thread = std::thread(&DisplayCompressedImages::controlLoop, this);
}

DisplayCompressedImages::~DisplayCompressedImages() {
  control_loop_thread.join();
}

void DisplayCompressedImages::controlLoop() {
  ros::Rate control_loop_rate(100.0);

  cv::namedWindow("view");
  // cv::startWindowThread();

  sensor_msgs::CompressedImage image_in_control_loop;
  cv::Mat cv_image;

  while (!ros::isShuttingDown()) {
    control_loop_rate.sleep();

    std::unique_lock<std::mutex> image_lock(image_mutex);
    if (has_new_image) {
      image_in_control_loop = image;
      has_new_image = false;
      image_lock.unlock();
      try {
        cv_image = cv::imdecode(cv::Mat(image_in_control_loop.data),1);//convert compressed image data to cv::Mat
        // ROS_INFO("Before imshow");
        cv::imshow("view", cv_image);
        // ROS_INFO("Before waitKey");
        cv::waitKey(1);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert to image!");
      }
    } else {
      image_lock.unlock();
    }

  }

  cv::destroyWindow("view");

}

void DisplayCompressedImages::imageCallback(boost::shared_ptr<sensor_msgs::CompressedImage> msg) {
  std::unique_lock<std::mutex> image_lock(image_mutex);
  image = *(msg.get());
  has_new_image = true;
  delay = ros::Time::now() - image.header.stamp;
  image_lock.unlock();
  ROS_INFO("Recv image, delay %f", delay.toSec());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "display_compressed_image");
  DisplayCompressedImages displayCompressedImages;
  ros::spin();
  return 0;
}
