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
  ros::Publisher imagePublisher;

  cv::Mat cvImage;
  bool haveImage;
  boost::shared_mutex cvImageMutex;
  std::mutex waitForDecodedFrameMutex;
  std::mutex waitForCvImageMutex;
  std::condition_variable gotNewCvImage;

  std::thread publishImageThread;
  std::thread cvDisplayThread;
  std::thread getImagesThread;

  ros::Time frameCallbackPreviousTime;

  H264Decoder* decoderObject;

  H264DecoderNode(ros::NodeHandle &nh, H264Decoder* h264Decoder) {
    haveImage = false;

    imagePublisher = nh.advertise<sensor_msgs::Image>("upward_looking_camera/image_raw", 1);

    frameCallbackPreviousTime = ros::Time::now();

    publishImageThread = std::thread(&H264DecoderNode::publishImage, this);
    cvDisplayThread = std::thread(&H264DecoderNode::cvDisplay, this);
    getImagesThread = std::thread(&H264DecoderNode::getImages, this);

    decoderObject = h264Decoder;
  }

  ~H264DecoderNode() {

  }

  void getImages() {
    std::unique_lock<std::mutex> waitForDecodedFrameLock(waitForDecodedFrameMutex);
    while (ros::ok()) {
      (decoderObject->decodedNewFrame).wait(waitForDecodedFrameLock);
      boost::unique_lock<boost::shared_mutex> cvImageLock(cvImageMutex);
      int gotError = decoderObject->getMostRecentFrame(cvImage);
      if (!gotError) {
        haveImage = true;
      }
      cvImageLock.unlock();
      gotNewCvImage.notify_all();
    }
  }


  void publishImage() {
    ros::Rate waitingRate(1);

    std::unique_lock<std::mutex> waitForCvImageLock(waitForCvImageMutex);
    waitForCvImageLock.unlock();

    while (ros::ok()) {
      ros::spinOnce();

      if (imagePublisher.getNumSubscribers() > 0) {
        waitForCvImageLock.lock();
        gotNewCvImage.wait(waitForCvImageLock);
        waitForCvImageLock.unlock();

        boost::shared_lock<boost::shared_mutex> cvImageLock(cvImageMutex);
        sensor_msgs::ImagePtr msg;
        if (haveImage) {
          msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImage).toImageMsg();
          cvImageLock.unlock();
        } else {
          cvImageLock.unlock();
          continue;
        }

        msg->header.stamp = ros::Time::now(); // TODO: can change it so that this is the time the tcp socket receives the image, and that timestamp is given by getMostRecentFrame
        msg->header.frame_id = "upward_looking_camera";

        imagePublisher.publish(msg);
      } else {
        ROS_INFO("No subscribers to upward_looking_camera/image_raw . Not published");
        waitingRate.sleep();
      }
    }
  }

  void cvDisplay() {
    cv::namedWindow("view");

    std::unique_lock<std::mutex> waitForCvImageLock(waitForCvImageMutex);
    waitForCvImageLock.unlock();

    while (ros::ok()) {
      ros::spinOnce();

      waitForCvImageLock.lock();
      gotNewCvImage.wait(waitForCvImageLock);
      waitForCvImageLock.unlock();

      boost::shared_lock<boost::shared_mutex> cvImageLock(cvImageMutex);
      if (haveImage) {
        cv::imshow("view", cvImage);
        cvImageLock.unlock();
      } else {
        cvImageLock.unlock();
        continue;
      }
      cv::waitKey(1);
    }
  }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "h264_decoder_node");
  ros::Time::init();

  ros::NodeHandle nh;

  H264Decoder* h264Decoder = new H264Decoder();
  H264DecoderNode* node = new H264DecoderNode(nh, h264Decoder);

  std::string tcpSocketHostname;
  int tcpSocketPort;
  ros::param::param<std::string>("tcpSocketHostname", tcpSocketHostname, "cococutkuri.personalrobotics.cs.washington.edu");
  ros::param::param<int>("tcpSocketPort", tcpSocketPort, 1234);
  h264Decoder->load(tcpSocketHostname, std::to_string(tcpSocketPort));

  h264Decoder->startRead();
  ros::spin();
  h264Decoder->stopRead();
  free(h264Decoder);
  return 0;
}
