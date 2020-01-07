#include "ros/ros.h"
#include "h264_decoder.h"
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <mutex>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

struct H264DecoderNode {
  ros::Publisher imagePublisher;

  cv::Mat cvImage;
  bool haveImage;
  std::mutex cvImageMutex;
  std::mutex waitForFirstFrameMutex;
  std::mutex waitForNewCvImageMutex;
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
    std::unique_lock<std::mutex> waitForFirstFrameLock(waitForFirstFrameMutex);
    // ROS_INFO("getImages pre wait for first frame");
    // (decoderObject->decodedNewFrame).wait(waitForFirstFrameLock);
    // ROS_INFO("getImages post wait for first frame");
    // ros::Rate haveGottenFirstFrameRate(1);
    while (ros::ok()) {
      // ROS_INFO("getImages pre wait for first frame");
      (decoderObject->decodedNewFrame).wait(waitForFirstFrameLock);
      // ROS_INFO("getImages post wait for first frame");
      // ROS_INFO("getImages, %s", bool(decoderObject->haveGottenFirstFrame) ? "true" : "false");
      // if (!bool(decoderObject->haveGottenFirstFrame)) {
      //   ROS_INFO("About to sleep getImages");
      //   haveGottenFirstFrameRate.sleep();
      //   continue;
      // }
      // ROS_INFO("getImages lock cvImageLock");
      std::unique_lock<std::mutex> cvImageLock(cvImageMutex);
      // ROS_INFO("getImages post acquire cvImageLock");
      int gotError = decoderObject->getMostRecentFrame(cvImage);
      if (!gotError) {
        haveImage = true;
      }
      cvImageLock.unlock();
      gotNewCvImage.notify_all();
      // ROS_INFO("getImages unlock cvImageLock");
    }
  }


  void publishImage() {
    std::unique_lock<std::mutex> waitForFirstFrameLock(waitForFirstFrameMutex);
    ROS_INFO("publishImage pre wait for first frame");
    (decoderObject->decodedNewFrame).wait(waitForFirstFrameLock);
    waitForFirstFrameLock.unlock();
    ROS_INFO("publishImage post wait for first frame");
    ros::Rate waitingRate(1);

    std::unique_lock<std::mutex> waitForNewCvImageLock(waitForNewCvImageMutex);
    waitForNewCvImageLock.unlock();

    while (ros::ok()) {
      ros::spinOnce();
      // ROS_INFO("publishImage, %s", bool(decoderObject->haveGottenFirstFrame) ? "true" : "false");
      // if (!bool(decoderObject->haveGottenFirstFrame)) {
	    //   ROS_INFO("About to sleep publishImage");
      //   waitingRate.sleep();
	    //   continue;
      // }

      // uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      // std::cout << "got cv::Mat " << *((int*)user) << " " << now << std::endl;
      if (imagePublisher.getNumSubscribers() > 0) {
        waitForNewCvImageLock.lock();
        gotNewCvImage.wait(waitForNewCvImageLock);
        waitForNewCvImageLock.unlock();

        //std::unique_lock<std::mutex> copyFrameLock(copyFrameMutex);
        std::unique_lock<std::mutex> cvImageLock(cvImageMutex);
        sensor_msgs::ImagePtr msg;
        if (haveImage) {
          msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImage).toImageMsg();
          cvImageLock.unlock();
        } else {
          cvImageLock.unlock();
          continue;
        }

        msg->header.stamp = ros::Time::now(); // TODO: can change it so that this is the time the tcp socket receives the image, and that timestamp is passed by changing the callback signature
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
    std::unique_lock<std::mutex> waitForFirstFrameLock(waitForFirstFrameMutex);
    // ROS_INFO("cvDisplay pre wait for first frame");
    (decoderObject->decodedNewFrame).wait(waitForFirstFrameLock);
    waitForFirstFrameLock.unlock();
    // ROS_INFO("cvDisplay post wait for first frame");
    // ros::Rate haveGottenFirstFrameRate(1);

    std::unique_lock<std::mutex> waitForNewCvImageLock(waitForNewCvImageMutex);
    waitForNewCvImageLock.unlock();

    while (ros::ok()) {
      ros::spinOnce();
      // ROS_INFO("cvDisplay, %s", bool(decoderObject->haveGottenFirstFrame) ? "true" : "false");
      // if (!bool(decoderObject->haveGottenFirstFrame)) {
      //   ROS_INFO("About to sleep cvDisplay");
      //   haveGottenFirstFrameRate.sleep();
      //   continue;
      // }

      // ROS_INFO("cvDisplay pre acquire cvImageLock");

      waitForNewCvImageLock.lock();
      gotNewCvImage.wait(waitForNewCvImageLock);
      waitForNewCvImageLock.unlock();

      std::unique_lock<std::mutex> cvImageLock(cvImageMutex);
      // ROS_INFO("cvDisplay post acquire cvImageLock");
      if (haveImage) {
        // ROS_INFO("pre imshow");
        cv::imshow("view", cvImage);
        // ROS_INFO("post imshow");
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

  ROS_INFO("Before creating H264Decoder");
  H264Decoder* h264Decoder = new H264Decoder();
  // // ROS_INFO("Have gotten first frame %s", bool(h264Decoder->haveGottenFirstFrame) ? "true" : "false");
  ROS_INFO("Before creating H264DecoderNode");
  H264DecoderNode* node = new H264DecoderNode(nh, h264Decoder);

  std::string tcpSocketHostname;
  int tcpSocketPort;
  ros::param::param<std::string>("tcpSocketHostname", tcpSocketHostname, "cococutkuri.personalrobotics.cs.washington.edu");
  ros::param::param<int>("tcpSocketPort", tcpSocketPort, 1234);
  ROS_INFO("Before H264Decoder load hostname %s, port %d", tcpSocketHostname.c_str(), tcpSocketPort);
  h264Decoder->load(tcpSocketHostname, std::to_string(tcpSocketPort));

  ROS_INFO("Before H264Decoder startRead");
  h264Decoder->startRead();
  // ROS_INFO("Before ros spin");
  ros::spin();
  // ROS_INFO("Before H264Decoder stopRead");
  h264Decoder->stopRead();
  free(h264Decoder);
  return 0;
}
