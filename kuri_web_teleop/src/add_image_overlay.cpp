#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CompressedImage.h"
#include "mobile_base_driver/Power.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include <mutex>
#include <thread>
#include <boost/shared_ptr.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <boost/range/algorithm.hpp>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/imgproc.hpp>


#define HEADING_FRAME_A "camera_rendering_frame_a"
// #define HEADING_FRAME_B "camera_rendering_frame_b"

/*
 * TODO (amal): fix the naming of variables! I have some variables which are the same
 * name as global and local variables, but in camel case versus underscore case,
 * which is awful!
 */

class AddImageOverlay {
  ros::NodeHandle nh;

  ros::Duration delay;

  ros::Subscriber image_sub;
  ros::Subscriber power_sub;
  ros::Subscriber scan_sub;
  ros::Subscriber joint_sub;

  // Image transport variables
  image_transport::ImageTransport transport;
  image_transport::CameraPublisher publisher;
  image_transport::CameraPublisher publisher_original;
  sensor_msgs::CameraInfo camera_info;

  // to be removed
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;

  sensor_msgs::CompressedImage image;
  bool has_new_image;
  std::mutex image_mutex;
  ros::Time lastImageRecvTimestamp;

  std::mutex battery_mutex;
  int battery_percent;
  bool is_charging;

  std::mutex scan_mutex;
  double min_distance;
  double min_distance_threshold = 0.5;
  double min_distance_radians;

  std::mutex joints_mutex;
  double curr_pan = 0;
  double curr_tilt = 0;

  double lagThreshold = 1.0; // seconds

  std::thread control_loop_thread;

public:
  AddImageOverlay();
  ~AddImageOverlay();
  void controlLoop();
  void imageCallback(boost::shared_ptr<sensor_msgs::CompressedImage> msg);
  void powerCallback(boost::shared_ptr<mobile_base_driver::Power> msg);
  void scanCallback(boost::shared_ptr<sensor_msgs::LaserScan> msg);
  void jointsCallback(boost::shared_ptr<sensor_msgs::JointState> msg);
};


AddImageOverlay::AddImageOverlay() : transport(nh) {

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

    cam_model_.fromCameraInfo(camera_info);
  }

  publisher = transport.advertiseCamera("upward_looking_camera_overlay", 1);
  publisher_original = transport.advertiseCamera("upward_looking_camera_original", 1);

  image_sub = nh.subscribe("/upward_looking_camera/compressed", 1, &AddImageOverlay::imageCallback, this);
  power_sub = nh.subscribe("/mobile_base/power", 1, &AddImageOverlay::powerCallback, this);
  scan_sub = nh.subscribe("/scan", 1, &AddImageOverlay::scanCallback, this);
  joint_sub = nh.subscribe("/joint_states", 1, &AddImageOverlay::jointsCallback, this);

  has_new_image = false;

  control_loop_thread = std::thread(&AddImageOverlay::controlLoop, this);
}

AddImageOverlay::~AddImageOverlay() {
  control_loop_thread.join();
}

// Repeatedly updates the republished image based on the latest data
// pulled from the above subscribers
void AddImageOverlay::controlLoop() {
  ros::Rate control_loop_rate(100.0);

  bool shouldShowImage;

  cv::namedWindow("view");
  // cv::startWindowThread();

  sensor_msgs::CompressedImage image_in_control_loop;
  cv::Mat cv_image, cv_image_original;
  bool areDimensionsSet = false;
  int width, height;
  std::string lagText = "Lag";
  int lagTextFont = cv::FONT_HERSHEY_PLAIN;
  int lagTextFontScale = 24;
  int lagTextFontThickness = 40;
  int lagTextBaseline=0;
  cv::Size lagTextSize = cv::getTextSize(lagText, lagTextFont, lagTextFontScale, lagTextFontThickness, &lagTextBaseline);

  while (!ros::isShuttingDown()) {
    control_loop_rate.sleep();
    shouldShowImage = false;
    // ROS_INFO("controlLoop %f", ros::Time::now().toSec());

    std::unique_lock<std::mutex> image_lock(image_mutex);
    // ROS_INFO("in control loop %f", (ros::Time::now() - lastImageRecvTimestamp).toSec());
    if (has_new_image) {
      image_in_control_loop = image;
      has_new_image = false;
      image_lock.unlock();
      try {
        cv_image = cv::imdecode(cv::Mat(image_in_control_loop.data),1);//convert compressed image data to cv::Mat
        cv_image_original = cv_image.clone();
        // cv_image_original = cv::Mat(cv_image.size(),cv_image.type());
        // cv_image.copyTo(cv_image_original);
        shouldShowImage = true;
        if (!areDimensionsSet) {
          width = cv_image.cols;
          height = cv_image.rows;
          areDimensionsSet = true;
        }

        // Draw the Battery Outline
        int batteryWidth = width*1/10;
        int batteryHeight = height*1/10;
        int batteryOffset = width*1/100;
        int topPartWidth = width*1/100;
        int batteryStartX = width-batteryOffset-topPartWidth-batteryWidth;
        int batteryStartY = batteryOffset;
        cv::Point topLeft(batteryStartX, batteryStartY);
        cv::Point bottomRight(batteryStartX+batteryWidth, batteryStartY+batteryHeight);
        cv::Scalar outlineColor(0, 0, 0);
        cv::rectangle(cv_image, topLeft, bottomRight, outlineColor, 3);

        // Draw the small thing at the top
        cv::Point topPartTopLeft(batteryStartX+batteryWidth, batteryStartY+batteryHeight/2-batteryHeight/6);
        cv::Point topPartBottomRight(batteryStartX+batteryWidth+topPartWidth, batteryStartY+batteryHeight/2+batteryHeight/6);
        cv::rectangle(cv_image, topPartTopLeft, topPartBottomRight, outlineColor, -1);

        int offsetForBar = batteryWidth/20;
        int batteryPercent;
        bool isCharging;
        {
          std::unique_lock<std::mutex> battery_lock(battery_mutex);
          batteryPercent = battery_percent;
          isCharging = is_charging;
          battery_lock.unlock();
        }
        int barWidth = (batteryWidth - offsetForBar*2)*batteryPercent/100;
        int barHeight = batteryHeight - offsetForBar*2;
        int barStartX = batteryStartX + offsetForBar;
        int barStartY = batteryStartY + offsetForBar;
        cv::Point barTopLeft(barStartX, barStartY);
        cv::Point barBottomRight(barStartX+barWidth, barStartY+barHeight);
        cv::Scalar barColor;
        if (batteryPercent >= 40) {
          barColor = cv::Scalar(0, 255, 0);
        } else if (batteryPercent >= 20) {
          barColor = cv::Scalar(0, 255, 255);
        } else {
          barColor = cv::Scalar(0, 0, 255);
        }

        cv::rectangle(cv_image, barTopLeft, barBottomRight, barColor, -1);

        // Draw lightning bolt
        if (isCharging) {
          int lightningOffset = batteryWidth/7;
          int lightningStartX = batteryStartX + lightningOffset;
          int lightningStartY = batteryStartY + lightningOffset;
          int lightningWidth = batteryWidth - lightningOffset*2;
          int lightningHeight = batteryHeight - lightningOffset*2;

          std::vector<cv::Point> lightningBolt;
          lightningBolt.push_back(cv::Point(lightningStartX, lightningStartY+lightningHeight*1/4));
          lightningBolt.push_back(cv::Point(lightningStartX+lightningWidth*3/5, lightningStartY+lightningHeight));
          lightningBolt.push_back(cv::Point(lightningStartX+lightningWidth*3/5, lightningStartY+lightningHeight*3/5));
          lightningBolt.push_back(cv::Point(lightningStartX+lightningWidth, lightningStartY+lightningHeight*3/4));
          lightningBolt.push_back(cv::Point(lightningStartX+lightningWidth*2/5, lightningStartY));
          lightningBolt.push_back(cv::Point(lightningStartX+lightningWidth*2/5, lightningStartY+lightningHeight*2/5));
          cv::Mat lightningBoltMat(lightningBolt);
          const cv::Point *pts = (const cv::Point*)lightningBoltMat.data;
          int npts = lightningBoltMat.rows;

          cv::polylines(cv_image, &pts, &npts, 1, true, cv::Scalar(0, 0, 0), 3);
        }

        tf::StampedTransform transformA, transformB;
        try {
          ros::Time acquisition_time = lastImageRecvTimestamp;
          ros::Duration timeout(1.0);
          // tf_listener_.waitForTransform(HEADING_FRAME, cam_model_.tfFrame(),
          //                               lastImageRecvTimestamp, timeout);
          tf_listener_.lookupTransform(cam_model_.tfFrame(), HEADING_FRAME_A,
                                       ros::Time(0), transformA);
          // tf_listener_.lookupTransform(cam_model_.tfFrame(), HEADING_FRAME_B,
          //                              ros::Time(0), transformB);
        }
        catch (tf::TransformException& ex) {
          ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
          return;
        }

        tf::Point ptA = transformA.getOrigin();
        // tf::Point ptB = transformB.getOrigin();
        cv::Point3d pt_cv_a(ptA.x(), ptA.y(), ptA.z());
        // cv::Point3d pt_cv_b(ptB.x(), ptB.y(), ptB.z());

        // ROS_INFO("point A x %f", (float) pt_cv_a.x);
        // ROS_INFO("point A y %f", (float) pt_cv_a.y);
        // ROS_INFO("point A z %f", (float) pt_cv_a.z);

        cv::Point3d line_start_a(ptA.x(), ptA.y() + 2, ptA.z());
        // cv::Point3d line_start_b(ptB.x(), ptB.y() + 2, ptB.z());

        // ROS_INFO("start A x %f", (float) line_start_a.x);
        // ROS_INFO("start A y %f", (float) line_start_a.y);
        // ROS_INFO("start A z %f", (float) line_start_a.z);

        cv::Point2d uv_a, uv_b;
        uv_a = cam_model_.project3dToPixel(pt_cv_a);
        // uv_b = cam_model_.project3dToPixel(pt_cv_b);

        // ROS_INFO("proj A x %f", (float) uv_a.x);
        // ROS_INFO("proj A y %f", (float) uv_a.y);

        cv::Point2d line_start_a_2d, line_start_b_2d;
        line_start_a_2d = cam_model_.project3dToPixel(line_start_a);
        // line_start_b_2d = cam_model_.project3dToPixel(line_start_b);

        // ROS_INFO("start A x %f", (float) line_start_a_2d.x);
        // ROS_INFO("start A y %f", (float) line_start_a_2d.y);

        
        // ROS_INFO("point x %f", (float) uv.x);
        // ROS_INFO("point y %f", (float) uv.y);

        cv::arrowedLine(cv_image, line_start_a_2d, uv_a, CV_RGB(255,0,0), 3, 8, 0, 0.02); 
        // cv::arrowedLine(cv_image, line_start_b_2d, uv_b, CV_RGB(255,0,0), 3, 8, 0, 0.02); 

        // double minDistance;
        // double minDistanceRadians;
        // {
        //   std::unique_lock<std::mutex> scan_lock(scan_mutex);
        //   minDistance = min_distance;
        //   minDistanceRadians = min_distance_radians;
        //   scan_lock.unlock();
        // }        
        
        // Draw the lidar proximity warning
        // if (minDistance < min_distance_threshold) {
        //   // TODO (amal): this assumes the head is centered. Instead, where the
        //   // red appears should be relative to the head orientation!
        //   double minRad = -0.785;
        //   double maxRad = 0.785;
        //   double relativeLocation = (std::min(std::max(minRad, minDistanceRadians), maxRad) - minRad)/(maxRad-minRad);
        //   // ROS_INFO("relativeLocation in scan %f", relativeLocation);
        //   int distanceWarningCenterX = relativeLocation*width;
        //   int distanceWarningCenterY = height;
        //   int distanceCenterWidth = width*1/10;
        //   int distanceCenterHeight = height*1/10;
        //   cv::rectangle(cv_image, cv::Point(distanceWarningCenterX-distanceCenterWidth/2,distanceWarningCenterY-distanceCenterHeight/2), cv::Point(distanceWarningCenterX+distanceCenterWidth/2,distanceWarningCenterY+distanceCenterHeight/2), cv::Scalar(0,0,255), -1);
        // }
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert to image!");
      }
    } else {
      // If there is sufficient delay between last recieved image and `now`, overlay LAG on the screen
      if ((ros::Time::now() - lastImageRecvTimestamp).toSec() >= lagThreshold && areDimensionsSet) {
        image_lock.unlock();
        try {
          cv::Point lagTextBottomLeft(width/2-lagTextSize.width/2, height/2+lagTextSize.height/2);
          cv::putText(cv_image, lagText, lagTextBottomLeft, lagTextFont, lagTextFontScale, cv::Scalar(0,0,255), lagTextFontThickness);
          shouldShowImage = true;
        } catch (cv_bridge::Exception& e) {
          ROS_ERROR("Could not convert to image!");
        }
      } else {
        image_lock.unlock();
      }
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
    sensor_msgs::ImagePtr msg_original = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_original).toImageMsg();
    {
      std::unique_lock<std::mutex> image_lock(image_mutex);
      msg->header.stamp = lastImageRecvTimestamp;
      msg_original->header.stamp = lastImageRecvTimestamp;
      image_lock.unlock();
    }
    msg->header.frame_id = "upward_looking_camera_optical_frame";
    msg_original->header.frame_id = "upward_looking_camera_optical_frame";
    publisher.publish(*msg, camera_info, msg->header.stamp);
    publisher_original.publish(*msg_original, camera_info, msg_original->header.stamp);

    try {
      if (false /*shouldShowImage*/) {
        cv::imshow("view", cv_image);
        cv::waitKey(1);
      }
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert to image!");
    }

  }

  cv::destroyWindow("view");

}

// 
//
// vvvvvv SUBSCRIBER CALLBACKS vvvvv
//
//

void AddImageOverlay::imageCallback(boost::shared_ptr<sensor_msgs::CompressedImage> msg) {
  // ROS_INFO("imageCallback %f", ros::Time::now().toSec());
  std::unique_lock<std::mutex> image_lock(image_mutex);
  lastImageRecvTimestamp = image.header.stamp;
  // ROS_INFO("image header stamp %f", image.header.stamp.toSec());
  image = *(msg.get());
  has_new_image = true;
  delay = ros::Time::now() - image.header.stamp;
  image_lock.unlock();
  // ROS_INFO("Recv image, delay %f", delay.toSec());
}

void AddImageOverlay::powerCallback(boost::shared_ptr<mobile_base_driver::Power> msg) {
  std::unique_lock<std::mutex> battery_lock(battery_mutex);
  battery_percent = msg->battery.rounded_pct;
  is_charging = msg->dock_present;
  battery_lock.unlock();
}

void AddImageOverlay::scanCallback(boost::shared_ptr<sensor_msgs::LaserScan> msg) {
  double minDistance = 10.0; // max is 6.75
  double distance;
  double minDistanceRadians;
  for (int i = 0; i < msg->ranges.size(); i++) {
    distance = msg->ranges[i];
    if (!std::isnan(distance) && !std::isinf(distance)) {
      if (distance <= minDistance) {
        minDistance = distance;
        minDistanceRadians = msg->angle_min + i*msg->angle_increment;
      }
    }
  }
  std::unique_lock<std::mutex> scan_lock(scan_mutex);
  min_distance = minDistance;
  min_distance_radians = minDistanceRadians;
  scan_lock.unlock();
}

void AddImageOverlay::jointsCallback(boost::shared_ptr<sensor_msgs::JointState> msg) {
  double pan = -1;
  double tilt = -1;
  for (int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "head_1_joint") {
      pan = msg->position[i];
    }
    if (msg->name[i] == "head_2_joint") {
      tilt = msg->position[i];
    }
  }

  std::unique_lock<std::mutex> joints_lock(joints_mutex);
  curr_pan = pan;
  curr_tilt = tilt;
  joints_lock.unlock();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_image_overlay");
  AddImageOverlay addImageOverlay;
  ros::spin();
  return 0;
}
