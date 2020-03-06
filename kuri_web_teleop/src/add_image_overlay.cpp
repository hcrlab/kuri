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
#include <math.h>

#define HEADING_FRAME_A "camera_rendering_frame_a"

class AddImageOverlay {
  ros::NodeHandle nh;

  // Used to debug
  ros::Duration delay;

  // Subscribers
  ros::Subscriber image_sub;
  ros::Subscriber power_sub;
  ros::Subscriber scan_sub;
  ros::Subscriber joint_sub;

  // Image transport variables
  image_transport::ImageTransport transport;
  image_transport::CameraPublisher publisher;
  image_transport::CameraPublisher publisher_original;
  sensor_msgs::CameraInfo camera_info;

  // Used to draw the heading arrow on the screen
  tf::TransformListener tf_listener;
  image_geometry::PinholeCameraModel cam_model;

  // Concurrency literals for the compressed image
  sensor_msgs::CompressedImage image;
  bool has_new_image;
  std::mutex image_mutex;
  ros::Time last_image_recv_timestamp;

  // Concurrency literals for Kuri's battery level
  std::mutex battery_mutex;
  int battery_percent;
  bool is_charging;

  // Concurrency literals for the laser scan data
  std::mutex scan_mutex;
  double min_distance;
  double min_distance_threshold = 0.5;
  double min_distance_radians;

  // Concurrency literals for Kuri's joint data
  std::mutex joints_mutex;
  double curr_pan = 0;
  double curr_tilt = 0;

  // The amount of acceptable lab before displaying the word "Lag" on the screen
  double lag_threshold = 1.0; // seconds

  // The control loop thread
  std::thread control_loop_thread;

public:
  AddImageOverlay();
  ~AddImageOverlay();

private:
  void loadCameraInfo();
  void controlLoop();

  // Subscriber callbacks
  void imageCallback(boost::shared_ptr<sensor_msgs::CompressedImage> msg);
  void powerCallback(boost::shared_ptr<mobile_base_driver::Power> msg);
  void scanCallback(boost::shared_ptr<sensor_msgs::LaserScan> msg);
  void jointsCallback(boost::shared_ptr<sensor_msgs::JointState> msg);

  // Draw overlays onto the image
  void addBatteryToImage(cv::Mat cv_image, int width, int height);
  void drawHeadingArrow(cv::Mat cv_image);
  void drawClosestObstacleIndicator(cv::Mat cv_image, int width, int height);
};


AddImageOverlay::AddImageOverlay() : transport(nh) {

  loadCameraInfo();

  // Create the publishers.
  // publisher_original is necessary to force receiving topics to use the
  // compressed image. Else, some code (e.g. SE2LAM) will subscribe to the
  // non-compressed image transport, greatly increasing the latency
  publisher = transport.advertiseCamera("upward_looking_camera_overlay", 1);
  publisher_original = transport.advertiseCamera("upward_looking_camera_original", 1);

  // Create the subscribers
  image_sub = nh.subscribe("/upward_looking_camera/compressed", 1, &AddImageOverlay::imageCallback, this);
  power_sub = nh.subscribe("/mobile_base/power", 1, &AddImageOverlay::powerCallback, this);
  scan_sub = nh.subscribe("/scan", 1, &AddImageOverlay::scanCallback, this);
  joint_sub = nh.subscribe("/joint_states", 1, &AddImageOverlay::jointsCallback, this);

  // Start the control loop
  has_new_image = false;
  control_loop_thread = std::thread(&AddImageOverlay::controlLoop, this);
}

AddImageOverlay::~AddImageOverlay() {
  // Wait for the control loop to terminate before ending
  control_loop_thread.join();
}

void AddImageOverlay::loadCameraInfo() {
  // Load the camera_info parameters
  int camera_info_width, camera_info_height;
  std::string camera_info_distortion_model;
  std::vector<double> camera_info_D;
  std::vector<double> camera_info_K;
  std::vector<double> camera_info_R;
  std::vector<double> camera_info_P;
  if (!nh.getParam("image_width", camera_info_width)) {
    ROS_WARN("Camera Calibration: image_width not set");
  }
  if (!nh.getParam("image_height", camera_info_height)) {
    ROS_WARN("Camera Calibration: image_height not set");
  }
  if (!nh.getParam("camera_model", camera_info_distortion_model)) {
    ROS_WARN("Camera Calibration: camera_model not set");
  }
  if (!nh.getParam("camera_matrix/data", camera_info_K)) {
    ROS_WARN("Camera Calibration: camera_matrix/data not set");
  }
  if (!nh.getParam("distortion_coefficients/data", camera_info_D)) {
    ROS_WARN("Camera Calibration: distortion_coefficients/data not set");
  }
  if (!nh.getParam("rectification_matrix/data", camera_info_R)) {
    ROS_WARN("Camera Calibration: rectification_matrix/data not set");
  }
  if (!nh.getParam("projection_matrix/data", camera_info_P)) {
    ROS_WARN("Camera Calibration: projection_matrix/data not set");
  }
  // Copy the param values into the camera_info message
  if (camera_info_P.size() == 12) {
    camera_info.header.frame_id = "upward_looking_camera_optical_frame";
    camera_info.width = camera_info_width;
    camera_info.height = camera_info_height;
    camera_info.distortion_model = camera_info_distortion_model;
    camera_info.D = camera_info_D; // D is variable length whereas the below ones are fixed length
    boost::range::copy(camera_info_K, camera_info.K.begin());
    boost::range::copy(camera_info_R, camera_info.R.begin());
    boost::range::copy(camera_info_P, camera_info.P.begin());

    cam_model.fromCameraInfo(camera_info);
  }
}

// Repeatedly updates the republished image based on the latest data
// pulled from the subscribers
void AddImageOverlay::controlLoop() {
  ros::Rate control_loop_rate(100.0);

  bool should_show_image;

  cv::namedWindow("view");
  // cv::startWindowThread();

  sensor_msgs::CompressedImage image_in_control_loop;
  cv::Mat cv_image, cv_image_original;
  bool are_dimensions_set = false;
  int width, height;
  std::string lag_text = "Lag";
  int lag_text_font = cv::FONT_HERSHEY_PLAIN;
  int lag_text_font_scale = 24;
  int lag_text_font_thickness = 40;
  int lag_text_baseline=0;
  cv::Size lag_text_size = cv::getTextSize(lag_text, lag_text_font, lag_text_font_scale, lag_text_font_thickness, &lag_text_baseline);

  while (!ros::isShuttingDown()) {
    control_loop_rate.sleep();
    should_show_image = false;

    std::unique_lock<std::mutex> image_lock(image_mutex);
    if (has_new_image) {
      // Get the latest compressed image, and uncompress it
      image_in_control_loop = image;
      has_new_image = false;
      image_lock.unlock();
      try {
        cv_image = cv::imdecode(cv::Mat(image_in_control_loop.data),1); // convert compressed image data to cv::Mat
        cv_image_original = cv_image.clone();
        should_show_image = true;
        if (!are_dimensions_set) {
          width = cv_image.cols;
          height = cv_image.rows;
          are_dimensions_set = true;
        }

        addBatteryToImage(cv_image, width, height);

        drawHeadingArrow(cv_image);

        // NOTE: Drawing a red rectangle in the direction of the closest obstacle
        // did not work too well (we also never fully developed/debugged it),
        // so we implemented an HTML-based approach instead.
        // drawClosestObstacleIndicator(cv_image, width, height);

      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert to image!");
      }
    } else {
      // If there is sufficient delay between last recieved image and now,
      // overlay LAG on the screen
      if ((ros::Time::now() - last_image_recv_timestamp).toSec() >= lag_threshold && are_dimensions_set) {
        image_lock.unlock();
        try {
          cv::Point lag_text_bottom_left(width/2-lag_text_size.width/2, height/2+lag_text_size.height/2);
          cv::putText(cv_image, lag_text, lag_text_bottom_left, lag_text_font, lag_text_font_scale, cv::Scalar(0,0,255), lag_text_font_thickness);
          should_show_image = true;
        } catch (cv_bridge::Exception& e) {
          ROS_ERROR("Could not convert to image!");
        }
      } else {
        image_lock.unlock();
      }
    }

    // Publish the image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
    sensor_msgs::ImagePtr msg_original = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_original).toImageMsg();
    {
      std::unique_lock<std::mutex> image_lock(image_mutex);
      msg->header.stamp = last_image_recv_timestamp;
      msg_original->header.stamp = last_image_recv_timestamp;
      image_lock.unlock();
    }
    msg->header.frame_id = "upward_looking_camera_optical_frame";
    msg_original->header.frame_id = "upward_looking_camera_optical_frame";
    publisher.publish(*msg, camera_info, msg->header.stamp);
    publisher_original.publish(*msg_original, camera_info, msg_original->header.stamp);

    // // Show the image
    // try {
    //   if (should_show_image) {
    //     cv::imshow("view", cv_image);
    //     cv::waitKey(1);
    //   }
    // } catch (cv_bridge::Exception& e) {
    //   ROS_ERROR("Could not convert to image!");
    // }
  }

  cv::destroyWindow("view");

}

void AddImageOverlay::addBatteryToImage(cv::Mat cv_image, int width, int height) {
  // Draw the Battery Outline
  int battery_width = width*1/10;
  int battery_height = height*1/10;
  int battery_offset = width*1/100;
  int top_part_width = width*1/100;
  int battery_start_x = width-battery_offset-top_part_width-battery_width;
  int battery_start_y = battery_offset;
  cv::Point top_left(battery_start_x, battery_start_y);
  cv::Point bottom_right(battery_start_x+battery_width, battery_start_y+battery_height);
  cv::Scalar outline_color(0, 0, 0);
  cv::rectangle(cv_image, top_left, bottom_right, outline_color, 3);

  // Draw the small bump at the top of the batter
  cv::Point top_part_top_left(battery_start_x+battery_width, battery_start_y+battery_height/2-battery_height/6);
  cv::Point top_part_bottom_right(battery_start_x+battery_width+top_part_width, battery_start_y+battery_height/2+battery_height/6);
  cv::rectangle(cv_image, top_part_top_left, top_part_bottom_right, outline_color, -1);

  // Draw the battery level
  int offset_for_bar = battery_width/20;
  int battery_percent_local;
  bool is_charging_local;
  // Get the most up-to-date batter level
  {
    std::unique_lock<std::mutex> battery_lock(battery_mutex);
    battery_percent_local = battery_percent;
    is_charging_local = is_charging;
    battery_lock.unlock();
  }
  int bar_width = (battery_width - offset_for_bar*2)*battery_percent_local/100;
  int bar_height = battery_height - offset_for_bar*2;
  int bar_start_x = battery_start_x + offset_for_bar;
  int bar_start_y = battery_start_y + offset_for_bar;
  cv::Point bar_top_left(bar_start_x, bar_start_y);
  cv::Point bar_bottom_right(bar_start_x+bar_width, bar_start_y+bar_height);
  cv::Scalar bar_color;
  if (battery_percent_local >= 40) {
    bar_color = cv::Scalar(0, 255, 0);
  } else if (battery_percent_local >= 20) {
    bar_color = cv::Scalar(0, 255, 255);
  } else {
    bar_color = cv::Scalar(0, 0, 255);
  }
  cv::rectangle(cv_image, bar_top_left, bar_bottom_right, bar_color, -1);

  // If the battery is_charging_local, draw the lightning bolt
  if (is_charging_local) {
    int lightning_offset = battery_width/7;
    int lightning_start_x = battery_start_x + lightning_offset;
    int lightning_start_y = battery_start_y + lightning_offset;
    int lightning_width = battery_width - lightning_offset*2;
    int lightning_height = battery_height - lightning_offset*2;

    std::vector<cv::Point> lightning_bolt;
    lightning_bolt.push_back(cv::Point(lightning_start_x, lightning_start_y+lightning_height*1/4));
    lightning_bolt.push_back(cv::Point(lightning_start_x+lightning_width*3/5, lightning_start_y+lightning_height));
    lightning_bolt.push_back(cv::Point(lightning_start_x+lightning_width*3/5, lightning_start_y+lightning_height*3/5));
    lightning_bolt.push_back(cv::Point(lightning_start_x+lightning_width, lightning_start_y+lightning_height*3/4));
    lightning_bolt.push_back(cv::Point(lightning_start_x+lightning_width*2/5, lightning_start_y));
    lightning_bolt.push_back(cv::Point(lightning_start_x+lightning_width*2/5, lightning_start_y+lightning_height*2/5));
    cv::Mat lightning_bolt_mat(lightning_bolt);
    const cv::Point *pts = (const cv::Point*)lightning_bolt_mat.data;
    int npts = lightning_bolt_mat.rows;

    cv::Scalar color(0, 0, 0);
    int thickness = 3;

    cv::polylines(cv_image, &pts, &npts, 1, true, color, thickness);
  }
}

void AddImageOverlay::drawHeadingArrow(cv::Mat cv_image) {
  tf::StampedTransform transform_a;
  tf::StampedTransform transform_b;
  try {
    tf_listener.lookupTransform(cam_model.tfFrame(), HEADING_FRAME_A,
                                 ros::Time(0), transform_a);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
    return;
  }

  tf::Point pt_a = transform_a.getOrigin();
  cv::Point3d pt_cv_a(pt_a.x(), pt_a.y(), pt_a.z());

  cv::Point3d line_start_a(pt_a.x(), pt_a.y() + 2, pt_a.z());

  cv::Point2d line_start;
  line_start = cam_model.project3dToPixel(line_start_a);

  cv::Point2d line_end;
  line_end = cam_model.project3dToPixel(pt_cv_a);

  cv::Scalar color = CV_RGB(255,0,0);
  const int thickness = 3;
  const int line_type = 8;
  const int shift = 0;
  const double tip_length = 0.02;

  // Draw an arrowed line
  cv::line(cv_image, line_start, line_end, color, thickness);

  const double tipSize = cv::norm(line_start - line_end) * tip_length; // Factor to normalize the size of the tip depending on the length of the arrow
  const double angle = atan2( (double) line_start.y - line_end.y, (double) line_start.x - line_end.x );

  cv::Point p(round(line_end.x + tipSize * cos(angle + CV_PI / 4)),
      round(line_end.y + tipSize * sin(angle + CV_PI / 4)));
  cv::line(cv_image, p, line_end, color, thickness, line_type, shift);

  p.x = round(line_end.x + tipSize * cos(angle - CV_PI / 4));
  p.y = round(line_end.y + tipSize * sin(angle - CV_PI / 4));
  cv::line(cv_image, p, line_end, color, thickness, line_type, shift);
}

void AddImageOverlay::drawClosestObstacleIndicator(cv::Mat cv_image, int width, int height) {
  double min_distance_local;
  double min_distance_radians_local;
  {
    std::unique_lock<std::mutex> scan_lock(scan_mutex);
    min_distance_local = min_distance;
    min_distance_radians_local = min_distance_radians;
    scan_lock.unlock();
  }

  // Draw the lidar proximity warning
  if (min_distance_local < min_distance_threshold) {
    // TODO: this assumes the head is centered. Instead, where the
    // red appears should be relative to the head orientation!
    double min_rad = -0.785;
    double max_rad = 0.785;
    double relative_location = (std::min(std::max(min_rad, min_distance_radians_local), max_rad) - min_rad)/(max_rad-min_rad);
    int distance_warning_center_x = relative_location*width;
    int distance_warning_center_y = height;
    int distance_center_width = width*1/10;
    int distance_center_height = height*1/10;

    cv::Scalar color(0,0,255);
    int thickness = -1;

    cv::rectangle(
      cv_image,
      cv::Point(
        distance_warning_center_x-distance_center_width/2,
        distance_warning_center_y-distance_center_height/2
      ),
      cv::Point(
        distance_warning_center_x+distance_center_width/2,
        distance_warning_center_y+distance_center_height/2
      ),
      color,
      thickness
    );
  }
}

void AddImageOverlay::imageCallback(boost::shared_ptr<sensor_msgs::CompressedImage> msg) {
  std::unique_lock<std::mutex> image_lock(image_mutex);
  last_image_recv_timestamp = image.header.stamp;
  image = *(msg.get());
  has_new_image = true;
  delay = ros::Time::now() - image.header.stamp;
  image_lock.unlock();
}

void AddImageOverlay::powerCallback(boost::shared_ptr<mobile_base_driver::Power> msg) {
  std::unique_lock<std::mutex> battery_lock(battery_mutex);
  battery_percent = msg->battery.rounded_pct;
  is_charging = msg->dock_present;
  battery_lock.unlock();
}

void AddImageOverlay::scanCallback(boost::shared_ptr<sensor_msgs::LaserScan> msg) {
  double min_distance_local = 10.0; // max is 6.75
  double distance;
  double min_distance_radians_local;
  for (int i = 0; i < msg->ranges.size(); i++) {
    distance = msg->ranges[i];
    if (!std::isnan(distance) && !std::isinf(distance)) {
      if (distance <= min_distance_local) {
        min_distance_local = distance;
        min_distance_radians_local = msg->angle_min + i*msg->angle_increment;
      }
    }
  }
  std::unique_lock<std::mutex> scan_lock(scan_mutex);
  min_distance = min_distance_local;
  min_distance_radians = min_distance_radians_local;
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
  AddImageOverlay add_image_overlay;
  ros::spin();
  return 0;
}
