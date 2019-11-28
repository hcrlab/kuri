
#include "ros/ros.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <mutex>
#include <thread>
#include <string>

#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/client/action_client.h>

class KuriJoystickTeleop
{
  // Define the publishers and subscribers
  ros::NodeHandle nh;                                                                 // use the standard node handle
  ros::Publisher base_cmd_pub;                                                        // publish velocity commands to the base
  actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> head_action_srv; // action server for the head
  actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> eye_action_srv;  // action server for the eyelids
  ros::Subscriber joystick_sub;                                                       // subscribe to the joystick
  ros::Subscriber nav_vel_sub;                                                        // subscribe to the navigation velocity
  ros::Subscriber head_controller_state_sub;                                          // subscribe to the head controller state
  ros::Subscriber eye_controller_state_sub;                                           // subscribe to the eye controller state

  // Define motion parameters
  double linear_max, angular_max;                       // Max/min velocities for the base
  double head_tilt_speed, head_tilt_max, head_tilt_min; // Speed and max/min positions for the head tilt
  double head_pan_speed, head_pan_max, head_pan_min;    // Speed and max/min positions for the head pan
  double head_eye_movement_duration;                    // How long to specify head/eye movements should last in the actionlib goal
  double eye_closed_position, eye_open_position;        // Eye open and closed positions
  double controlLoop_hz;                                // The desired speed of the control loop

  // Concurrency literals
  geometry_msgs::Twist joystick_vel;   // the joystick vel value
	std::mutex joystick_vel_mutex;       // used to synchronize access to joystick_vel
  bool new_joystick_vel;               // whether we have an unread joystick_vel value or not

  geometry_msgs::Twist nav_vel;        // the joystick vel value
	std::mutex nav_vel_mutex;            // used to synchronize access to joystick_vel
  bool new_nav_vel;                    // whether we have an unread joystick_vel value or not

  bool emergency_stop;                 // whether we have e-stopped the robot or not
	std::mutex emergency_stop_mutex;     // used to synchronize access to emergency_stop
  bool new_emergency_stop;             // whether we have an unread emergency_stop value or not

  int head_tilt_command;               // -1, 0, 1
  int head_pan_command;                // -1, 0, 1
	std::mutex head_state_command_mutex; // used to synchronize access to head_tilt_command and head_pan_command
  bool new_head_state_command;         // whether we have an unread head_tilt_command and head_pan_command value or not

  double current_head_tilt;            // current head tilt position
  double current_head_pan;             // current head pan position
	std::mutex current_head_state_mutex; // used to synchronize access to currentHead values
  bool new_current_head_state;         // whether we have an unread currentHead values or not

	std::mutex eye_state_command_mutex;  // used to synchronize access to new_eye_state_command
  bool new_eye_state_command;          // whether we have a new eyeStateCommand from the joystick

  double current_eye_pos;              // current eye position
	std::mutex current_eye_state_mutex;  // used to synchronize access to current_eye_pos
  bool new_current_eye_state;          // whether we have an unread current_eye_pos or not

  std::thread controlLoop_thread;     // main control loop thread

public:
  KuriJoystickTeleop();                                                                        // constructor
  ~KuriJoystickTeleop();                                                                       // destructor
  void controlLoop();                                                                           // main control loop
  void joystickCallback(const sensor_msgs::Joy &msg);                                           // callback for the joystick subscription
  void navVelCallback(const geometry_msgs::Twist &msg);                                        // callback for the navigation velocity subscription
  void headControllerStateCallback(const control_msgs::JointTrajectoryControllerState & msg); // callback for the head controller state subscription
  void eyeControllerStateCallback(const control_msgs::JointTrajectoryControllerState & msg);  // callback for the eye controller state subscription
};
