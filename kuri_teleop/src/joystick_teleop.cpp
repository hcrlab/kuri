#include <kuri_teleop/joystick_teleop.h>

JoystickTeleop::JoystickTeleop() :
head_action_srv("head_controller/follow_joint_trajectory"),
eye_action_srv("eyelids_controller/follow_joint_trajectory") {
  linear_max = 0.4; // meter / sec
  angular_max = M_PI / 3; // rad / sec
  head_tilt_speed = 2.0; // position units / sec (based on controlLoop_hz)
  head_tilt_max = 0.3; // position units
  head_tilt_min = -0.8; // position units
  head_pan_speed = 2.0; // position units per control / sec (based on controlLoop_hz)
  head_pan_max = 0.75; // position units
  head_pan_min = -0.75; // position units
  head_eye_movement_duration = 0.15; // secs
  eye_closed_position = 0.41; // position units
  eye_open_position = 0.0; // position units
  controlLoop_hz = 100.0; // Hz

  base_cmd_pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1); //"cmd_vel" may also work
  joystick_sub = nh.subscribe("joy", 1, &JoystickTeleop::joystickCallback, this);
  nav_vel_sub = nh.subscribe("kuri_navigation_command_velocity", 1, &JoystickTeleop::navVelCallback, this);

  head_controller_state_sub = nh.subscribe("/head_controller/state", 1, &JoystickTeleop::headControllerStateCallback, this);
  eye_controller_state_sub = nh.subscribe("/eyelids_controller/state", 1, &JoystickTeleop::eyeControllerStateCallback, this);

  new_joystick_vel = false;
  new_nav_vel = false;
  new_emergency_stop = false;
  new_head_state_command = false;
  new_current_head_state = false;

  controlLoop_thread = std::thread(&JoystickTeleop::controlLoop, this);
}

JoystickTeleop::~JoystickTeleop() {
  // This should work, because roscpp's default SIGINT handler calls
  // ros::shutdown(), and controlLoop will terminate once ros::isShuttingDown()
  // returns true, so it is okay for this function to wait until controlLoop_thread
  // terminates.
  controlLoop_thread.join();
}


void JoystickTeleop::controlLoop() {
  ros::Rate controlLoop_rate(controlLoop_hz);

  bool was_new_vel_received = false;
  bool was_new_head_state_command_received = false;
  bool was_new_eye_state_command_received = false;

  geometry_msgs::Twist last_joystick_vel;
  last_joystick_vel.linear.x = 0;
  last_joystick_vel.linear.y = 0;
  last_joystick_vel.linear.z = 0;
  last_joystick_vel.angular.x = 0;
  last_joystick_vel.angular.y = 0;
  last_joystick_vel.angular.z = 0;
  geometry_msgs::Twist last_nav_vel;
  last_nav_vel.linear.x = 0;
  last_nav_vel.linear.y = 0;
  last_nav_vel.linear.z = 0;
  last_nav_vel.angular.x = 0;
  last_nav_vel.angular.y = 0;
  last_nav_vel.angular.z = 0;
  geometry_msgs::Twist last_published_vel;
  last_published_vel.linear.x = 0;
  last_published_vel.linear.y = 0;
  last_published_vel.linear.z = 0;
  last_published_vel.angular.x = 0;
  last_published_vel.angular.y = 0;
  last_published_vel.angular.z = 0;

  control_msgs::FollowJointTrajectoryGoal last_published_head_goal;
  last_published_head_goal.trajectory.header.seq = 0;
  last_published_head_goal.trajectory.header.stamp = ros::Time::now();
  last_published_head_goal.trajectory.header.frame_id = "";
  last_published_head_goal.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
  trajectory_msgs::JointTrajectoryPoint point_0;
  last_published_head_goal.trajectory.points = {point_0};
  last_published_head_goal.trajectory.points[0].positions = {0.0, 0.0};
  last_published_head_goal.trajectory.points[0].velocities = {};
  last_published_head_goal.trajectory.points[0].accelerations = {};
  last_published_head_goal.trajectory.points[0].effort = {};
  last_published_head_goal.trajectory.points[0].time_from_start = ros::Duration(head_eye_movement_duration);

  control_msgs::FollowJointTrajectoryGoal last_published_eye_goal;
  last_published_eye_goal.trajectory.header.seq = 0;
  last_published_eye_goal.trajectory.header.stamp = ros::Time::now();
  last_published_eye_goal.trajectory.header.frame_id = "";
  last_published_eye_goal.trajectory.joint_names = {"eyelids_joint"};
  trajectory_msgs::JointTrajectoryPoint point_1;
  last_published_eye_goal.trajectory.points = {point_1};
  last_published_eye_goal.trajectory.points[0].positions = {0.0};
  last_published_eye_goal.trajectory.points[0].velocities = {};
  last_published_eye_goal.trajectory.points[0].accelerations = {};
  last_published_eye_goal.trajectory.points[0].effort = {};
  last_published_eye_goal.trajectory.points[0].time_from_start = ros::Duration(head_eye_movement_duration);

  int last_joystick_head_tilt_value = 0;
  double current_head_tilt_value = 0.0;
  int last_joystick_head_pan_value = 0;
  int last_joystick_head_pan_reset_value = 0;
  double current_head_pan_value = 0.0;
  bool have_received_head_state_value = false;
  double target_eye_pos = eye_open_position;
  bool are_eyes_open = true;
  bool have_received_eye_state_value = false;

  while (!ros::isShuttingDown()) {
    controlLoop_rate.sleep();

    {
      // check if the e-stop was pressed
      std::unique_lock<std::mutex> emergency_stop_lock(emergency_stop_mutex);
      if (new_emergency_stop) {
        new_emergency_stop = false;
        if (emergency_stop) {
          emergency_stop_lock.unlock();
          last_published_vel.linear.x = 0;
          last_published_vel.linear.y = 0;
          last_published_vel.linear.z = 0;
          last_published_vel.angular.x = 0;
          last_published_vel.angular.y = 0;
          last_published_vel.angular.z = 0;
          ROS_INFO("Control loop got e-stop");
        } else {
          ROS_INFO("Control loop got re-enable robot");
        }
        continue;
      } else {
        emergency_stop_lock.unlock();
      }
    }

    was_new_vel_received = false;
    was_new_head_state_command_received = false;
    was_new_eye_state_command_received = false;

    {
      // check if a velocity was published from the joystick
      std::unique_lock<std::mutex> joystick_vel_lock(joystick_vel_mutex);
      if (new_joystick_vel) {
        new_joystick_vel = false;
        last_joystick_vel.linear.x = joystick_vel.linear.x;
        last_joystick_vel.linear.y = 0;
        last_joystick_vel.linear.z = 0;
        last_joystick_vel.angular.x = 0;
        last_joystick_vel.angular.y = 0;
        last_joystick_vel.angular.z = joystick_vel.angular.z;
        joystick_vel_lock.unlock();
        was_new_vel_received = true;
      } else {
        joystick_vel_lock.unlock();
      }
    }

    {
      // check if a velocity was published from the navigation topic
      std::unique_lock<std::mutex> nav_vel_lock(nav_vel_mutex);
      if (new_nav_vel) {
        new_nav_vel = false;
        last_nav_vel.linear.x = nav_vel.linear.x;
        last_nav_vel.linear.y = 0;
        last_nav_vel.linear.z = 0;
        last_nav_vel.angular.x = 0;
        last_nav_vel.angular.y = 0;
        last_nav_vel.angular.z = nav_vel.angular.z;
        nav_vel_lock.unlock();
        was_new_vel_received = true;
      } else {
        nav_vel_lock.unlock();
      }
    }

    {
      // publish the Velocities
      std::unique_lock<std::mutex> emergency_stop_lock(emergency_stop_mutex);
      if (!emergency_stop) {
        emergency_stop_lock.unlock();
        if (was_new_vel_received) {
          last_published_vel.linear.x = last_nav_vel.linear.x + last_joystick_vel.linear.x;
          last_published_vel.linear.y = 0;
          last_published_vel.linear.z = 0;
          last_published_vel.angular.x = 0;
          last_published_vel.angular.y = 0;
          last_published_vel.angular.z = last_nav_vel.angular.z + last_joystick_vel.angular.z;
        }
        base_cmd_pub.publish(last_published_vel);
      } else {
        emergency_stop_lock.unlock();
      }
    }

    {
      // check if we have received the current head state
      std::unique_lock<std::mutex> current_head_state_lock(current_head_state_mutex);
      if (new_current_head_state) {
        new_current_head_state = false;
        current_head_tilt_value = current_head_tilt;
        current_head_pan_value = current_head_pan;
        have_received_head_state_value = true;
        current_head_state_lock.unlock();
      } else {
        current_head_state_lock.unlock();
      }
    }

    {
      // check if a head state command was published from the joystick
      if (have_received_head_state_value) {
        std::unique_lock<std::mutex> head_state_command_lock(head_state_command_mutex);
        if (new_head_state_command) {
          new_head_state_command = false;
          last_joystick_head_tilt_value = head_tilt_command;
          last_joystick_head_pan_reset_value = head_pan_reset;
          last_joystick_head_pan_value = head_pan_command;
          head_state_command_lock.unlock();
          was_new_head_state_command_received = true;
        } else {
          head_state_command_lock.unlock();
        }
      }
    }

    // publish the head goal
    last_published_head_goal.trajectory.header.stamp = ros::Time::now();
    last_published_head_goal.trajectory.points[0].positions[0] = last_joystick_head_pan_reset_value == 1 ? ((head_pan_max + head_pan_min) / 2.0) : std::min(std::max(current_head_pan_value + -1.0 * ((double)last_joystick_head_pan_value) * head_pan_speed * head_eye_movement_duration, head_pan_min), head_pan_max);
    last_published_head_goal.trajectory.points[0].positions[1] = std::min(std::max(current_head_tilt_value + -1.0 * ((double)last_joystick_head_tilt_value) * head_tilt_speed * head_eye_movement_duration, head_tilt_min), head_tilt_max);
    if (last_joystick_head_pan_value != 0 || last_joystick_head_tilt_value != 0 || last_joystick_head_pan_reset_value == 1) {
      head_action_srv.sendGoal(last_published_head_goal);
    }

    {
      // check if we have received the current head state
      std::unique_lock<std::mutex> current_eye_state_lock(current_eye_state_mutex);
      if (new_current_eye_state) {
        new_current_eye_state = false;
        are_eyes_open = ((current_eye_pos - eye_open_position) / (eye_closed_position - eye_open_position)) < 0.5;
        have_received_eye_state_value = true;
        current_eye_state_lock.unlock();
      } else {
        current_eye_state_lock.unlock();
      }
    }

    {
      // check if a head state command was published from the joystick
      if (have_received_eye_state_value) {
        std::unique_lock<std::mutex> eye_state_command_lock(eye_state_command_mutex);
        if (new_eye_state_command) {
          new_eye_state_command = false;
          eye_state_command_lock.unlock();
          target_eye_pos = are_eyes_open ? eye_closed_position : eye_open_position;
          was_new_eye_state_command_received = true;
        } else {
          eye_state_command_lock.unlock();
        }
      }
    }

    last_published_eye_goal.trajectory.header.stamp = ros::Time::now();
    last_published_eye_goal.trajectory.points[0].positions[0] = target_eye_pos;
    if (was_new_eye_state_command_received) eye_action_srv.sendGoal(last_published_eye_goal);
  }
}

void JoystickTeleop::joystickCallback(const sensor_msgs::Joy &msg) {
  /*
  Logitech F710 (XInput mode, controlled by the toggle on the controller top):
  NOTE: If the Mode light is green, the joystick swaps Left Joystick and
  Arrow Keys

  Axes: [
  Left Joystick L-R (left positive),
  Left Joystick U-D (up positive),
  LT (1 released, -1 pressed),
  Right Joystick L-R (left positive),
  Right Joystick U-D (up positive),
  LR (1 released, -1 pressed),
  Arrow Keys L-R (left positive),
  Arrow Keys U-D (up positive),
  ]

  Buttons: [
  A, B, X, Y, LB, RB, Back, Start, Center,
  Left Joystick Push, Right Joystick Push,
  ]

  For this mapping:
  - Left Joystick L-R will be angular velocity
  - Right Joystick U-D will be linear velocity
  - LB and LR will be e-stop buttons
  - Start will be a re-enable button
  - Arrow Keys U-D head tilt
  - Arrow Keys L-R head pan
  - Button A will toggle the eyes open/close
  - Button X centers the head (pan)

  */

  if (msg.buttons[4] || msg.buttons[5]) { // if any of the estop buttons are pressed
    std::unique_lock<std::mutex> emergency_stop_lock(emergency_stop_mutex);
    if (!emergency_stop) new_emergency_stop = true;
    emergency_stop = true;
    emergency_stop_lock.unlock();
    ROS_INFO("Joystick callback got e-stop");
    return;
  } else if (msg.buttons[7]) { // re-enable robot
    std::unique_lock<std::mutex> emergency_stop_lock(emergency_stop_mutex);
    if (emergency_stop) new_emergency_stop = true;
    emergency_stop = false;
    emergency_stop_lock.unlock();
    ROS_INFO("Joystick callback got re-enable");
    return;
  }

  // NOTE: this was an attempt to use just one 2DOF joystick for both linear
  // and rotation movements, but turned out to be unintuitive.

  /* double left_right = msg.axes[0];
  double up_down = msg.axes[1];
  double linear_vel = 0.0, angular_vel = 0.0;

  if (left_right != 0.0 || up_down != 0.0) {
    double magnitude = std::sqrt(std::pow(left_right, 2.0) + std::pow(up_down, 2.0));

    if (std::abs(up_down) >= std::abs(left_right)) {
      linear_vel = std::copysign(magnitude, up_down);
      angular_vel = std::copysign(magnitude, left_right)*std::abs(left_right)/std::abs(up_down);
    } else {
      linear_vel = std::copysign(magnitude, up_down)*std::abs(up_down)/std::abs(left_right);
      angular_vel = std::copysign(magnitude, left_right);
    }
  } */

  double linear_vel = msg.axes[4];
  double angular_vel = msg.axes[0];

  geometry_msgs::Twist vel;
  vel.linear.x = linear_max * linear_vel;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = angular_max * angular_vel;
  {
    std::unique_lock<std::mutex> joystick_vel_lock(joystick_vel_mutex);
    joystick_vel = vel;
    new_joystick_vel = true;
    joystick_vel_lock.unlock();
  }

  int head_tilt = msg.axes[7];
  int head_pan = msg.axes[6];
  {
    std::unique_lock<std::mutex> head_state_command_lock(head_state_command_mutex);
    head_pan_reset = msg.buttons[2];
    head_tilt_command = head_tilt;
    head_pan_command = head_pan;
    new_head_state_command = true;
    head_state_command_lock.unlock();
  }

  int eye_toggle = msg.buttons[0];
  {
    std::unique_lock<std::mutex> eye_state_command_lock(eye_state_command_mutex);
    new_eye_state_command = (eye_toggle == 1.0);
    eye_state_command_lock.unlock();
  }
}

void JoystickTeleop::navVelCallback(const geometry_msgs::Twist &msg) {
  geometry_msgs::Twist vel;
  vel.linear.x = msg.linear.x;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = msg.angular.z;
  {
    std::unique_lock<std::mutex> nav_vel_lock(nav_vel_mutex);
    nav_vel = vel;
    new_nav_vel = true;
    nav_vel_lock.unlock();
  }
}

void JoystickTeleop::headControllerStateCallback(const control_msgs::JointTrajectoryControllerState & msg) {
  double current_pan = msg.desired.positions[0];
  double current_tilt = msg.desired.positions[1];
  {
    std::unique_lock<std::mutex> current_head_state_lock(current_head_state_mutex);
    current_head_tilt = current_tilt;
    current_head_pan = current_pan;
    new_current_head_state = true;
    current_head_state_lock.unlock();
  }
}

void JoystickTeleop::eyeControllerStateCallback(const control_msgs::JointTrajectoryControllerState & msg) {
  double current_pos = msg.actual.positions[0];
  {
    std::unique_lock<std::mutex> current_eye_state_lock(current_eye_state_mutex);
    current_eye_pos = current_pos;
    new_current_eye_state = true;
    current_eye_state_lock.unlock();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kuri_joystick_teleop");
  JoystickTeleop JoystickTeleopNode;
  ros::spin();
  return 0;
}
