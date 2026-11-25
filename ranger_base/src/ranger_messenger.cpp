/**
* @file ranger_messenger.cpp
* @date 2021-04-20
* @brief
*
# @copyright Copyright (c) 2021 AgileX Robotics
* @copyright Copyright (c) 2023 Weston Robot Pte. Ltd.
*/

#include "ranger_base/ranger_messenger.hpp"

#include "ranger_base/kinematics_model.hpp"

using namespace rclcpp;
using namespace ranger_msgs::msg;

namespace westonrobot {
// namespace {
// double DegreeToRadian(double x) { return x * M_PI / 180.0; }
// }  // namespace

///////////////////////////////////////////////////////////////////////////////////
RangerROSMessenger::RangerROSMessenger(rclcpp::Node::SharedPtr& node){

  node_ = node;
  LoadParameters();

  // connect to robot and setup ROS subscription
  if (robot_type_ == RangerSubType::kRangerMiniV1) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV1);
  } else if (robot_type_ == RangerSubType::kRangerMiniV2) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV2);
  } else if (robot_type_ == RangerSubType::kRangerMiniV3) {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRangerMiniV3);
  } else {
    robot_ = std::make_shared<RangerRobot>(RangerRobot::Variant::kRanger);
  }

  if (port_name_.find("can") != std::string::npos) {
    if (!robot_->Connect(port_name_)) {
      RCLCPP_ERROR(node_->get_logger(),"Failed to connect to the CAN port");
      return;
    }
    robot_->EnableCommandedMode();
  } else {
    RCLCPP_ERROR(node_->get_logger(),"Invalid port name: %s", port_name_.c_str());
    return;
  }

  SetupSubscription();
}

void RangerROSMessenger::Run() {
  rclcpp::Rate rate(update_rate_);
  while (rclcpp::ok()) {
    PublishStateToROS();
    rclcpp::spin_some(node_);
    rate.sleep();
  }
}

void RangerROSMessenger::LoadParameters() {
  //load parameter from launch files
  port_name_ = node_->declare_parameter<std::string>("port_name","can0");
  robot_model_ = node_->declare_parameter<std::string>("robot_model","ranger");
  odom_frame_ =  node_->declare_parameter<std::string>("odom_frame","odom");
  base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
  update_rate_ = node_->declare_parameter<int>("update_rate", 50);
  odom_topic_name_ = node_->declare_parameter<std::string>("odom_topic_name", "odom");
  publish_odom_tf_ = node_->declare_parameter<bool>("publish_odom_tf",false);
  // Proposal, off by default (today's behaviour): a zero command keeps the base's reported
  // oblique or spin mode so the wheels do not swing back at every stop.
  zero_command_keeps_mode_ =
      node_->declare_parameter<bool>("zero_command_keeps_mode", false);

  RCLCPP_INFO(node_->get_logger(),
      "Successfully loaded the following parameters: \n port_name: %s\n "
      "robot_model: %s\n odom_frame: %s\n base_frame: %s\n "
      "update_rate: %d\n odom_topic_name: %s\n "
      "publish_odom_tf: %d\n zero_command_keeps_mode: %d\n",
      port_name_.c_str(), robot_model_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), update_rate_, odom_topic_name_.c_str(),
      publish_odom_tf_, zero_command_keeps_mode_);

  // load robot parameters
  if (robot_model_ == "ranger_mini_v1") {
    robot_type_ = RangerSubType::kRangerMiniV1;

    robot_params_.track = RangerMiniV1Params::track;
    robot_params_.wheelbase = RangerMiniV1Params::wheelbase;
    robot_params_.max_linear_speed = RangerMiniV1Params::max_linear_speed;
    robot_params_.max_angular_speed = RangerMiniV1Params::max_angular_speed;
    robot_params_.max_speed_cmd = RangerMiniV1Params::max_speed_cmd;
    robot_params_.max_steer_angle_central =
        RangerMiniV1Params::max_steer_angle_central;
    robot_params_.max_steer_angle_parallel =
        RangerMiniV1Params::max_steer_angle_parallel;
    robot_params_.max_round_angle = RangerMiniV1Params::max_round_angle;
    robot_params_.min_turn_radius = RangerMiniV1Params::min_turn_radius;
    robot_params_.max_steer_angle_ackermann =
        RangerMiniV1Params::max_steer_angle_ackermann;
  } else {
    if (robot_model_ == "ranger_mini_v2") {
      robot_type_ = RangerSubType::kRangerMiniV2;

      robot_params_.track = RangerMiniV2Params::track;
      robot_params_.wheelbase = RangerMiniV2Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV2Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV2Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV2Params::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerMiniV2Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV2Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV2Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV2Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV2Params::max_steer_angle_ackermann;
    } else if (robot_model_ == "ranger_mini_v3") {
      robot_type_ = RangerSubType::kRangerMiniV3;

      robot_params_.track = RangerMiniV3Params::track;
      robot_params_.wheelbase = RangerMiniV3Params::wheelbase;
      robot_params_.max_linear_speed = RangerMiniV3Params::max_linear_speed;
      robot_params_.max_angular_speed = RangerMiniV3Params::max_angular_speed;
      robot_params_.max_speed_cmd = RangerMiniV3Params::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerMiniV3Params::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerMiniV3Params::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerMiniV3Params::max_round_angle;
      robot_params_.min_turn_radius = RangerMiniV3Params::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerMiniV3Params::max_steer_angle_ackermann;
    } else {
      robot_type_ = RangerSubType::kRanger;

      robot_params_.track = RangerParams::track;
      robot_params_.wheelbase = RangerParams::wheelbase;
      robot_params_.max_linear_speed = RangerParams::max_linear_speed;
      robot_params_.max_angular_speed = RangerParams::max_angular_speed;
      robot_params_.max_speed_cmd = RangerParams::max_speed_cmd;
      robot_params_.max_steer_angle_central =
          RangerParams::max_steer_angle_central;
      robot_params_.max_steer_angle_parallel =
          RangerParams::max_steer_angle_parallel;
      robot_params_.max_round_angle = RangerParams::max_round_angle;
      robot_params_.min_turn_radius = RangerParams::min_turn_radius;
      robot_params_.max_steer_angle_ackermann =
          RangerParams::max_steer_angle_ackermann;
    }
  }
  parking_mode_ = false;

  limits_ = motion_logic::Limits{
      robot_params_.wheelbase, robot_params_.track, robot_params_.max_linear_speed,
      robot_params_.max_angular_speed, robot_params_.max_steer_angle_parallel,
      robot_params_.max_steer_angle_ackermann, robot_params_.min_turn_radius,
      robot_params_.max_round_angle, robot_type_ == RangerSubType::kRangerMiniV1};
  odometry_ = std::make_unique<motion_logic::OdometryIntegrator>(limits_);
}

void RangerROSMessenger::SetupSubscription() {
  // publisher
  system_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::SystemState>(topics::kSystemState, 10);
  motion_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::MotionState>(topics::kMotionState, 10);
  motion_feedback_pub_ = node_->create_publisher<ranger_msgs::msg::MotionFeedback>(
      topics::kMotionFeedback, 10);
  actuator_state_pub_ =
      node_->create_publisher<ranger_msgs::msg::ActuatorStateArray>(topics::kActuatorState, 10);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 10);
  battery_state_pub_ =
      node_->create_publisher<sensor_msgs::msg::BatteryState>(topics::kBatteryState, 10);

  // subscriber
  motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      topics::kCmdVel, 5,
      std::bind(&RangerROSMessenger::TwistCmdCallback, this, std::placeholders::_1));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void RangerROSMessenger::PublishStateToROS() {
  current_time_ = node_->get_clock()->now();

  static bool init_run = true;
  if (init_run) {
    last_time_ = current_time_;
    init_run = false;
    return;
  }

  auto state = robot_->GetRobotState();
  auto actuator_state = robot_->GetActuatorState();

  // update odometry from what the base reports (never from the last command)
  motion_logic::FeedbackSample sample;
  sample.reported_mode = state.motion_mode_state.motion_mode;
  sample.mode_changing = state.motion_mode_state.mode_changing != 0;
  sample.linear_velocity = state.motion_state.linear_velocity;
  sample.angular_velocity = state.motion_state.angular_velocity;
  sample.steering_angle = state.motion_state.steering_angle;
  sample.wheel_angle = actuator_state.motor_angles.angle_5;
  sample.wheel_speed = actuator_state.motor_speeds.speed_1;
  motion_logic::OdometryStep odometry_step{};
  {
    double dt = (current_time_ - last_time_).seconds();
    odometry_step = odometry_->Step(sample, dt);
    PublishOdometry(odometry_step);
    last_time_ = current_time_;
  }

  // publish system state
  {
    ranger_msgs::msg::SystemState system_msg;
    system_msg.header.stamp = current_time_;
    system_msg.vehicle_state = state.system_state.vehicle_state;
    system_msg.control_mode = state.system_state.control_mode;
    system_msg.error_code = state.system_state.error_code;
    system_msg.battery_voltage = state.system_state.battery_voltage;
    system_msg.motion_mode = state.motion_mode_state.motion_mode;

    system_state_pub_->publish(system_msg);
  }

  // publish motion mode
  {
    ranger_msgs::msg::MotionState motion_msg;
    motion_msg.header.stamp = current_time_;
    motion_msg.motion_mode = state.motion_mode_state.motion_mode;

    motion_state_pub_->publish(motion_msg);
  }

  // publish what the base reports about its motion (switching flag, reported and measured state)
  {
    ranger_msgs::msg::MotionFeedback feedback_msg;
    feedback_msg.header.stamp = current_time_;
    feedback_msg.motion_mode = sample.reported_mode;
    feedback_msg.mode_changing = sample.mode_changing;
    feedback_msg.steering_angle = sample.steering_angle;
    feedback_msg.linear_velocity = sample.linear_velocity;
    feedback_msg.angular_velocity = sample.angular_velocity;
    feedback_msg.wheel_angles = {
        actuator_state.motor_angles.angle_5, actuator_state.motor_angles.angle_6,
        actuator_state.motor_angles.angle_7, actuator_state.motor_angles.angle_8};
    feedback_msg.wheel_speeds = {
        actuator_state.motor_speeds.speed_1, actuator_state.motor_speeds.speed_2,
        actuator_state.motor_speeds.speed_3, actuator_state.motor_speeds.speed_4};
    feedback_msg.odometry_geometry = static_cast<uint8_t>(odometry_step.geometry);

    motion_feedback_pub_->publish(feedback_msg);
  }

  // publish actuator state
  {
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "Angle_5:%f Angle_6:%f Angle_7:%f Angle_8:%f",
    //                 actuator_state.motor_angles.angle_5,
    //                 actuator_state.motor_angles.angle_6,
    //                 actuator_state.motor_angles.angle_7,
    //                 actuator_state.motor_angles.angle_8);
    // RCLCPP_DEBUG(node_->get_logger(),"feedback", "speed_1:%f speed_2:%f speed_3:%f speed_4:%f",
    //                 actuator_state.motor_speeds.speed_1,
    //                 actuator_state.motor_speeds.speed_2,
    //                 actuator_state.motor_speeds.speed_3,
    //                 actuator_state.motor_speeds.speed_4);

    ranger_msgs::msg::ActuatorStateArray actuator_msg;
    actuator_msg.header.stamp = current_time_;
    for (int i = 0; i < 8; i++) {
      ranger_msgs::msg::DriverState driver_state_msg;
      driver_state_msg.driver_voltage =
          actuator_state.actuator_ls_state->driver_voltage;
      driver_state_msg.driver_temperature =
          actuator_state.actuator_ls_state->driver_temp;
      driver_state_msg.motor_temperature =
          actuator_state.actuator_ls_state->motor_temp;
      driver_state_msg.driver_state =
          actuator_state.actuator_ls_state->driver_state;

      ranger_msgs::msg::MotorState motor_state_msg;
      motor_state_msg.current = actuator_state.actuator_hs_state->current;
      motor_state_msg.pulse_count = actuator_state.actuator_hs_state->pulse_count;
      motor_state_msg.rpm = actuator_state.actuator_hs_state->rpm;
      motor_state_msg.motor_angles = actuator_state.motor_angles.angle_5;
      motor_state_msg.motor_speeds = actuator_state.motor_speeds.speed_1;

      ranger_msgs::msg::ActuatorState actuator_state_msg;
      actuator_state_msg.id = i;
      actuator_state_msg.driver = driver_state_msg;
      actuator_state_msg.motor = motor_state_msg;

      actuator_msg.states.push_back(actuator_state_msg);
    }

    actuator_state_pub_->publish(actuator_msg);
  }

  // publish BMS state
  {
    auto common_sensor_state = robot_->GetCommonSensorState();

    sensor_msgs::msg::BatteryState batt_msg;
    batt_msg.header.stamp = current_time_;
    // The SDK's Ranger Mini 3.0 class passes the BMS voltage on in units of 0.1 V (its Mini 2.0
    // class applies the 0.1 itself); /system_state.battery_voltage comes from another frame and
    // is already in volts.
    const double bms_voltage_scale = robot_type_ == RangerSubType::kRangerMiniV3 ? 0.1 : 1.0;
    batt_msg.voltage = common_sensor_state.bms_basic_state.voltage * bms_voltage_scale;
    batt_msg.temperature = common_sensor_state.bms_basic_state.temperature;
    batt_msg.current = common_sensor_state.bms_basic_state.current;
    batt_msg.percentage = common_sensor_state.bms_basic_state.battery_soc;
    batt_msg.charge = std::numeric_limits<float>::quiet_NaN();
    batt_msg.capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    batt_msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batt_msg.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batt_msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.present = std::numeric_limits<uint8_t>::quiet_NaN();

    battery_state_pub_->publish(batt_msg);
  }
}

void RangerROSMessenger::PublishOdometry(const motion_logic::OdometryStep& step) {
  // update odometry topics
  geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(step.theta);

  // publish odometry and tf messages
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time_;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = step.x;
  odom_msg.pose.pose.position.y = step.y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = step.vx;
  odom_msg.twist.twist.linear.y = step.vy;
  odom_msg.twist.twist.angular.z = step.wz;

  odom_pub_->publish(odom_msg);

  // // publish tf transformation
  if (publish_odom_tf_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = step.x;
    tf_msg.transform.translation.y = step.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);
  }
}

namespace {
// Sends the commands of the pure logic through the SDK.
class RobotCommandSink : public motion_logic::CommandSink {
 public:
  explicit RobotCommandSink(RangerRobot& robot) : robot_(robot) {}
  void SetMotionMode(uint8_t mode) override { robot_.SetMotionMode(mode); }
  void SetMotionCommand(double linear, double steering, double angular) override {
    robot_.SetMotionCommand(linear, steering, angular);
  }

 private:
  RangerRobot& robot_;
};
}  // namespace

void RangerROSMessenger::TwistCmdCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
  // check for parking mode, only applicable to RangerMiniV2
  if (parking_mode_ && robot_type_ == RangerSubType::kRangerMiniV2) {
    return;
  }

  // The command depends on this twist only. With zero_command_keeps_mode, a zero twist also
  // reads the base's reported mode and wheel angle (its state, not an earlier command).
  motion_logic::ZeroCommandPolicy zero_policy{false, 0, 0.0};
  if (zero_command_keeps_mode_) {
    zero_policy.keep_reported_mode = true;
    zero_policy.reported_mode = robot_->GetRobotState().motion_mode_state.motion_mode;
    zero_policy.measured_wheel_angle = robot_->GetActuatorState().motor_angles.angle_5;
  }
  const motion_logic::MotionCommand command = motion_logic::CommandFromTwist(
      msg->linear.x, msg->linear.y, msg->angular.z, limits_, zero_policy);

  if (!command.valid) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Ignoring a velocity command that is not finite");
    return;
  }
  if (command.dropped != motion_logic::kDropNone) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Velocity command (%.3f, %.3f, %.3f): motion mode %u cannot realise%s%s%s; "
        "commanded as zero",
        msg->linear.x, msg->linear.y, msg->angular.z, static_cast<unsigned>(command.mode),
        (command.dropped & motion_logic::kDropLinearX) ? " linear.x" : "",
        (command.dropped & motion_logic::kDropLinearY) ? " linear.y" : "",
        (command.dropped & motion_logic::kDropAngularZ) ? " angular.z" : "");
  }

  RobotCommandSink sink(*robot_);
  motion_logic::DispatchCommand(command, sink);
}

geometry_msgs::msg::Quaternion RangerROSMessenger::createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

}  // namespace westonrobot
