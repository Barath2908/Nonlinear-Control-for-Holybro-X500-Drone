/**
 * @file sliding_mode_controller.cpp
 * @brief Implementation of Sliding Mode Controller for quadrotor position control
 * @author Barath Kumar JK
 * @date 2025
 */

#include "vio_mavlink_bridge/sliding_mode_controller.hpp"

#include <cmath>
#include <algorithm>

namespace vio_mavlink_bridge
{

SlidingModeController::SlidingModeController(const rclcpp::NodeOptions & options)
: Node("sliding_mode_controller", options)
{
  RCLCPP_INFO(get_logger(), "Initializing Sliding Mode Controller");
  
  declare_parameters();
  load_parameters();
  initialize_subscribers();
  initialize_publishers();
  initialize_timers();
  
  // Initialize timestamps
  last_odom_time_ = get_clock()->now();
  last_setpoint_time_ = get_clock()->now();
  
  RCLCPP_INFO(get_logger(), "Sliding Mode Controller initialized");
}

void SlidingModeController::declare_parameters()
{
  // Sliding surface parameters
  declare_parameter("lambda_x", 2.0);
  declare_parameter("lambda_y", 2.0);
  declare_parameter("lambda_z", 1.5);
  
  // Control gains
  declare_parameter("k_x", 5.0);
  declare_parameter("k_y", 5.0);
  declare_parameter("k_z", 3.0);
  
  // Boundary layer
  declare_parameter("eta_x", 0.1);
  declare_parameter("eta_y", 0.1);
  declare_parameter("eta_z", 0.1);
  
  // Model parameters
  declare_parameter("mass", 1.5);
  declare_parameter("gravity", 9.81);
  
  // Limits
  declare_parameter("max_thrust", 25.0);
  declare_parameter("min_thrust", 0.0);
  declare_parameter("max_tilt_angle", 0.5);
  declare_parameter("max_velocity_xy", 2.0);
  declare_parameter("max_velocity_z", 1.0);
  declare_parameter("max_acceleration", 5.0);
  
  // Integral gains
  declare_parameter("ki_x", 0.0);
  declare_parameter("ki_y", 0.0);
  declare_parameter("ki_z", 0.0);
  declare_parameter("integral_limit", 2.0);
  
  // Control settings
  declare_parameter("control_rate_hz", 100.0);
  declare_parameter("use_attitude_control", true);
  declare_parameter("enabled", false);
  
  // Topics
  declare_parameter("odom_topic", "/vio/odom_ned");
  declare_parameter("setpoint_topic", "/setpoint/pose");
}

void SlidingModeController::load_parameters()
{
  params_.lambda_x = get_parameter("lambda_x").as_double();
  params_.lambda_y = get_parameter("lambda_y").as_double();
  params_.lambda_z = get_parameter("lambda_z").as_double();
  
  params_.k_x = get_parameter("k_x").as_double();
  params_.k_y = get_parameter("k_y").as_double();
  params_.k_z = get_parameter("k_z").as_double();
  
  params_.eta_x = get_parameter("eta_x").as_double();
  params_.eta_y = get_parameter("eta_y").as_double();
  params_.eta_z = get_parameter("eta_z").as_double();
  
  params_.mass = get_parameter("mass").as_double();
  params_.gravity = get_parameter("gravity").as_double();
  
  params_.max_thrust = get_parameter("max_thrust").as_double();
  params_.min_thrust = get_parameter("min_thrust").as_double();
  params_.max_tilt_angle = get_parameter("max_tilt_angle").as_double();
  params_.max_velocity_xy = get_parameter("max_velocity_xy").as_double();
  params_.max_velocity_z = get_parameter("max_velocity_z").as_double();
  params_.max_acceleration = get_parameter("max_acceleration").as_double();
  
  params_.ki_x = get_parameter("ki_x").as_double();
  params_.ki_y = get_parameter("ki_y").as_double();
  params_.ki_z = get_parameter("ki_z").as_double();
  params_.integral_limit = get_parameter("integral_limit").as_double();
  
  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  use_attitude_control_ = get_parameter("use_attitude_control").as_bool();
  enabled_ = get_parameter("enabled").as_bool();
  
  RCLCPP_INFO(get_logger(), "SMC Parameters:");
  RCLCPP_INFO(get_logger(), "  Lambda: [%.2f, %.2f, %.2f]", 
    params_.lambda_x, params_.lambda_y, params_.lambda_z);
  RCLCPP_INFO(get_logger(), "  K: [%.2f, %.2f, %.2f]", 
    params_.k_x, params_.k_y, params_.k_z);
  RCLCPP_INFO(get_logger(), "  Eta: [%.2f, %.2f, %.2f]", 
    params_.eta_x, params_.eta_y, params_.eta_z);
  RCLCPP_INFO(get_logger(), "  Mass: %.2f kg", params_.mass);
  RCLCPP_INFO(get_logger(), "  Control rate: %.1f Hz", control_rate_hz_);
}

void SlidingModeController::initialize_subscribers()
{
  std::string odom_topic = get_parameter("odom_topic").as_string();
  std::string setpoint_topic = get_parameter("setpoint_topic").as_string();
  
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&SlidingModeController::odom_callback, this, std::placeholders::_1));
  
  setpoint_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    setpoint_topic,
    rclcpp::QoS(10),
    std::bind(&SlidingModeController::setpoint_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(get_logger(), "Subscribed to: %s, %s", odom_topic.c_str(), setpoint_topic.c_str());
}

void SlidingModeController::initialize_publishers()
{
  attitude_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>(
    "/mavros/setpoint_raw/attitude",
    rclcpp::QoS(10));
  
  position_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
    "/mavros/setpoint_raw/local",
    rclcpp::QoS(10));
  
  debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/smc/debug",
    rclcpp::QoS(10));
  
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/smc/cmd_vel",
    rclcpp::QoS(10));
}

void SlidingModeController::initialize_timers()
{
  auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SlidingModeController::control_loop_callback, this));
}

void SlidingModeController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  state_.position = Eigen::Vector3d(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z);
  
  state_.velocity = Eigen::Vector3d(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z);
  
  state_.orientation = Eigen::Quaterniond(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  
  last_odom_time_ = msg->header.stamp;
  state_.initialized = true;
}

void SlidingModeController::setpoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  state_.position_des = Eigen::Vector3d(
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z);
  
  // Extract yaw from quaternion
  Eigen::Quaterniond q(
    msg->pose.orientation.w,
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z);
  
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  state_.yaw_des = euler(2);
  
  last_setpoint_time_ = msg->header.stamp;
  
  RCLCPP_DEBUG(get_logger(), "New setpoint: [%.2f, %.2f, %.2f]",
    state_.position_des.x(), state_.position_des.y(), state_.position_des.z());
}

void SlidingModeController::control_loop_callback()
{
  if (!enabled_) return;
  
  auto now = get_clock()->now();
  
  // Check for timeouts
  double odom_age = (now - last_odom_time_).seconds();
  double setpoint_age = (now - last_setpoint_time_).seconds();
  
  if (odom_age > ODOM_TIMEOUT_SEC) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Odometry timeout (%.2f s)", odom_age);
    return;
  }
  
  if (setpoint_age > SETPOINT_TIMEOUT_SEC) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
      "Setpoint timeout (%.2f s), holding position", setpoint_age);
    // Hold current position
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.position_des = state_.position;
  }
  
  // Compute control
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (!state_.initialized) return;
  
  double dt = 1.0 / control_rate_hz_;
  Eigen::Vector3d accel = compute_sliding_mode_control(dt);
  
  // Convert to attitude commands
  acceleration_to_attitude(
    accel,
    state_.yaw_des,
    state_.thrust,
    state_.roll_des,
    state_.pitch_des);
  
  // Apply limits
  apply_limits(state_.thrust, state_.roll_des, state_.pitch_des);
  
  // Publish
  if (use_attitude_control_) {
    publish_attitude_target();
  } else {
    publish_position_target();
  }
  
  publish_debug_info();
}

Eigen::Vector3d SlidingModeController::compute_sliding_mode_control(double dt)
{
  // Compute errors
  state_.position_error = state_.position_des - state_.position;
  state_.velocity_error = state_.velocity_des - state_.velocity;
  
  // Compute sliding surface: s = e_dot + λ*e
  state_.sliding_surface = compute_sliding_surface();
  
  // Update integral term (if using PI sliding mode)
  update_integral(dt);
  
  // Compute control: u = λ*e_dot + k*sat(s/η) + ki*∫e
  Eigen::Vector3d u;
  
  // Equivalent control (to keep on sliding surface)
  Eigen::Vector3d ueq;
  ueq.x() = params_.lambda_x * state_.velocity_error.x() + state_.acceleration_des.x();
  ueq.y() = params_.lambda_y * state_.velocity_error.y() + state_.acceleration_des.y();
  ueq.z() = params_.lambda_z * state_.velocity_error.z() + state_.acceleration_des.z();
  
  // Switching control (with saturation for chattering reduction)
  Eigen::Vector3d usw;
  usw.x() = params_.k_x * saturation(state_.sliding_surface.x(), params_.eta_x);
  usw.y() = params_.k_y * saturation(state_.sliding_surface.y(), params_.eta_y);
  usw.z() = params_.k_z * saturation(state_.sliding_surface.z(), params_.eta_z);
  
  // Integral term
  Eigen::Vector3d uint;
  uint.x() = params_.ki_x * state_.integral_error.x();
  uint.y() = params_.ki_y * state_.integral_error.y();
  uint.z() = params_.ki_z * state_.integral_error.z();
  
  // Total control
  u = ueq + usw + uint;
  
  // Add gravity compensation for z-axis
  u.z() += params_.gravity;
  
  // Apply acceleration limits
  for (int i = 0; i < 3; i++) {
    u(i) = std::clamp(u(i), -params_.max_acceleration, params_.max_acceleration);
  }
  
  state_.control_acceleration = u;
  
  return u;
}

Eigen::Vector3d SlidingModeController::compute_sliding_surface()
{
  Eigen::Vector3d s;
  
  // s = e_dot + λ*e
  s.x() = state_.velocity_error.x() + params_.lambda_x * state_.position_error.x();
  s.y() = state_.velocity_error.y() + params_.lambda_y * state_.position_error.y();
  s.z() = state_.velocity_error.z() + params_.lambda_z * state_.position_error.z();
  
  return s;
}

double SlidingModeController::saturation(double s, double eta) const
{
  if (eta <= 0) return sign_with_deadband(s, 0.001);
  
  if (std::abs(s) <= eta) {
    return s / eta;
  } else {
    return (s > 0) ? 1.0 : -1.0;
  }
}

double SlidingModeController::sign_with_deadband(double x, double deadband) const
{
  if (std::abs(x) < deadband) return 0.0;
  return (x > 0) ? 1.0 : -1.0;
}

void SlidingModeController::acceleration_to_attitude(
  const Eigen::Vector3d & accel_des,
  double yaw_des,
  double & thrust,
  double & roll,
  double & pitch)
{
  // Total thrust magnitude
  double thrust_mag = params_.mass * accel_des.norm();
  thrust = thrust_mag;
  
  // Avoid division by zero
  if (thrust_mag < 0.1) {
    roll = 0.0;
    pitch = 0.0;
    return;
  }
  
  // Desired thrust direction (unit vector)
  Eigen::Vector3d thrust_dir = accel_des.normalized();
  
  // Compute roll and pitch from desired thrust direction
  // For NED frame: positive pitch = nose down, positive roll = right wing down
  
  // Simplified attitude computation (small angle approximation)
  // More accurate: use proper rotation matrix decomposition
  double cy = std::cos(yaw_des);
  double sy = std::sin(yaw_des);
  
  // Roll: rotation about body x-axis
  // Pitch: rotation about body y-axis
  // Given desired acceleration in NED and yaw, compute roll/pitch
  
  // ax = g * (cy*sin(pitch)*cos(roll) + sy*sin(roll))
  // ay = g * (sy*sin(pitch)*cos(roll) - cy*sin(roll))
  // az = g * cos(pitch)*cos(roll)
  
  double az = accel_des.z();
  double ax = accel_des.x();
  double ay = accel_des.y();
  
  // Using small angle: sin(x) ≈ x, cos(x) ≈ 1
  // For larger angles, use full computation
  pitch = std::atan2(ax * cy + ay * sy, az);
  roll = std::atan2(
    (ay * cy - ax * sy) * std::cos(pitch),
    az);
  
  // Clamp to reasonable values to avoid singularities
  pitch = std::clamp(pitch, -M_PI_4, M_PI_4);
  roll = std::clamp(roll, -M_PI_4, M_PI_4);
}

void SlidingModeController::apply_limits(double & thrust, double & roll, double & pitch)
{
  thrust = std::clamp(thrust, params_.min_thrust, params_.max_thrust);
  roll = std::clamp(roll, -params_.max_tilt_angle, params_.max_tilt_angle);
  pitch = std::clamp(pitch, -params_.max_tilt_angle, params_.max_tilt_angle);
}

void SlidingModeController::update_integral(double dt)
{
  // Anti-windup: only integrate when close to sliding surface
  const double integration_zone = 5.0;  // Only integrate within 5x eta
  
  if (std::abs(state_.sliding_surface.x()) < integration_zone * params_.eta_x) {
    state_.integral_error.x() += state_.position_error.x() * dt;
  }
  if (std::abs(state_.sliding_surface.y()) < integration_zone * params_.eta_y) {
    state_.integral_error.y() += state_.position_error.y() * dt;
  }
  if (std::abs(state_.sliding_surface.z()) < integration_zone * params_.eta_z) {
    state_.integral_error.z() += state_.position_error.z() * dt;
  }
  
  // Clamp integral
  for (int i = 0; i < 3; i++) {
    state_.integral_error(i) = std::clamp(
      state_.integral_error(i),
      -params_.integral_limit,
      params_.integral_limit);
  }
}

void SlidingModeController::reset_controller()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_.integral_error.setZero();
  state_.position_des = state_.position;
  state_.velocity_des.setZero();
  state_.acceleration_des.setZero();
}

void SlidingModeController::publish_attitude_target()
{
  mavros_msgs::msg::AttitudeTarget msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "base_link";
  
  // Type mask: ignore body rates
  msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                  mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                  mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
  
  // Convert roll, pitch, yaw to quaternion
  Eigen::Quaterniond q = Eigen::AngleAxisd(state_.yaw_des, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(state_.pitch_des, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(state_.roll_des, Eigen::Vector3d::UnitX());
  
  msg.orientation.w = q.w();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  
  // Thrust (normalized 0-1 for PX4, raw for ArduPilot may differ)
  msg.thrust = static_cast<float>(state_.thrust / params_.max_thrust);
  
  attitude_pub_->publish(msg);
}

void SlidingModeController::publish_position_target()
{
  mavros_msgs::msg::PositionTarget msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "odom";
  msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  
  // Set acceleration setpoints
  msg.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                  mavros_msgs::msg::PositionTarget::IGNORE_PY |
                  mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                  mavros_msgs::msg::PositionTarget::IGNORE_VX |
                  mavros_msgs::msg::PositionTarget::IGNORE_VY |
                  mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                  mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
  
  msg.acceleration_or_force.x = state_.control_acceleration.x();
  msg.acceleration_or_force.y = state_.control_acceleration.y();
  msg.acceleration_or_force.z = state_.control_acceleration.z();
  msg.yaw = static_cast<float>(state_.yaw_des);
  
  position_pub_->publish(msg);
}

void SlidingModeController::publish_debug_info()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(18);
  
  // Position error
  msg.data[0] = state_.position_error.x();
  msg.data[1] = state_.position_error.y();
  msg.data[2] = state_.position_error.z();
  
  // Velocity error
  msg.data[3] = state_.velocity_error.x();
  msg.data[4] = state_.velocity_error.y();
  msg.data[5] = state_.velocity_error.z();
  
  // Sliding surface
  msg.data[6] = state_.sliding_surface.x();
  msg.data[7] = state_.sliding_surface.y();
  msg.data[8] = state_.sliding_surface.z();
  
  // Control output
  msg.data[9] = state_.control_acceleration.x();
  msg.data[10] = state_.control_acceleration.y();
  msg.data[11] = state_.control_acceleration.z();
  
  // Attitude commands
  msg.data[12] = state_.thrust;
  msg.data[13] = state_.roll_des;
  msg.data[14] = state_.pitch_des;
  msg.data[15] = state_.yaw_des;
  
  // Integral error
  msg.data[16] = state_.integral_error.norm();
  msg.data[17] = state_.position_error.norm();
  
  debug_pub_->publish(msg);
}

}  // namespace vio_mavlink_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vio_mavlink_bridge::SlidingModeController)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vio_mavlink_bridge::SlidingModeController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
