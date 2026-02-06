/**
 * @file mavlink_bridge_node.cpp
 * @brief Implementation of MAVLink External Vision Bridge for ArduPilot EKF3
 * @author Barath Kumar JK
 * @date 2025
 */

#include "vio_mavlink_bridge/mavlink_bridge_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cmath>
#include <chrono>

namespace vio_mavlink_bridge
{

MavlinkBridgeNode::MavlinkBridgeNode(const rclcpp::NodeOptions & options)
: Node("mavlink_bridge_node", options)
{
  RCLCPP_INFO(get_logger(), "Initializing VIO-MAVLink Bridge Node");
  
  declare_parameters();
  load_parameters();
  initialize_serial();
  initialize_subscribers();
  initialize_publishers();
  initialize_timers();
  
  // Initialize TF2
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Start serial read thread
  serial_read_thread_ = std::thread(&MavlinkBridgeNode::serial_read_thread, this);
  
  RCLCPP_INFO(get_logger(), "VIO-MAVLink Bridge Node initialized successfully");
}

MavlinkBridgeNode::~MavlinkBridgeNode()
{
  running_ = false;
  
  if (serial_read_thread_.joinable()) {
    serial_read_thread_.join();
  }
  
  if (serial_fd_ >= 0) {
    close(serial_fd_);
  }
  
  RCLCPP_INFO(get_logger(), "VIO-MAVLink Bridge Node shutdown complete");
}

void MavlinkBridgeNode::declare_parameters()
{
  // Serial parameters
  declare_parameter("serial_port", "/dev/ttyACM0");
  declare_parameter("baudrate", 921600);
  
  // MAVLink parameters
  declare_parameter("system_id", 1);
  declare_parameter("component_id", 197);
  declare_parameter("target_system", 1);
  declare_parameter("target_component", 1);
  
  // Topic parameters
  declare_parameter("vio_topic", "/camera/odom/sample");
  declare_parameter("odom_ned_topic", "/vio/odom_ned");
  
  // Frame parameters
  declare_parameter("vio_frame", "camera_odom_frame");
  declare_parameter("body_frame", "base_link");
  declare_parameter("local_frame", "odom");
  declare_parameter("use_ned", true);
  
  // Timing parameters
  declare_parameter("publish_rate_hz", 30.0);
  declare_parameter("vision_delay_ms", 50);
  declare_parameter("coordinate_frame", 21);  // MAV_FRAME_LOCAL_FRD
  
  // Heartbeat parameters
  declare_parameter("heartbeat_rate_hz", 1.0);
  declare_parameter("status_rate_hz", 2.0);
}

void MavlinkBridgeNode::load_parameters()
{
  serial_port_ = get_parameter("serial_port").as_string();
  baudrate_ = get_parameter("baudrate").as_int();
  
  system_id_ = static_cast<uint8_t>(get_parameter("system_id").as_int());
  component_id_ = static_cast<uint8_t>(get_parameter("component_id").as_int());
  target_system_ = static_cast<uint8_t>(get_parameter("target_system").as_int());
  target_component_ = static_cast<uint8_t>(get_parameter("target_component").as_int());
  
  frame_config_.vio_frame = get_parameter("vio_frame").as_string();
  frame_config_.body_frame = get_parameter("body_frame").as_string();
  frame_config_.local_frame = get_parameter("local_frame").as_string();
  frame_config_.use_ned = get_parameter("use_ned").as_bool();
  
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  vision_delay_ms_ = get_parameter("vision_delay_ms").as_int();
  coordinate_frame_ = static_cast<uint8_t>(get_parameter("coordinate_frame").as_int());
  
  RCLCPP_INFO(get_logger(), "Loaded parameters:");
  RCLCPP_INFO(get_logger(), "  Serial: %s @ %d baud", serial_port_.c_str(), baudrate_);
  RCLCPP_INFO(get_logger(), "  MAVLink: sys=%d, comp=%d", system_id_, component_id_);
  RCLCPP_INFO(get_logger(), "  Publish rate: %.1f Hz", publish_rate_hz_);
  RCLCPP_INFO(get_logger(), "  Vision delay: %d ms", vision_delay_ms_);
}

void MavlinkBridgeNode::initialize_serial()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
    RCLCPP_WARN(get_logger(), "Running in simulation mode (no MAVLink output)");
    return;
  }
  
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to get serial port attributes");
    close(serial_fd_);
    serial_fd_ = -1;
    return;
  }
  
  // Set baud rate
  speed_t baud;
  switch (baudrate_) {
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 500000: baud = B500000; break;
    case 576000: baud = B576000; break;
    case 921600: baud = B921600; break;
    case 1000000: baud = B1000000; break;
    default:
      RCLCPP_WARN(get_logger(), "Unknown baud rate %d, using 921600", baudrate_);
      baud = B921600;
  }
  
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);
  
  // 8N1
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  
  // No hardware flow control
  tty.c_cflag &= ~CRTSCTS;
  
  // Enable receiver, ignore modem control lines
  tty.c_cflag |= CREAD | CLOCAL;
  
  // Raw input
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  
  // Raw output
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;
  
  // Non-blocking read
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to set serial port attributes");
    close(serial_fd_);
    serial_fd_ = -1;
    return;
  }
  
  // Flush buffers
  tcflush(serial_fd_, TCIOFLUSH);
  
  serial_connected_ = true;
  RCLCPP_INFO(get_logger(), "Serial port initialized: %s @ %d baud",
    serial_port_.c_str(), baudrate_);
}

void MavlinkBridgeNode::initialize_subscribers()
{
  std::string vio_topic = get_parameter("vio_topic").as_string();
  
  vio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    vio_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&MavlinkBridgeNode::vio_odom_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(get_logger(), "Subscribed to VIO topic: %s", vio_topic.c_str());
}

void MavlinkBridgeNode::initialize_publishers()
{
  std::string odom_ned_topic = get_parameter("odom_ned_topic").as_string();
  
  odom_ned_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    odom_ned_topic,
    rclcpp::SensorDataQoS());
  
  vio_health_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/vio/health",
    rclcpp::QoS(1).reliable());
  
  RCLCPP_INFO(get_logger(), "Publishing NED odometry to: %s", odom_ned_topic.c_str());
}

void MavlinkBridgeNode::initialize_timers()
{
  double heartbeat_rate = get_parameter("heartbeat_rate_hz").as_double();
  double status_rate = get_parameter("status_rate_hz").as_double();
  
  auto heartbeat_period = std::chrono::duration<double>(1.0 / heartbeat_rate);
  heartbeat_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
    std::bind(&MavlinkBridgeNode::heartbeat_timer_callback, this));
  
  auto status_period = std::chrono::duration<double>(1.0 / status_rate);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
    std::bind(&MavlinkBridgeNode::status_timer_callback, this));
  
  // Health check at 5 Hz
  health_check_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&MavlinkBridgeNode::health_check_callback, this));
}

void MavlinkBridgeNode::vio_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Update health tracking
  auto now = get_clock()->now();
  last_vio_msg_time_ = now;
  
  vio_msg_times_.push_back(now);
  while (vio_msg_times_.size() > VIO_RATE_WINDOW_SIZE) {
    vio_msg_times_.pop_front();
  }
  
  // Transform to NED if needed
  nav_msgs::msg::Odometry odom_ned = *msg;
  
  if (frame_config_.use_ned) {
    transform_to_ned(msg->pose.pose, odom_ned.pose.pose);
    transform_velocity_to_ned(msg->twist.twist, odom_ned.twist.twist);
    odom_ned.header.frame_id = "odom_ned";
    odom_ned.child_frame_id = "base_link_ned";
  }
  
  // Publish NED odometry for debugging
  odom_ned_pub_->publish(odom_ned);
  
  // Broadcast TF
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = odom_ned.header;
  tf_msg.child_frame_id = odom_ned.child_frame_id;
  tf_msg.transform.translation.x = odom_ned.pose.pose.position.x;
  tf_msg.transform.translation.y = odom_ned.pose.pose.position.y;
  tf_msg.transform.translation.z = odom_ned.pose.pose.position.z;
  tf_msg.transform.rotation = odom_ned.pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf_msg);
  
  // Send to ArduPilot via MAVLink
  if (serial_connected_) {
    bool success = send_vision_position_estimate(odom_ned);
    success &= send_vision_speed_estimate(odom_ned);
    
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (success) {
      status_.messages_sent++;
    } else {
      status_.messages_failed++;
    }
  }
}

bool MavlinkBridgeNode::send_vision_position_estimate(const nav_msgs::msg::Odometry & odom)
{
  mavlink_message_t msg;
  mavlink_vision_position_estimate_t vpe;
  
  // Timestamp in microseconds
  vpe.usec = get_time_usec();
  
  // Position (NED frame for ArduPilot)
  vpe.x = static_cast<float>(odom.pose.pose.position.x);
  vpe.y = static_cast<float>(odom.pose.pose.position.y);
  vpe.z = static_cast<float>(odom.pose.pose.position.z);
  
  // Orientation as Euler angles (NED convention)
  double roll, pitch, yaw;
  quaternion_to_euler_ned(odom.pose.pose.orientation, roll, pitch, yaw);
  vpe.roll = static_cast<float>(roll);
  vpe.pitch = static_cast<float>(pitch);
  vpe.yaw = static_cast<float>(yaw);
  
  // Covariance (upper-right triangular, row-major)
  // [cov_x, cov_xy, cov_xz, cov_xroll, cov_xpitch, cov_xyaw,
  //        cov_y, cov_yz, cov_yroll, cov_ypitch, cov_yyaw,
  //               cov_z, cov_zroll, cov_zpitch, cov_zyaw,
  //                      cov_roll, cov_rollpitch, cov_rollyaw,
  //                                cov_pitch, cov_pitchyaw,
  //                                           cov_yaw]
  std::array<float, 21> cov;
  extract_covariance(odom, cov);
  std::copy(cov.begin(), cov.end(), vpe.covariance);
  
  // Reset counter (set to 0 when reset occurs, increment otherwise)
  vpe.reset_counter = 0;
  
  mavlink_msg_vision_position_estimate_encode(
    system_id_, component_id_, &msg, &vpe);
  
  return write_mavlink_message(msg);
}

bool MavlinkBridgeNode::send_vision_speed_estimate(const nav_msgs::msg::Odometry & odom)
{
  mavlink_message_t msg;
  mavlink_vision_speed_estimate_t vse;
  
  vse.usec = get_time_usec();
  vse.x = static_cast<float>(odom.twist.twist.linear.x);
  vse.y = static_cast<float>(odom.twist.twist.linear.y);
  vse.z = static_cast<float>(odom.twist.twist.linear.z);
  
  // Velocity covariance (diagonal elements)
  vse.covariance[0] = static_cast<float>(odom.twist.covariance[0]);   // vx
  vse.covariance[1] = 0.0f;
  vse.covariance[2] = 0.0f;
  vse.covariance[3] = static_cast<float>(odom.twist.covariance[7]);   // vy
  vse.covariance[4] = 0.0f;
  vse.covariance[5] = static_cast<float>(odom.twist.covariance[14]);  // vz
  
  vse.reset_counter = 0;
  
  mavlink_msg_vision_speed_estimate_encode(
    system_id_, component_id_, &msg, &vse);
  
  return write_mavlink_message(msg);
}

void MavlinkBridgeNode::send_heartbeat()
{
  if (!serial_connected_) return;
  
  mavlink_message_t msg;
  mavlink_heartbeat_t hb;
  
  hb.type = MAV_TYPE_ONBOARD_CONTROLLER;
  hb.autopilot = MAV_AUTOPILOT_INVALID;
  hb.base_mode = 0;
  hb.custom_mode = 0;
  hb.system_status = MAV_STATE_ACTIVE;
  
  mavlink_msg_heartbeat_encode(system_id_, component_id_, &msg, &hb);
  write_mavlink_message(msg);
}

bool MavlinkBridgeNode::write_mavlink_message(const mavlink_message_t & msg)
{
  if (serial_fd_ < 0) return false;
  
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  
  std::lock_guard<std::mutex> lock(serial_mutex_);
  ssize_t written = write(serial_fd_, buffer, len);
  
  if (written != static_cast<ssize_t>(len)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Failed to write MAVLink message: %zd/%d bytes", written, len);
    return false;
  }
  
  return true;
}

void MavlinkBridgeNode::serial_read_thread()
{
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t byte;
  
  while (running_) {
    if (serial_fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    
    ssize_t n = read(serial_fd_, &byte, 1);
    if (n > 0) {
      if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
        // Process incoming messages (e.g., ACKs, heartbeats)
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_HEARTBEAT:
            {
              std::lock_guard<std::mutex> lock(status_mutex_);
              status_.is_connected = true;
            }
            break;
          case MAVLINK_MSG_ID_STATUSTEXT:
            {
              mavlink_statustext_t st;
              mavlink_msg_statustext_decode(&msg, &st);
              RCLCPP_INFO(get_logger(), "ArduPilot: %s", st.text);
            }
            break;
          default:
            break;
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void MavlinkBridgeNode::heartbeat_timer_callback()
{
  send_heartbeat();
}

void MavlinkBridgeNode::status_timer_callback()
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "Bridge Status - Connected: %s, VIO Healthy: %s, Rate: %.1f Hz, Sent: %lu, Failed: %lu",
    status_.is_connected ? "yes" : "no",
    status_.vio_healthy ? "yes" : "no",
    status_.vio_rate_hz,
    status_.messages_sent,
    status_.messages_failed);
}

void MavlinkBridgeNode::health_check_callback()
{
  auto now = get_clock()->now();
  double time_since_last = (now - last_vio_msg_time_).seconds();
  
  std::lock_guard<std::mutex> lock(status_mutex_);
  
  // Check VIO timeout
  status_.vio_healthy = (time_since_last < VIO_TIMEOUT_SEC);
  
  // Calculate VIO rate
  if (vio_msg_times_.size() >= 2) {
    double window_duration = (vio_msg_times_.back() - vio_msg_times_.front()).seconds();
    if (window_duration > 0) {
      status_.vio_rate_hz = static_cast<double>(vio_msg_times_.size() - 1) / window_duration;
    }
  }
  
  // Publish health status
  std_msgs::msg::Bool health_msg;
  health_msg.data = status_.vio_healthy;
  vio_health_pub_->publish(health_msg);
  
  if (!status_.vio_healthy) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "VIO timeout! Last message %.2f seconds ago", time_since_last);
  }
}

void MavlinkBridgeNode::transform_to_ned(
  const geometry_msgs::msg::Pose & pose_in,
  geometry_msgs::msg::Pose & pose_out)
{
  // Transform from camera frame (typically FLU or similar) to NED
  // Common VIO output: X-forward, Y-left, Z-up (FLU)
  // ArduPilot NED: X-north, Y-east, Z-down
  
  // Assuming VIO is in ENU (East-North-Up) convention:
  // NED_x = ENU_y (North)
  // NED_y = ENU_x (East)
  // NED_z = -ENU_z (Down)
  
  pose_out.position.x = pose_in.position.y;   // ENU_y -> NED_x
  pose_out.position.y = pose_in.position.x;   // ENU_x -> NED_y
  pose_out.position.z = -pose_in.position.z;  // -ENU_z -> NED_z
  
  // Transform quaternion from ENU to NED
  // This requires a 180-degree rotation about X, then 90-degree about Z
  tf2::Quaternion q_enu(
    pose_in.orientation.x,
    pose_in.orientation.y,
    pose_in.orientation.z,
    pose_in.orientation.w);
  
  // Rotation from ENU to NED frame
  tf2::Quaternion q_enu_to_ned;
  q_enu_to_ned.setRPY(M_PI, 0, M_PI_2);
  
  tf2::Quaternion q_ned = q_enu_to_ned * q_enu;
  q_ned.normalize();
  
  pose_out.orientation.x = q_ned.x();
  pose_out.orientation.y = q_ned.y();
  pose_out.orientation.z = q_ned.z();
  pose_out.orientation.w = q_ned.w();
}

void MavlinkBridgeNode::transform_velocity_to_ned(
  const geometry_msgs::msg::Twist & twist_in,
  geometry_msgs::msg::Twist & twist_out)
{
  // Transform velocity from ENU to NED
  twist_out.linear.x = twist_in.linear.y;   // ENU_vy -> NED_vx
  twist_out.linear.y = twist_in.linear.x;   // ENU_vx -> NED_vy
  twist_out.linear.z = -twist_in.linear.z;  // -ENU_vz -> NED_vz
  
  twist_out.angular.x = twist_in.angular.y;
  twist_out.angular.y = twist_in.angular.x;
  twist_out.angular.z = -twist_in.angular.z;
}

void MavlinkBridgeNode::quaternion_to_euler_ned(
  const geometry_msgs::msg::Quaternion & q,
  double & roll, double & pitch, double & yaw)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);
}

uint64_t MavlinkBridgeNode::get_time_usec() const
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()
    ).count()
  );
}

void MavlinkBridgeNode::extract_covariance(
  const nav_msgs::msg::Odometry & odom,
  std::array<float, 21> & cov)
{
  // Extract from 6x6 pose covariance to upper-right triangular
  // Order: x, y, z, roll, pitch, yaw
  const auto & pc = odom.pose.covariance;
  
  int idx = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = i; j < 6; j++) {
      cov[idx++] = static_cast<float>(pc[i * 6 + j]);
    }
  }
}

}  // namespace vio_mavlink_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vio_mavlink_bridge::MavlinkBridgeNode)

// Main entry point
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<vio_mavlink_bridge::MavlinkBridgeNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
