/**
 * @file mavlink_bridge_node.hpp
 * @brief MAVLink External Vision Bridge for ArduPilot EKF3
 * @author Barath Kumar JK
 * @date 2025
 * 
 * This node subscribes to VIO odometry and publishes VISION_POSITION_ESTIMATE
 * messages to ArduPilot via MAVLink for GPS-denied navigation.
 */

#ifndef VIO_MAVLINK_BRIDGE__MAVLINK_BRIDGE_NODE_HPP_
#define VIO_MAVLINK_BRIDGE__MAVLINK_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mavlink/common/mavlink.h>

#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>

namespace vio_mavlink_bridge
{

/**
 * @brief Status of the VIO-MAVLink bridge
 */
struct BridgeStatus
{
  bool is_connected{false};
  bool vio_healthy{false};
  bool ekf_converged{false};
  double vio_rate_hz{0.0};
  double latency_ms{0.0};
  uint64_t messages_sent{0};
  uint64_t messages_failed{0};
};

/**
 * @brief Configuration for coordinate frame transformation
 */
struct FrameConfig
{
  std::string vio_frame{"camera_odom_frame"};
  std::string body_frame{"base_link"};
  std::string local_frame{"odom"};
  bool use_ned{true};  // ArduPilot uses NED
};

/**
 * @class MavlinkBridgeNode
 * @brief ROS 2 node that bridges VIO data to ArduPilot via MAVLink
 */
class MavlinkBridgeNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the MAVLink bridge node
   * @param options Node options for ROS 2
   */
  explicit MavlinkBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor - ensures clean shutdown
   */
  ~MavlinkBridgeNode() override;

private:
  // ============== Parameter Declaration ==============
  void declare_parameters();
  void load_parameters();

  // ============== Initialization ==============
  void initialize_serial();
  void initialize_subscribers();
  void initialize_publishers();
  void initialize_timers();

  // ============== Callbacks ==============
  /**
   * @brief Callback for VIO odometry messages
   * @param msg Odometry message from VIO system
   */
  void vio_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Timer callback to send MAVLink heartbeat
   */
  void heartbeat_timer_callback();

  /**
   * @brief Timer callback to publish bridge status
   */
  void status_timer_callback();

  /**
   * @brief Timer callback to check VIO health
   */
  void health_check_callback();

  // ============== MAVLink Communication ==============
  /**
   * @brief Send VISION_POSITION_ESTIMATE to ArduPilot
   * @param odom Odometry message to send
   * @return true if message sent successfully
   */
  bool send_vision_position_estimate(const nav_msgs::msg::Odometry & odom);

  /**
   * @brief Send VISION_SPEED_ESTIMATE to ArduPilot
   * @param odom Odometry message containing velocity
   * @return true if message sent successfully
   */
  bool send_vision_speed_estimate(const nav_msgs::msg::Odometry & odom);

  /**
   * @brief Send MAVLink heartbeat
   */
  void send_heartbeat();

  /**
   * @brief Write MAVLink message to serial port
   * @param msg MAVLink message to send
   * @return true if write successful
   */
  bool write_mavlink_message(const mavlink_message_t & msg);

  /**
   * @brief Read and process incoming MAVLink messages
   */
  void read_mavlink_messages();

  /**
   * @brief Serial port read thread function
   */
  void serial_read_thread();

  // ============== Coordinate Transformations ==============
  /**
   * @brief Transform pose from VIO frame to NED frame
   * @param pose_in Input pose in VIO frame
   * @param pose_out Output pose in NED frame
   */
  void transform_to_ned(
    const geometry_msgs::msg::Pose & pose_in,
    geometry_msgs::msg::Pose & pose_out);

  /**
   * @brief Transform velocity from VIO frame to NED frame
   * @param twist_in Input twist in VIO frame
   * @param twist_out Output twist in NED frame
   */
  void transform_velocity_to_ned(
    const geometry_msgs::msg::Twist & twist_in,
    geometry_msgs::msg::Twist & twist_out);

  /**
   * @brief Extract Euler angles from quaternion (NED convention)
   * @param q Quaternion
   * @param roll Output roll angle (rad)
   * @param pitch Output pitch angle (rad)
   * @param yaw Output yaw angle (rad)
   */
  void quaternion_to_euler_ned(
    const geometry_msgs::msg::Quaternion & q,
    double & roll, double & pitch, double & yaw);

  // ============== Utility Functions ==============
  /**
   * @brief Get current time in microseconds (for MAVLink timestamp)
   * @return Time in microseconds since epoch
   */
  uint64_t get_time_usec() const;

  /**
   * @brief Calculate covariance values for MAVLink message
   * @param odom Odometry message with covariance
   * @param cov Output covariance array (upper-right triangular)
   */
  void extract_covariance(
    const nav_msgs::msg::Odometry & odom,
    std::array<float, 21> & cov);

  /**
   * @brief Update VIO health metrics
   */
  void update_vio_health();

  // ============== Member Variables ==============
  
  // Serial port
  int serial_fd_{-1};
  std::string serial_port_;
  int baudrate_;
  std::atomic<bool> serial_connected_{false};

  // MAVLink
  uint8_t system_id_{1};
  uint8_t component_id_{197};  // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
  uint8_t target_system_{1};
  uint8_t target_component_{1};

  // Threading
  std::thread serial_read_thread_;
  std::atomic<bool> running_{true};
  std::mutex serial_mutex_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_odom_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ned_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vio_health_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr health_check_timer_;

  // TF2
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Configuration
  FrameConfig frame_config_;
  double publish_rate_hz_;
  int vision_delay_ms_;
  uint8_t coordinate_frame_;  // MAV_FRAME

  // Status tracking
  BridgeStatus status_;
  std::mutex status_mutex_;
  
  // VIO health tracking
  rclcpp::Time last_vio_msg_time_;
  std::deque<rclcpp::Time> vio_msg_times_;
  static constexpr size_t VIO_RATE_WINDOW_SIZE = 30;
  static constexpr double VIO_TIMEOUT_SEC = 0.5;

  // Performance metrics
  std::chrono::steady_clock::time_point last_send_time_;
  double avg_latency_ms_{0.0};
};

}  // namespace vio_mavlink_bridge

#endif  // VIO_MAVLINK_BRIDGE__MAVLINK_BRIDGE_NODE_HPP_
