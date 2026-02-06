/**
 * @file sliding_mode_controller.hpp
 * @brief Sliding Mode Controller for quadrotor position control
 * @author Barath Kumar JK
 * @date 2025
 * 
 * Implements a robust sliding mode controller for GPS-denied indoor flight
 * with chattering reduction via boundary layer approach.
 */

#ifndef VIO_MAVLINK_BRIDGE__SLIDING_MODE_CONTROLLER_HPP_
#define VIO_MAVLINK_BRIDGE__SLIDING_MODE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <memory>
#include <mutex>

namespace vio_mavlink_bridge
{

/**
 * @brief Sliding mode controller parameters
 */
struct SMCParams
{
  // Sliding surface parameters (λ)
  double lambda_x{2.0};
  double lambda_y{2.0};
  double lambda_z{1.5};
  
  // Control gains (k)
  double k_x{5.0};
  double k_y{5.0};
  double k_z{3.0};
  
  // Boundary layer thickness (η) for chattering reduction
  double eta_x{0.1};
  double eta_y{0.1};
  double eta_z{0.1};
  
  // Model parameters
  double mass{1.5};  // kg
  double gravity{9.81};  // m/s^2
  
  // Limits
  double max_thrust{25.0};  // N
  double min_thrust{0.0};   // N
  double max_tilt_angle{0.5};  // rad (~28 deg)
  double max_velocity_xy{2.0};  // m/s
  double max_velocity_z{1.0};   // m/s
  double max_acceleration{5.0}; // m/s^2
  
  // Integral gains (optional PI sliding mode)
  double ki_x{0.0};
  double ki_y{0.0};
  double ki_z{0.0};
  double integral_limit{2.0};
};

/**
 * @brief Controller state for tracking and debugging
 */
struct ControllerState
{
  // Current state
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  
  // Desired state
  Eigen::Vector3d position_des{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_des{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_des{Eigen::Vector3d::Zero()};
  double yaw_des{0.0};
  
  // Errors
  Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
  
  // Sliding surfaces
  Eigen::Vector3d sliding_surface{Eigen::Vector3d::Zero()};
  
  // Control outputs
  Eigen::Vector3d control_acceleration{Eigen::Vector3d::Zero()};
  double thrust{0.0};
  double roll_des{0.0};
  double pitch_des{0.0};
  
  // Integral terms
  Eigen::Vector3d integral_error{Eigen::Vector3d::Zero()};
  
  // Timestamps
  rclcpp::Time last_update_time;
  bool initialized{false};
};

/**
 * @class SlidingModeController
 * @brief ROS 2 node implementing sliding mode position control
 */
class SlidingModeController : public rclcpp::Node
{
public:
  /**
   * @brief Construct the sliding mode controller node
   * @param options Node options for ROS 2
   */
  explicit SlidingModeController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~SlidingModeController() override = default;

private:
  // ============== Initialization ==============
  void declare_parameters();
  void load_parameters();
  void initialize_subscribers();
  void initialize_publishers();
  void initialize_timers();

  // ============== Callbacks ==============
  /**
   * @brief Callback for current state (odometry)
   * @param msg Odometry message from VIO
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Callback for setpoint pose
   * @param msg Desired pose
   */
  void setpoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Control loop timer callback
   */
  void control_loop_callback();

  // ============== Core Control ==============
  /**
   * @brief Compute sliding mode control output
   * @param dt Time step
   * @return Control acceleration vector
   */
  Eigen::Vector3d compute_sliding_mode_control(double dt);

  /**
   * @brief Compute sliding surface values
   * @return Sliding surface vector s = e_dot + λ*e
   */
  Eigen::Vector3d compute_sliding_surface();

  /**
   * @brief Saturation function for chattering reduction
   * @param s Sliding surface value
   * @param eta Boundary layer thickness
   * @return Saturated value in [-1, 1]
   */
  double saturation(double s, double eta) const;

  /**
   * @brief Sign function with deadband
   * @param x Input value
   * @param deadband Deadband threshold
   * @return Sign of x, or 0 if within deadband
   */
  double sign_with_deadband(double x, double deadband) const;

  /**
   * @brief Convert desired acceleration to attitude target
   * @param accel_des Desired acceleration in world frame
   * @param yaw_des Desired yaw angle
   * @param thrust Output thrust value
   * @param roll Output roll angle
   * @param pitch Output pitch angle
   */
  void acceleration_to_attitude(
    const Eigen::Vector3d & accel_des,
    double yaw_des,
    double & thrust,
    double & roll,
    double & pitch);

  /**
   * @brief Apply control limits
   * @param thrust Thrust to limit
   * @param roll Roll to limit
   * @param pitch Pitch to limit
   */
  void apply_limits(double & thrust, double & roll, double & pitch);

  /**
   * @brief Update integral error terms
   * @param dt Time step
   */
  void update_integral(double dt);

  /**
   * @brief Reset controller state
   */
  void reset_controller();

  // ============== Publishing ==============
  /**
   * @brief Publish attitude target command
   */
  void publish_attitude_target();

  /**
   * @brief Publish position target command
   */
  void publish_position_target();

  /**
   * @brief Publish debug information
   */
  void publish_debug_info();

  // ============== Member Variables ==============
  
  // Parameters
  SMCParams params_;
  double control_rate_hz_{100.0};
  bool use_attitude_control_{true};  // vs position control
  bool enabled_{false};
  
  // Controller state
  ControllerState state_;
  std::mutex state_mutex_;
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_sub_;
  
  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // Time tracking
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_setpoint_time_;
  static constexpr double ODOM_TIMEOUT_SEC = 0.5;
  static constexpr double SETPOINT_TIMEOUT_SEC = 2.0;
};

}  // namespace vio_mavlink_bridge

#endif  // VIO_MAVLINK_BRIDGE__SLIDING_MODE_CONTROLLER_HPP_
