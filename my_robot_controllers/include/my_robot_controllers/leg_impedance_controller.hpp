#ifndef MY_ROBOT_CONTROLLERS__LEG_IMPEDANCE_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLERS__LEG_IMPEDANCE_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "odrive_mit_example/msg/leg_cmd.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limits_urdf.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"
#include "urdf/model.h"

namespace my_robot_controllers
{

class LegImpedanceController : public controller_interface::ControllerInterface
{
public:
  LegImpedanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  rclcpp::Subscription<odrive_mit_example::msg::LegCmd>::SharedPtr sub_command_;
  odrive_mit_example::msg::LegCmd latest_cmd_;
  bool has_new_cmd_ = false;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration cmd_timeout_{0, 500000000};  // default 0.5 s, configurable via cmd_timeout_s

  // Storage for joint limits
  std::vector<joint_limits::JointLimits> joint_limits_;

  // Storage for actuator offsets
  std::vector<double> joint_offsets_;

  // Storage for actuator directions
  std::vector<double> joint_directions_;

  // Hard joint-velocity cap (rad/s). 0 = disabled (URDF limits only). Set via the
  // 'max_joint_velocity' parameter. When > 0: clamps velocity_des AND slew-rate-limits
  // position_des to +/- max_joint_velocity_ * dt per update (a real velocity cap, unlike
  // velocity_des alone). Baseline re-initialised to the measured position on activation.
  double max_joint_velocity_ = 0.0;
  std::vector<double> prev_pos_cmd_;   // last commanded URDF-frame position per joint (slew baseline)
  bool slew_init_ = false;             // re-init prev_pos_cmd_ on (re)activation

  // ---- Pre-flight / engage interlock --------------------------------------
  // The controller activates LIMP (zero torque) and will not apply gains until a set
  // of safety checks all pass on LIVE data in the update loop. If they don't pass within
  // engage_timeout_s_, update() returns ERROR so the controller_manager deactivates it
  // (it REFUSES to engage rather than ever commanding the motors from an unsafe state).
  bool preflight_enabled_ = true;
  double engage_pos_tol_ = 0.2;         // anti-jump: |cmd - measured| must be < this (rad)
  double engage_vel_thresh_ = 0.2;      // at-rest: |measured vel| must be < this (rad/s)
  int engage_settle_cycles_ = 10;       // live-position: valid cycles in passive closed-loop first
  bool require_upright_ = false;        // enable the IMU torso-upright check
  double upright_max_tilt_deg_ = 25.0;  // max torso tilt from vertical at engage (deg)
  double imu_timeout_s_ = 0.5;          // IMU reading must be fresher than this (s)
  double engage_timeout_s_ = 8.0;       // refuse (ERROR + deactivate) if not engaged within this

  // Runtime fall guard: while ENGAGED, if torso tilt exceeds fall_tilt_deg_ the controller
  // latches LIMP (zero torque) so it never writes jerky corrections to a toppling robot.
  // Latched until the controller is re-activated.
  bool fall_limp_enabled_ = false;
  double fall_tilt_deg_ = 50.0;

  enum class EngageState { VALIDATING, ENGAGED, FAULT_LATCHED, ESTOP };
  EngageState engage_state_ = EngageState::VALIDATING;
  rclcpp::Time engage_start_time_{0, 0, RCL_ROS_TIME};
  int settle_count_ = 0;
  rclcpp::Time last_block_log_{0, 0, RCL_ROS_TIME};

  // IMU subscription for the upright check (sim feed or real BNO08x; topic is a param)
  std::string imu_topic_ = "/imu";
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  std::mutex imu_mutex_;
  double imu_qx_ = 0.0, imu_qy_ = 0.0, imu_qz_ = 0.0, imu_qw_ = 1.0;
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  bool have_imu_ = false;

  // ---- Dashboard status (~/preflight_status, JSON String) -----------------
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;   // operator hard-stop
  rclcpp::Time last_status_pub_{0, 0, RCL_ROS_TIME};
  std::string preflight_reason_ = "init";

  // Helpers
  void write_limp();                                                    // zero kp/kd/vel/eff (passive)
  bool run_preflight(const rclcpp::Time & time, std::string & reason);  // true once all checks pass
  bool current_tilt_deg(const rclcpp::Time & time, double & tilt_deg);  // torso tilt from a fresh IMU
  void publish_status(const rclcpp::Time & time);                       // ~10 Hz JSON for the dashboard
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace my_robot_controllers

#endif  // MY_ROBOT_CONTROLLERS__LEG_IMPEDANCE_CONTROLLER_HPP_
