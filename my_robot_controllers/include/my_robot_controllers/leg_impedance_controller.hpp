#ifndef MY_ROBOT_CONTROLLERS__LEG_IMPEDANCE_CONTROLLER_HPP_
#define MY_ROBOT_CONTROLLERS__LEG_IMPEDANCE_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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

  // To store handles for easier access
  // Structure: [joint_index][interface_index]
  // Interfaces order: position, velocity, effort, kp, kd
  // Or maybe just a flat list mapped by name?
  // We can't store handles directly easily if they are moved, but in ros2_control we usually access them from command_interfaces_
  
  // Helper to map commands
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace my_robot_controllers

#endif  // MY_ROBOT_CONTROLLERS__LEG_IMPEDANCE_CONTROLLER_HPP_


