#include "odrive_mit_example/msg/leg_cmd.hpp"
#include "my_robot_controllers/leg_impedance_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_robot_controllers
{

LegImpedanceController::LegImpedanceController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration LegImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Request all 5 interfaces for all 3 joints
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
    config.names.push_back(joint + "/effort");
    config.names.push_back(joint + "/kp");
    config.names.push_back(joint + "/kd");
  }
  return config;
}

controller_interface::InterfaceConfiguration LegImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::CallbackReturn LegImpedanceController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LegImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joint_names_ = node->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_offsets_.resize(joint_names_.size(), 0.0);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
      std::string param_name = "actuator_offsets." + joint_names_[i];
      if (node->has_parameter(param_name)) {
          joint_offsets_[i] = node->get_parameter(param_name).as_double();
          RCLCPP_INFO(node->get_logger(), "Loaded offset for joint %s: %f", joint_names_[i].c_str(), joint_offsets_[i]);
      } else {
          // Try to declare it if not already declared (though get_parameter usually works if declared in YAML)
          // To be safe, we declare with default 0.0
          try {
              joint_offsets_[i] = node->declare_parameter<double>(param_name, 0.0);
              // If it was in YAML but not declared, declare_parameter returns the YAML value
              RCLCPP_INFO(node->get_logger(), "Loaded offset for joint %s: %f (defaulted/declared)", joint_names_[i].c_str(), joint_offsets_[i]);
          } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
              joint_offsets_[i] = node->get_parameter(param_name).as_double();
              RCLCPP_INFO(node->get_logger(), "Loaded offset for joint %s: %f", joint_names_[i].c_str(), joint_offsets_[i]);
          }
      }
  }

  // Load Actuator Directions
  joint_directions_.resize(joint_names_.size(), 1.0);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
      std::string param_name = "actuator_directions." + joint_names_[i];
      if (node->has_parameter(param_name)) {
          joint_directions_[i] = node->get_parameter(param_name).as_double();
          RCLCPP_INFO(node->get_logger(), "Loaded direction for joint %s: %f", joint_names_[i].c_str(), joint_directions_[i]);
      } else {
          try {
              joint_directions_[i] = node->declare_parameter<double>(param_name, 1.0);
              RCLCPP_INFO(node->get_logger(), "Loaded direction for joint %s: %f (defaulted/declared)", joint_names_[i].c_str(), joint_directions_[i]);
          } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
              joint_directions_[i] = node->get_parameter(param_name).as_double();
              RCLCPP_INFO(node->get_logger(), "Loaded direction for joint %s: %f", joint_names_[i].c_str(), joint_directions_[i]);
          }
      }
  }

  // Load Joint Limits
  joint_limits_.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // Attempt to get joint limits from URDF via parameter server
    // The "robot_description" is usually available on the controller_manager, 
    // but often controllers can access it if they are on the same node structure 
    // or if we explicitly look for the parameter on the appropriate node.
    // 
    // joint_limits::get_joint_limits methods typically look for "joint_limits" namespace parameters 
    // OR require the URDF string.
    
    // We will attempt to find the robot_description parameter
    // If not found, we fall back to the safe defaults.
    
    bool limits_found = false;
    try {
        // Option 1: Try parameter server limits (joint_limits.joint_name...)
        if (joint_limits::get_joint_limits(joint_names_[i], node, joint_limits_[i])) {
            limits_found = true;
            RCLCPP_INFO(node->get_logger(), "Loaded limits for joint %s from parameters.", joint_names_[i].c_str());
        } else {
            // Option 2: Try parsing from URDF parameter 'robot_description' on the node
            // This is how most controllers do it if the joint_limits namespace params aren't set
            if (node->has_parameter("robot_description")) {
                std::string urdf_string = node->get_parameter("robot_description").as_string();
                urdf::Model urdf_model;
                if (urdf_model.initString(urdf_string)) {
                    auto urdf_joint = urdf_model.getJoint(joint_names_[i]);
                    // Found the function name: getJointLimits (camelCase) instead of get_joint_limits (snake_case) for URDF version
                    // Note: parameter version uses get_joint_limits (snake_case)
                    if (urdf_joint && joint_limits::getJointLimits(urdf_joint, joint_limits_[i])) {
                        limits_found = true;
                        RCLCPP_INFO(node->get_logger(), "Loaded limits for joint %s from URDF.", joint_names_[i].c_str());
                    }
                }
            }
        }
    } catch (const std::exception & e) {
        RCLCPP_WARN(node->get_logger(), "Error loading limits for %s: %s", joint_names_[i].c_str(), e.what());
    }

    if (!limits_found) {
        RCLCPP_ERROR(node->get_logger(), "No limits found for joint %s! Limits are required in URDF or parameters.", joint_names_[i].c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Timeout after which kp/kd are zeroed if no command is received (limp fallback)
  double timeout_s = node->declare_parameter("cmd_timeout_s", 0.5);
  cmd_timeout_ = rclcpp::Duration::from_seconds(timeout_s);

  // Subscribe to command
  sub_command_ = node->create_subscription<odrive_mit_example::msg::LegCmd>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    [this](const odrive_mit_example::msg::LegCmd::SharedPtr msg) {
      latest_cmd_ = *msg;
      last_cmd_time_ = get_node()->now();
      has_new_cmd_ = true;
    });
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LegImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set last_cmd_time_ far in the past so the timeout triggers immediately
  // on activation — motor stays limp until first command arrives.
  last_cmd_time_ = get_node()->now() - cmd_timeout_ - rclcpp::Duration::from_seconds(1.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LegImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LegImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!has_new_cmd_) {
      return controller_interface::return_type::OK;
  }

  // If no command has arrived within cmd_timeout_s, go limp (zero kp, kd, vel, effort).
  // This prevents the motor staying stiff if the command publisher crashes.
  if ((time - last_cmd_time_) > cmd_timeout_) {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
          int base = static_cast<int>(i) * 5;
          command_interfaces_[base + 1].set_value(0.0);  // velocity
          command_interfaces_[base + 2].set_value(0.0);  // effort
          command_interfaces_[base + 3].set_value(0.0);  // kp
          command_interfaces_[base + 4].set_value(0.0);  // kd
      }
      return controller_interface::return_type::OK;
  }

  // Map Message Data -> Hardware Handles
  // Handles are stored in command_interfaces_ vector in the order we requested them
  // Structure: [Hip_Pos, Hip_Vel, Hip_Eff, Hip_Kp, Hip_Kd, Knee_Pos, ...]
  
  int handle_idx = 0;
  // Use joint_names_.size() instead of fixed 3
  for (size_t i = 0; i < joint_names_.size(); ++i) {
     // Safety check for message size
     if (i >= latest_cmd_.position_des.size() || 
         i >= latest_cmd_.velocity_des.size() ||
         i >= latest_cmd_.feedforward_torque.size() ||
         i >= latest_cmd_.kp_scale.size() ||
         i >= latest_cmd_.kd_scale.size()) {
         break; // Or log error? Breaking to avoid segfault.
     }

     // Enforce Position Limits
     double pos_cmd = latest_cmd_.position_des[i];
     if (joint_limits_[i].has_position_limits) {
         if (pos_cmd > joint_limits_[i].max_position) {
             pos_cmd = joint_limits_[i].max_position;
         } else if (pos_cmd < joint_limits_[i].min_position) {
             pos_cmd = joint_limits_[i].min_position;
         }
     }
     
     // Enforce Velocity Limits (Saturate des velocity)
     double vel_cmd = latest_cmd_.velocity_des[i];
     if (joint_limits_[i].has_velocity_limits) {
         if (vel_cmd > joint_limits_[i].max_velocity) {
             vel_cmd = joint_limits_[i].max_velocity;
         } else if (vel_cmd < -joint_limits_[i].max_velocity) {
             vel_cmd = -joint_limits_[i].max_velocity;
         }
     }
     
     // Enforce Effort Limits (Feedforward)
     double eff_cmd = latest_cmd_.feedforward_torque[i];
     if (joint_limits_[i].has_effort_limits) {
         if (eff_cmd > joint_limits_[i].max_effort) {
             eff_cmd = joint_limits_[i].max_effort;
         } else if (eff_cmd < -joint_limits_[i].max_effort) {
             eff_cmd = -joint_limits_[i].max_effort;
         }
     }

     // Apply Offset and Direction to Position Command
     // hardware_pos = (urdf_pos + offset) * direction
     pos_cmd = (pos_cmd + joint_offsets_[i]) * joint_directions_[i];
     
     // Apply Direction to Velocity and Effort
     vel_cmd *= joint_directions_[i];
     eff_cmd *= joint_directions_[i];

     command_interfaces_[handle_idx++].set_value(pos_cmd);
     command_interfaces_[handle_idx++].set_value(vel_cmd);
     command_interfaces_[handle_idx++].set_value(eff_cmd);
     command_interfaces_[handle_idx++].set_value(latest_cmd_.kp_scale[i]);
     command_interfaces_[handle_idx++].set_value(latest_cmd_.kd_scale[i]);
  }

  return controller_interface::return_type::OK;
}

void LegImpedanceController::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // implementation
    (void)msg;
}

}  // namespace my_robot_controllers

PLUGINLIB_EXPORT_CLASS(
  my_robot_controllers::LegImpedanceController,
  controller_interface::ControllerInterface)