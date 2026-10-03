#include "odrive_mit_example/msg/leg_cmd.hpp"
#include "my_robot_controllers/leg_impedance_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cmath>
#include <cstdio>
#include <sstream>

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
  double timeout_s = 0.5;
  if (!node->has_parameter("cmd_timeout_s")) {
      node->declare_parameter("cmd_timeout_s", 0.5);
  }
  timeout_s = node->get_parameter("cmd_timeout_s").as_double();
  cmd_timeout_ = rclcpp::Duration::from_seconds(timeout_s);

  // Hard joint-velocity cap (rad/s); 0 = disabled. Caps velocity_des AND slew-limits position_des.
  if (!node->has_parameter("max_joint_velocity")) {
      node->declare_parameter("max_joint_velocity", 0.0);
  }
  max_joint_velocity_ = node->get_parameter("max_joint_velocity").as_double();
  prev_pos_cmd_.assign(joint_names_.size(), 0.0);
  if (max_joint_velocity_ > 0.0) {
      RCLCPP_WARN(node->get_logger(),
        "max_joint_velocity = %.3f rad/s -> velocity cap + position slew-limit ACTIVE", max_joint_velocity_);
  } else {
      RCLCPP_INFO(node->get_logger(), "max_joint_velocity disabled (0); URDF velocity limits only.");
  }

  // ---- Pre-flight / engage interlock parameters ----
  // The controller stays LIMP (zero torque) and refuses to apply gains until these checks
  // all pass on live data. See run_preflight().
  auto declare_if = [&](const std::string & nm, auto def) {
      if (!node->has_parameter(nm)) node->declare_parameter(nm, def);
  };
  declare_if("preflight_enabled", true);
  declare_if("engage_pos_tol", 0.2);
  declare_if("engage_vel_thresh", 0.2);
  declare_if("engage_settle_cycles", 5);
  declare_if("require_upright", false);
  declare_if("upright_max_tilt_deg", 25.0);
  declare_if("imu_timeout_s", 0.5);
  declare_if("engage_timeout_s", 8.0);
  declare_if("imu_topic", std::string("/imu"));
  declare_if("fall_limp_enabled", false);
  declare_if("fall_tilt_deg", 50.0);
  preflight_enabled_    = node->get_parameter("preflight_enabled").as_bool();
  engage_pos_tol_       = node->get_parameter("engage_pos_tol").as_double();
  engage_vel_thresh_    = node->get_parameter("engage_vel_thresh").as_double();
  engage_settle_cycles_ = static_cast<int>(node->get_parameter("engage_settle_cycles").as_int());
  require_upright_      = node->get_parameter("require_upright").as_bool();
  upright_max_tilt_deg_ = node->get_parameter("upright_max_tilt_deg").as_double();
  imu_timeout_s_        = node->get_parameter("imu_timeout_s").as_double();
  engage_timeout_s_     = node->get_parameter("engage_timeout_s").as_double();
  imu_topic_            = node->get_parameter("imu_topic").as_string();
  fall_limp_enabled_    = node->get_parameter("fall_limp_enabled").as_bool();
  fall_tilt_deg_        = node->get_parameter("fall_tilt_deg").as_double();
  RCLCPP_INFO(node->get_logger(),
    "[preflight] enabled=%d pos_tol=%.2f vel_thr=%.2f settle=%d upright=%d(tilt<%.0f) timeout=%.1fs",
    preflight_enabled_, engage_pos_tol_, engage_vel_thresh_, engage_settle_cycles_,
    require_upright_, upright_max_tilt_deg_, engage_timeout_s_);
  if (fall_limp_enabled_) {
      RCLCPP_WARN(node->get_logger(),
        "[fall-guard] ON — latch LIMP if torso tilt > %.0f deg while engaged", fall_tilt_deg_);
  }

  if (require_upright_ || fall_limp_enabled_) {
      sub_imu_ = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(imu_mutex_);
            imu_qx_ = msg->orientation.x; imu_qy_ = msg->orientation.y;
            imu_qz_ = msg->orientation.z; imu_qw_ = msg->orientation.w;
            last_imu_time_ = get_node()->now();
            have_imu_ = true;
        });
      RCLCPP_INFO(node->get_logger(),
        "[preflight] subscribing IMU '%s' (upright=%d fall-guard=%d)",
        imu_topic_.c_str(), require_upright_, fall_limp_enabled_);
  }

  // Dashboard status (~10 Hz JSON on ~/preflight_status)
  pub_status_ = node->create_publisher<std_msgs::msg::String>("~/preflight_status", 10);

  // Operator hard-stop: any 'true' on ~/estop latches the controller LIMP (zero torque,
  // ignores all commands) until re-activation (the dashboard RE-ARM clears it).
  sub_estop_ = node->create_subscription<std_msgs::msg::Bool>(
    "~/estop", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::SharedPtr m) {
      if (m->data) {
        engage_state_ = EngageState::ESTOP;
        preflight_reason_ = "E-STOP (operator)";
        RCLCPP_ERROR(get_node()->get_logger(), "[E-STOP] operator hard-stop — LIMP, ignoring commands");
      }
    });

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
  // Static pre-flight: the claimed interface counts must match (5 command + 3 state per
  // joint). If not, REFUSE TO ACTIVATE — something is mis-wired and indexing would be unsafe.
  if (command_interfaces_.size() != joint_names_.size() * 5 ||
      state_interfaces_.size() != joint_names_.size() * 3) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "[preflight] interface mismatch (cmd %zu/exp %zu, state %zu/exp %zu) — REFUSING TO ACTIVATE",
        command_interfaces_.size(), joint_names_.size() * 5,
        state_interfaces_.size(), joint_names_.size() * 3);
      return controller_interface::CallbackReturn::ERROR;
  }

  // Set last_cmd_time_ far in the past so the timeout triggers immediately
  // on activation — motor stays limp until first command arrives.
  last_cmd_time_ = get_node()->now() - cmd_timeout_ - rclcpp::Duration::from_seconds(1.0);
  slew_init_ = false;  // re-initialise the slew baseline to the measured position on first update

  // Reset the engage interlock. With preflight on we start VALIDATING (limp) and only
  // engage gains once run_preflight() passes; with it off we engage immediately (legacy).
  engage_state_ = preflight_enabled_ ? EngageState::VALIDATING : EngageState::ENGAGED;
  settle_count_ = 0;
  engage_start_time_ = get_node()->now();
  last_block_log_ = get_node()->now();
  if (preflight_enabled_) {
      RCLCPP_INFO(get_node()->get_logger(),
        "[preflight] activated LIMP — validating before engage (timeout %.1fs after first cmd)",
        engage_timeout_s_);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LegImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LegImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  publish_status(time);   // ~10 Hz JSON for the dashboard (throttled inside)

  // ---- Latched LIMP: operator E-STOP or a fall. Zero torque, ignore commands, until re-arm ----
  if (engage_state_ == EngageState::ESTOP || engage_state_ == EngageState::FAULT_LATCHED) {
      write_limp();
      if ((time - last_block_log_) > rclcpp::Duration::from_seconds(2.0)) {
          RCLCPP_ERROR(get_node()->get_logger(), "%s",
            engage_state_ == EngageState::ESTOP
              ? "[E-STOP] LIMP — re-arm to recover"
              : "[fall-guard] LATCHED LIMP — re-activate to recover");
          last_block_log_ = time;
      }
      return controller_interface::return_type::OK;
  }

  // ---- Pre-flight engage interlock ----------------------------------------
  // While VALIDATING, command zero torque (passive) and run the safety checks on live
  // data. Don't apply gains until they all pass. If they don't within engage_timeout_s_
  // (timed from the FIRST command, so waiting for the operator is fine), return ERROR and
  // the controller_manager deactivates us — we REFUSE to engage from an unsafe state.
  if (preflight_enabled_ && engage_state_ == EngageState::VALIDATING) {
      write_limp();
      bool have_cmd = has_new_cmd_ && ((time - last_cmd_time_) <= cmd_timeout_);
      if (!have_cmd) {
          engage_start_time_ = time;   // idle (no command yet): don't run the engage clock
          return controller_interface::return_type::OK;
      }
      std::string reason;
      if (run_preflight(time, reason)) {
          engage_state_ = EngageState::ENGAGED;
          preflight_reason_ = "engaged";
          RCLCPP_INFO(get_node()->get_logger(), "[preflight] all checks PASSED — engaging gains");
      } else {
          preflight_reason_ = reason;
          if ((time - engage_start_time_) > rclcpp::Duration::from_seconds(engage_timeout_s_)) {
              RCLCPP_ERROR(get_node()->get_logger(),
                "[preflight] REFUSING TO ENGAGE (%.1fs elapsed): %s", engage_timeout_s_, reason.c_str());
              return controller_interface::return_type::ERROR;
          } else if ((time - last_block_log_) > rclcpp::Duration::from_seconds(1.0)) {
              RCLCPP_WARN(get_node()->get_logger(), "[preflight] holding limp: %s", reason.c_str());
              last_block_log_ = time;
          }
      }
      return controller_interface::return_type::OK;
  }

  // ---- Runtime fall guard (ENGAGED) ---------------------------------------
  // If the torso tips past fall_tilt_deg_, latch LIMP so we never write jerky corrections
  // to a toppling robot. Latched until the controller is re-activated.
  if (fall_limp_enabled_) {
      double tilt;
      if (current_tilt_deg(time, tilt) && tilt > fall_tilt_deg_) {
          engage_state_ = EngageState::FAULT_LATCHED;
          last_block_log_ = time - rclcpp::Duration::from_seconds(2.0);
          {
              char rb[96];
              std::snprintf(rb, sizeof(rb), "FALL: tilt %.0f > %.0f deg — latched limp", tilt, fall_tilt_deg_);
              preflight_reason_ = rb;
          }
          write_limp();
          RCLCPP_ERROR(get_node()->get_logger(),
            "[fall-guard] FALL DETECTED — tilt %.0f deg > %.0f — LATCHING LIMP (re-activate to recover)",
            tilt, fall_tilt_deg_);
          return controller_interface::return_type::OK;
      }
  }

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

  // Real velocity cap: dt for the position slew-limit (clamped vs the MEASURED position below).
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) dt = 0.02;  // guard against an anomalous first/large period

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

     // Real velocity cap (rad/s): clamp desired velocity, and limit the position command to
     // within cap*dt of the MEASURED position so it can never run ahead of a lagging joint.
     // Bounding the position error this way bounds the actual joint speed (kills the overshoot).
     if (max_joint_velocity_ > 0.0) {
         if (vel_cmd >  max_joint_velocity_) vel_cmd =  max_joint_velocity_;
         if (vel_cmd < -max_joint_velocity_) vel_cmd = -max_joint_velocity_;
         double max_step = max_joint_velocity_ * dt;
         double meas = state_interfaces_[i * 3].get_value() / joint_directions_[i] - joint_offsets_[i];  // measured, URDF frame
         if (pos_cmd > meas + max_step) pos_cmd = meas + max_step;
         else if (pos_cmd < meas - max_step) pos_cmd = meas - max_step;
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

void LegImpedanceController::write_limp()
{
  // Zero-torque passive command: kp=kd=0 (no torque regardless of position/velocity),
  // and command the MEASURED position so there is no step when gains later engage.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
      int base = static_cast<int>(i) * 5;
      double meas_hw = state_interfaces_[i * 3].get_value();
      if (!std::isfinite(meas_hw)) meas_hw = 0.0;
      command_interfaces_[base + 0].set_value(meas_hw);  // position (held; kp=0 so inert)
      command_interfaces_[base + 1].set_value(0.0);      // velocity
      command_interfaces_[base + 2].set_value(0.0);      // effort
      command_interfaces_[base + 3].set_value(0.0);      // kp
      command_interfaces_[base + 4].set_value(0.0);      // kd
  }
}

bool LegImpedanceController::current_tilt_deg(const rclcpp::Time & time, double & tilt_deg)
{
  // Torso tilt from world-vertical, from the latest IMU orientation. Returns false if no
  // fresh IMU. cos(tilt) = R22 = 1 - 2(qx^2 + qy^2) (angle of the body up-axis from vertical).
  double qx, qy; rclcpp::Time t_imu; bool have;
  {
      std::lock_guard<std::mutex> lk(imu_mutex_);
      qx = imu_qx_; qy = imu_qy_; t_imu = last_imu_time_; have = have_imu_;
  }
  if (!have || (time - t_imu) > rclcpp::Duration::from_seconds(imu_timeout_s_)) return false;
  double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
  r22 = std::max(-1.0, std::min(1.0, r22));
  tilt_deg = std::acos(r22) * 180.0 / M_PI;
  return true;
}

bool LegImpedanceController::run_preflight(const rclcpp::Time & time, std::string & reason)
{
  const size_t n = joint_names_.size();

  // (1) A fresh command must exist (needed for the anti-jump comparison below).
  if (!has_new_cmd_ || (time - last_cmd_time_) > cmd_timeout_ ||
      latest_cmd_.position_des.size() < n) {
      reason = "waiting for command"; return false;
  }

  // (2) Encoder sanity: every measured position/velocity finite and in range.
  for (size_t i = 0; i < n; ++i) {
      double meas = state_interfaces_[i * 3].get_value() / joint_directions_[i] - joint_offsets_[i];
      double vel  = state_interfaces_[i * 3 + 1].get_value();
      if (!std::isfinite(meas) || !std::isfinite(vel)) {
          reason = "encoder NaN (" + joint_names_[i] + ")"; settle_count_ = 0; return false;
      }
      if (joint_limits_[i].has_position_limits &&
          (meas < joint_limits_[i].min_position - 0.05 || meas > joint_limits_[i].max_position + 0.05)) {
          reason = "measured pos out of limits (" + joint_names_[i] + ")"; settle_count_ = 0; return false;
      }
  }

  // (3) Live position: require several consecutive valid cycles so the motors have been
  // in passive closed-loop long enough to stream live (not stale/cached) encoder values.
  if (settle_count_ < engage_settle_cycles_) {
      settle_count_++;
      reason = "settling (" + std::to_string(settle_count_) + "/" +
               std::to_string(engage_settle_cycles_) + ")"; return false;
  }

  // (4) At rest: no joint moving faster than the threshold.
  for (size_t i = 0; i < n; ++i) {
      double vel = state_interfaces_[i * 3 + 1].get_value();
      if (std::abs(vel) > engage_vel_thresh_) {
          reason = "joint moving (" + joint_names_[i] + ")"; return false;
      }
  }

  // (5) Anti-jump: the first commanded position must be near the measured position, so
  // engaging gains can't slam the joint from a stale/standing pose to a far target.
  for (size_t i = 0; i < n; ++i) {
      double meas = state_interfaces_[i * 3].get_value() / joint_directions_[i] - joint_offsets_[i];
      double cmd  = latest_cmd_.position_des[i];
      if (std::abs(cmd - meas) > engage_pos_tol_) {
          char buf[160];
          std::snprintf(buf, sizeof(buf), "anti-jump %s: cmd %.2f vs meas %.2f (tol %.2f)",
                        joint_names_[i].c_str(), cmd, meas, engage_pos_tol_);
          reason = buf; return false;
      }
  }

  // (6) Upright (optional): torso tilt from vertical below the limit, from a fresh IMU.
  if (require_upright_) {
      double tilt;
      if (!current_tilt_deg(time, tilt)) { reason = "no fresh IMU"; return false; }
      if (tilt > upright_max_tilt_deg_) {
          char buf[96];
          std::snprintf(buf, sizeof(buf), "not upright: tilt %.0f deg > %.0f", tilt, upright_max_tilt_deg_);
          reason = buf; return false;
      }
  }

  reason = "ok";
  return true;
}

void LegImpedanceController::publish_status(const rclcpp::Time & time)
{
  if (!pub_status_) return;
  if ((time - last_status_pub_) < rclcpp::Duration::from_seconds(0.1)) return;  // ~10 Hz
  last_status_pub_ = time;

  const size_t n = joint_names_.size();
  const char * st = "ENGAGED";
  if (engage_state_ == EngageState::VALIDATING) st = "VALIDATING";
  else if (engage_state_ == EngageState::FAULT_LATCHED) st = "FAULT_LATCHED";
  else if (engage_state_ == EngageState::ESTOP) st = "ESTOP";

  // Current truth of each check (read-only — for the dashboard lights).
  bool have_cmd = has_new_cmd_ && (time - last_cmd_time_) <= cmd_timeout_ &&
                  latest_cmd_.position_des.size() >= n;
  bool enc_ok = true, atrest_ok = true, antijump_ok = have_cmd;
  for (size_t i = 0; i < n; ++i) {
      double meas = state_interfaces_[i * 3].get_value() / joint_directions_[i] - joint_offsets_[i];
      double vel  = state_interfaces_[i * 3 + 1].get_value();
      if (!std::isfinite(meas) || !std::isfinite(vel)) enc_ok = false;
      else if (joint_limits_[i].has_position_limits &&
               (meas < joint_limits_[i].min_position - 0.05 || meas > joint_limits_[i].max_position + 0.05))
          enc_ok = false;
      if (std::abs(vel) > engage_vel_thresh_) atrest_ok = false;
      if (have_cmd && std::abs(latest_cmd_.position_des[i] - meas) > engage_pos_tol_) antijump_ok = false;
  }
  bool settle_ok = settle_count_ >= engage_settle_cycles_;
  double tilt = 0.0; bool tilt_fresh = current_tilt_deg(time, tilt);
  bool upright_ok = require_upright_ ? (tilt_fresh && tilt <= upright_max_tilt_deg_) : true;

  std::ostringstream js;
  js.setf(std::ios::fixed); js.precision(2);
  js << "{\"state\":\"" << st << "\",\"preflight_enabled\":" << (preflight_enabled_ ? "true" : "false")
     << ",\"reason\":\"" << preflight_reason_ << "\",\"checks\":{"
     << "\"cmd\":" << (have_cmd ? "true" : "false")
     << ",\"encoder\":" << (enc_ok ? "true" : "false")
     << ",\"settle\":" << (settle_ok ? "true" : "false")
     << ",\"at_rest\":" << (atrest_ok ? "true" : "false")
     << ",\"anti_jump\":" << (antijump_ok ? "true" : "false")
     << ",\"upright\":" << (upright_ok ? "true" : "false") << "}"
     << ",\"settle\":" << settle_count_ << ",\"settle_target\":" << engage_settle_cycles_
     << ",\"tilt_deg\":" << tilt << ",\"tilt_valid\":" << (tilt_fresh ? "true" : "false")
     << ",\"cfg\":{\"pos_tol\":" << engage_pos_tol_ << ",\"vel_thr\":" << engage_vel_thresh_
     << ",\"upright_max\":" << upright_max_tilt_deg_ << ",\"engage_timeout\":" << engage_timeout_s_
     << ",\"max_joint_velocity\":" << max_joint_velocity_
     << ",\"fall_guard\":" << (fall_limp_enabled_ ? "true" : "false")
     << ",\"fall_tilt\":" << fall_tilt_deg_
     << ",\"require_upright\":" << (require_upright_ ? "true" : "false") << "}}";

  std_msgs::msg::String msg; msg.data = js.str();
  pub_status_->publish(msg);
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