
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"
#include <algorithm>

// MIT control protocol constants (SteadyWin GIM6010-8 / GIM8108-8)
// Reference: SteadyWin GIM6010-8 Motor Manual rev2.2, §4.1.2 (Mit_Control, cmd_id 0x008)
// Command ranges reflect GIM6010-8 physical limits; the CAN protocol supports wider ranges.
constexpr float MIT_P_MIN = -12.5f, MIT_P_MAX = 12.5f;   // rad
constexpr float MIT_V_MIN = -45.0f, MIT_V_MAX = 45.0f;   // rad/s
constexpr float MIT_KP_MIN = 0.0f,  MIT_KP_MAX = 500.0f;
constexpr float MIT_KD_MIN = 0.0f,  MIT_KD_MAX = 5.0f;
constexpr float MIT_T_MIN = -18.0f, MIT_T_MAX = 18.0f;   // Nm

// Feedback decode constants match the full protocol range from the DBC / manual
constexpr float MIT_FB_POS_SCALE = 0.000381f, MIT_FB_POS_OFFSET = -12.5f;
constexpr float MIT_FB_VEL_SCALE = 0.03175f,  MIT_FB_VEL_OFFSET = -65.0f;
constexpr float MIT_FB_TRQ_SCALE = 0.02442f,  MIT_FB_TRQ_OFFSET = -50.0f;

// Linear float-to-unsigned encoding used by the MIT CAN protocol (big-endian packing)
inline uint16_t mit_float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    x = std::max(x_min, std::min(x_max, x));
    return static_cast<uint16_t>((x - x_min) * static_cast<float>((1 << bits) - 1) / (x_max - x_min));
}

namespace odrive_ros2_control {

class Axis;

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);
    void set_axis_command_mode(const Axis& axis);

    bool active_;
    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id) : can_intf_(can_intf), node_id_(node_id) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    void on_can_msg();

    SocketCanIntf* can_intf_;
    uint32_t node_id_;

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0f;    // [rad]
    double vel_setpoint_ = 0.0f;    // [rad/s]
    double torque_setpoint_ = 0.0f; // [Nm]
    double kp_setpoint_ = 0.0;      // MIT position gain
    double kd_setpoint_ = 0.0;      // MIT damping gain

    // State (ODrives => ros2_control)
    // rclcpp::Time encoder_estimates_timestamp_;
    // uint32_t axis_error_ = 0;
    // uint8_t axis_state_ = 0;
    // uint8_t procedure_result_ = 0;
    // uint8_t trajectory_done_flag_ = 0;
    double pos_estimate_ = NAN; // [rad]
    double vel_estimate_ = NAN; // [rad/s]
    // double iq_setpoint_ = NAN;
    // double iq_measured_ = NAN;
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    // uint32_t active_errors_ = 0;
    // uint32_t disarm_reason_ = 0;
    // double fet_temperature_ = NAN;
    // double motor_temperature_ = NAN;
    // double bus_voltage_ = NAN;
    // double bus_current_ = NAN;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;
    bool mit_input_enabled_ = false; // true when kp+kd interfaces are claimed

    template <typename T>
    void send(const T& msg) const {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_intf_name_ = info_.hardware_parameters["can"];

    for (auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")));
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(
            rclcpp::get_logger("ODriveHardwareInterface"),
            "Failed to initialize SocketCAN on %s",
            can_intf_name_.c_str()
        );
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");

    // This can be called several seconds before the controller finishes starting.
    // Therefore we enable the ODrives only in perform_command_mode_switch().

    active_ = true;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

    active_ = false;
    for (auto& axis : axes_) {
        set_axis_command_mode(axis);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            "kp",
            &axes_[i].kp_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            "kd",
            &axes_[i].kd_setpoint_
        ));
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 5> interfaces = {{
            {info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
            {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
            {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT,   &axis.torque_input_enabled_},
            {info_.joints[i].name + "/kp",                                    &axis.mit_input_enabled_},
            {info_.joints[i].name + "/kd",                                    &axis.mit_input_enabled_}}};

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            set_axis_command_mode(axis);
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    timestamp_ = timestamp;

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (auto& axis : axes_) {
        // MIT mode: pack position, velocity, Kp, Kd, torque_ff into a single 8-byte frame.
        // cmd_id 0x008, big-endian. All values are on the output-shaft side (rad / rad/s / Nm).
        if (axis.mit_input_enabled_) {
            constexpr uint8_t kMITControl = 0x008;
            uint16_t p  = mit_float_to_uint(static_cast<float>(axis.pos_setpoint_),    MIT_P_MIN,  MIT_P_MAX,  16);
            uint16_t v  = mit_float_to_uint(static_cast<float>(axis.vel_setpoint_),    MIT_V_MIN,  MIT_V_MAX,  12);
            uint16_t kp = mit_float_to_uint(static_cast<float>(axis.kp_setpoint_),     MIT_KP_MIN, MIT_KP_MAX, 12);
            uint16_t kd = mit_float_to_uint(static_cast<float>(axis.kd_setpoint_),     MIT_KD_MIN, MIT_KD_MAX, 12);
            uint16_t t  = mit_float_to_uint(static_cast<float>(axis.torque_setpoint_), MIT_T_MIN,  MIT_T_MAX,  12);

            struct can_frame frame{};
            frame.can_id  = axis.node_id_ << 5 | kMITControl;
            frame.can_dlc = 8;
            frame.data[0] = (p >> 8) & 0xFF;
            frame.data[1] =  p & 0xFF;
            frame.data[2] = (v >> 4) & 0xFF;
            frame.data[3] = ((v & 0x0F) << 4) | ((kp >> 8) & 0x0F);
            frame.data[4] =  kp & 0xFF;
            frame.data[5] = (kd >> 4) & 0xFF;
            frame.data[6] = ((kd & 0x0F) << 4) | ((t >> 8) & 0x0F);
            frame.data[7] =  t & 0xFF;
            axis.can_intf_->send_can_frame(frame);
            continue;
        }

        // Send the CAN message that fits the set of enabled setpoints
        if (axis.pos_input_enabled_) {
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = axis.pos_setpoint_ / (2 * M_PI);
            msg.Vel_FF = axis.vel_input_enabled_ ? (axis.vel_setpoint_ / (2 * M_PI)) : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel = axis.vel_setpoint_ / (2 * M_PI);
            msg.Input_Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = axis.torque_setpoint_;
            axis.send(msg);
        } else {
            // no control enabled - don't send any setpoint
        }
    }

    return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void ODriveHardwareInterface::set_axis_command_mode(const Axis& axis) {
    if (!active_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Interface inactive. Setting axis to idle.");
        Set_Axis_State_msg_t idle_msg;
        idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(idle_msg);
        return;
    }

    Clear_Errors_msg_t clear_error_msg;
    clear_error_msg.Identify = 0;
    Set_Axis_State_msg_t state_msg;
    state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

    if (axis.mit_input_enabled_) {
        // MIT mode: the firmware handles 0x008 frames without a prior Set_Controller_Mode.
        // Just clear errors and enter closed-loop; MIT frames drive the motor from write().
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to MIT control.");
        axis.send(clear_error_msg);
        axis.send(state_msg);
        return;
    }

    Set_Controller_Mode_msg_t control_msg;
    control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;

    if (axis.pos_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to position control.");
        control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
    } else if (axis.vel_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to velocity control.");
        control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
    } else if (axis.torque_input_enabled_) {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting to torque control.");
        control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "No control mode specified. Setting to idle.");
        state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(state_msg);
        return;
    }

    axis.send(control_msg);
    axis.send(clear_error_msg);
    axis.send(state_msg);
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
        case 0x008: {
            // MIT feedback frame (motor → host), 6 bytes, big-endian.
            // Byte 0: motor_id; Bytes 1-2: position (16-bit); Bytes 3-4[7:4]: velocity (12-bit);
            // Bytes 4[3:0]-5: torque (12-bit).
            if (frame.can_dlc < 6) {
                RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "MIT feedback frame too short");
                break;
            }
            uint16_t p_raw = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2];
            uint16_t v_raw = (static_cast<uint16_t>(frame.data[3]) << 4) | ((frame.data[4] >> 4) & 0x0F);
            uint16_t t_raw = ((static_cast<uint16_t>(frame.data[4]) & 0x0F) << 8) | frame.data[5];
            pos_estimate_    = static_cast<double>(p_raw) * MIT_FB_POS_SCALE + MIT_FB_POS_OFFSET;
            vel_estimate_    = static_cast<double>(v_raw) * MIT_FB_VEL_SCALE + MIT_FB_VEL_OFFSET;
            torque_estimate_ = static_cast<double>(t_raw) * MIT_FB_TRQ_SCALE + MIT_FB_TRQ_OFFSET;
        } break;
            // silently ignore unimplemented command IDs
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
