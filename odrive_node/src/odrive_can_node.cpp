#include "odrive_can_node.hpp"
#include "odrive_enums.h"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>
#include <algorithm>
#include <cstdint>

enum CmdId : uint32_t {
    kHeartbeat = 0x001,            // ControllerStatus  - publisher
    kGetError = 0x003,             // SystemStatus      - publisher
    kSetAxisState = 0x007,         // SetAxisState      - service
    kMITControl = 0x008,           // MIT Control       - subscriber
    kGetEncoderEstimates = 0x009,  // ControllerStatus  - publisher
    kSetControllerMode = 0x00b,    // ControlMessage    - subscriber
    kSetInputPos,                  // ControlMessage    - subscriber
    kSetInputVel,                  // ControlMessage    - subscriber
    kSetInputTorque,               // ControlMessage    - subscriber
    kSetLimits = 0x00F,            // SetLimits         - service
    kGetIq = 0x014,                // ControllerStatus  - publisher
    kGetTemp,                      // SystemStatus      - publisher
    kGetBusVoltageCurrent = 0x017, // SystemStatus      - publisher
    kClearErrors = 0x018,          // ClearErrors       - service
    kGetTorques = 0x01c,           // ControllerStatus  - publisher
};

// MIT Control Protocol Constants (SteadyWin GIM6010-8)
// Reference: SteadyWin GIM6010-8 Motor Manual rev2.2 - CAN MIT Protocol
// Command scaling (for sending commands)
constexpr float MIT_P_MIN = -12.5f;    // Minimum position [rad]
constexpr float MIT_P_MAX = 12.5f;     // Maximum position [rad]
constexpr float MIT_V_MIN = -45.0f;    // Minimum velocity [rad/s]
constexpr float MIT_V_MAX = 45.0f;     // Maximum velocity [rad/s]
constexpr float MIT_KP_MIN = 0.0f;     // Minimum Kp
constexpr float MIT_KP_MAX = 500.0f;   // Maximum Kp
constexpr float MIT_KD_MIN = 0.0f;     // Minimum Kd
constexpr float MIT_KD_MAX = 5.0f;     // Maximum Kd
constexpr float MIT_T_MIN = -18.0f;    // Minimum torque [Nm]
constexpr float MIT_T_MAX = 18.0f;     // Maximum torque [Nm]

// MIT Feedback scaling (from DBC file - Axis0_Mit_Feedback)
// Fb_Position: 16 bits, scale 0.000381, offset -12.5, range [-12.5, 12.5] rad
// Fb_Velocity: 12 bits, scale 0.03175, offset -65, range [-65, 65] rad/s
// Fb_Torque:   12 bits, scale 0.02442, offset -50, range [-50, 50] Nm
constexpr float MIT_FB_POS_SCALE = 0.000381f;
constexpr float MIT_FB_POS_OFFSET = -12.5f;
constexpr float MIT_FB_VEL_SCALE = 0.03175f;
constexpr float MIT_FB_VEL_OFFSET = -65.0f;
constexpr float MIT_FB_TRQ_SCALE = 0.02442f;
constexpr float MIT_FB_TRQ_OFFSET = -50.0f;

// MIT Protocol helper functions
inline float clamp(float val, float min_val, float max_val) {
    return std::max(min_val, std::min(max_val, val));
}

// Convert float to unsigned int with linear scaling
inline uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x_min;
    x = clamp(x, x_min, x_max);
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// Convert unsigned int to float with linear scaling
inline float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

enum ControlMode : uint64_t {
    kVoltageControl,
    kTorqueControl,
    kVelocityControl,
    kPositionControl,
    kMITControlMode,  // MIT Control Mode (SteadyWin GIM6010-8)
};

ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {
    
    rclcpp::Node::declare_parameter<std::string>("interface", "can0");
    rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);
    rclcpp::Node::declare_parameter<bool>("axis_idle_on_shutdown", false);

    rclcpp::QoS ctrl_stat_qos(rclcpp::KeepAll{});
    ctrl_publisher_ = rclcpp::Node::create_publisher<ControllerStatus>("controller_status", ctrl_stat_qos);
    
    rclcpp::QoS odrv_stat_qos(rclcpp::KeepAll{});
    odrv_publisher_ = rclcpp::Node::create_publisher<ODriveStatus>("odrive_status", odrv_stat_qos);

    rclcpp::QoS ctrl_msg_qos(rclcpp::KeepAll{});
    subscriber_ = rclcpp::Node::create_subscription<ControlMessage>("control_message", ctrl_msg_qos, std::bind(&ODriveCanNode::subscriber_callback, this, _1));

    rclcpp::QoS srv_qos(rclcpp::KeepAll{});

#if RCLCPP_VERSION_MAJOR >= 18 
    // For ros2 jazzy and above. 
    // PR about deprecation of get_rmw_qos_profile: 
    //  - https://github.com/ros2/rclcpp/pull/713
    //  - https://github.com/ros2/rclcpp/pull/1969
    auto srv_qos_profile = srv_qos;
#else
    auto srv_qos_profile = srv_qos.get_rmw_qos_profile();
#endif

    service_ = rclcpp::Node::create_service<AxisState>("request_axis_state", std::bind(&ODriveCanNode::service_callback, this, _1, _2), srv_qos_profile);
    service_clear_errors_ = rclcpp::Node::create_service<Empty>("clear_errors", std::bind(&ODriveCanNode::service_clear_errors_callback, this, _1, _2), srv_qos_profile);
    service_set_limits_ = rclcpp::Node::create_service<SetLimits>("set_limits", std::bind(&ODriveCanNode::service_set_limits_callback, this, _1, _2), srv_qos_profile);
}

void ODriveCanNode::deinit() {
    if (axis_idle_on_shutdown_) {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
        write_le<uint32_t>(ODriveAxisState::AXIS_STATE_IDLE, frame.data);
        frame.can_dlc = 4;
        can_intf_.send_can_frame(frame);
    }

    sub_evt_.deinit();
    srv_evt_.deinit();
    can_intf_.deinit();
}

bool ODriveCanNode::init(EpollEventLoop* event_loop) {

    node_id_ = rclcpp::Node::get_parameter("node_id").as_int();
    axis_idle_on_shutdown_ = rclcpp::Node::get_parameter("axis_idle_on_shutdown").as_bool();
    std::string interface = rclcpp::Node::get_parameter("interface").as_string();

    if (!can_intf_.init(interface, event_loop, std::bind(&ODriveCanNode::recv_callback, this, _1))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize socket can interface: %s", interface.c_str());
        return false;
    }
    if (!sub_evt_.init(event_loop, std::bind(&ODriveCanNode::ctrl_msg_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize subscriber event");
        return false;
    }
    if (!srv_evt_.init(event_loop, std::bind(&ODriveCanNode::request_state_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize service event");
        return false;
    }
    if (!srv_clear_errors_evt_.init(event_loop, std::bind(&ODriveCanNode::request_clear_errors_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize clear errors service event");
        return false;
    }
    if (!srv_set_limits_evt_.init(event_loop, std::bind(&ODriveCanNode::request_set_limits_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize set limits service event");
        return false;
    }
    RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d", node_id_);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "interface: %s", interface.c_str());
    return true;
}

void ODriveCanNode::recv_callback(const can_frame& frame) {

    if(((frame.can_id >> 5) & 0x3F) != node_id_) return;

    switch(frame.can_id & 0x1F) {
        case CmdId::kHeartbeat: {
            if (!verify_length("kHeartbeat", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.active_errors    = read_le<uint32_t>(frame.data + 0);
            ctrl_stat_.axis_state        = read_le<uint8_t>(frame.data + 4);
            ctrl_stat_.procedure_result  = read_le<uint8_t>(frame.data + 5);
            ctrl_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
            ctrl_pub_flag_ |= 0b0001;
            fresh_heartbeat_.notify_one();
            break;
        }
        case CmdId::kGetError: {
            if (!verify_length("kGetError", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
            odrv_pub_flag_ |= 0b001;
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            if (!verify_length("kGetEncoderEstimates", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
            ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0010;
            break;
        }
        case CmdId::kGetIq: {
            if (!verify_length("kGetIq", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b0100;
            break;
        }
        case CmdId::kGetTemp: {
            if (!verify_length("kGetTemp", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.fet_temperature   = read_le<float>(frame.data + 0);
            odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b010;
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            if (!verify_length("kGetBusVoltageCurrent", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
            odrv_stat_.bus_current = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b100;
            break;
        }
        case CmdId::kGetTorques: {
            if (!verify_length("kGetTorques", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.torque_target   = read_le<float>(frame.data + 0);
            ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b1000; 
            break;
        }
        case CmdId::kMITControl: {
            // MIT Control Feedback (SteadyWin GIM6010-8 Motor)
            // Reference: SteadyWin GIM6010-8 Motor Manual rev2.2 - CAN MIT Protocol
            // DBC: BO_ 8 Axis0_Mit_Feedback: 6 ODrive_Axis0
            //
            // MIT feedback format (6 bytes, big-endian/Motorola):
            // Fb_Node_ID:  Byte 0 [7:0]           - 8 bits
            // Fb_Position: Byte 1-2 [15:0]        - 16 bits, scale 0.000381, offset -12.5
            // Fb_Velocity: Byte 3[7:0], 4[7:4]    - 12 bits, scale 0.03175, offset -65
            // Fb_Torque:   Byte 4[3:0], 5[7:0]    - 12 bits, scale 0.02442, offset -50
            if (!verify_length("kMITControl", 6, frame.can_dlc)) break;
            
            uint8_t motor_id = frame.data[0];
            uint16_t p_int = ((uint16_t)frame.data[1] << 8) | frame.data[2];
            uint16_t v_int = ((uint16_t)frame.data[3] << 4) | ((frame.data[4] >> 4) & 0x0F);
            uint16_t t_int = (((uint16_t)frame.data[4] & 0x0F) << 8) | frame.data[5];
            
            // Apply DBC scaling: physical_value = raw_value * scale + offset
            float pos = (float)p_int * MIT_FB_POS_SCALE + MIT_FB_POS_OFFSET;
            float vel = (float)v_int * MIT_FB_VEL_SCALE + MIT_FB_VEL_OFFSET;
            float torque = (float)t_int * MIT_FB_TRQ_SCALE + MIT_FB_TRQ_OFFSET;
            
            RCLCPP_INFO(rclcpp::Node::get_logger(), 
                "MIT feedback: motor_id=%d pos=%.3f rad vel=%.3f rad/s torque=%.3f Nm",
                motor_id, pos, vel, torque);
            
            {
                std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
                ctrl_stat_.pos_estimate = pos;
                ctrl_stat_.vel_estimate = vel;
                ctrl_stat_.torque_estimate = torque;
            }
            // Publish immediately for MIT mode (don't wait for other CAN messages)
            ctrl_publisher_->publish(ctrl_stat_);
            break;
        }
        case CmdId::kSetAxisState:
        case CmdId::kSetControllerMode:
        case CmdId::kSetInputPos:
        case CmdId::kSetInputVel:
        case CmdId::kSetInputTorque:
        case CmdId::kClearErrors: {
            break; // Ignore commands coming from another master/host on the bus
        }
        default: {
            RCLCPP_WARN(rclcpp::Node::get_logger(), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
            break;
        }
    }
    
    if (ctrl_pub_flag_ == 0b1111) {
        ctrl_publisher_->publish(ctrl_stat_);
        ctrl_pub_flag_ = 0;
    }
    
    if (odrv_pub_flag_ == 0b111) {
        odrv_publisher_->publish(odrv_stat_);
        odrv_pub_flag_ = 0;
    }
}

void ODriveCanNode::subscriber_callback(const ControlMessage::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::Node::get_logger(), 
        "Received control_message: mode=%u pos=%.3f vel=%.3f torque=%.3f kp=%.3f kd=%.3f",
        msg->control_mode, msg->input_pos, msg->input_vel, msg->input_torque, 
        msg->input_kp, msg->input_kd);
    std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
    ctrl_msg_ = *msg;
    sub_evt_.set();
}

void ODriveCanNode::service_callback(const std::shared_ptr<AxisState::Request> request, std::shared_ptr<AxisState::Response> response) {
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state_ = request->axis_requested_state;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting axis state: %d", axis_state_);
    }
    srv_evt_.set();

    // Wait for at least 1 second for a new heartbeat to arrive.
    // If the requested state is something other than CLOSED_LOOP_CONTROL, also
    // wait for the procedure to complete (procedure_result != BUSY).
    std::unique_lock<std::mutex> guard(ctrl_stat_mutex_); // define lock for controller status
    auto call_time = std::chrono::steady_clock::now();
    fresh_heartbeat_.wait(guard, [this, &call_time, &request]() {
        bool is_busy = this->ctrl_stat_.procedure_result == ODriveProcedureResult::PROCEDURE_RESULT_BUSY;
        bool requested_closed_loop = request->axis_requested_state == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
        bool minimum_time_passed = (std::chrono::steady_clock::now() - call_time >= std::chrono::seconds(1));
        bool complete = (requested_closed_loop || !is_busy) && minimum_time_passed;
        return complete;
        }); // wait for procedure_result
    
    response->axis_state = ctrl_stat_.axis_state;
    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}

void ODriveCanNode::service_clear_errors_callback(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response) {
    RCLCPP_INFO(rclcpp::Node::get_logger(), "clearing errors");
    srv_clear_errors_evt_.set();
}

void ODriveCanNode::service_set_limits_callback(const std::shared_ptr<SetLimits::Request> request, std::shared_ptr<SetLimits::Response> response) {
    {
        std::lock_guard<std::mutex> guard(limits_mutex_);
        velocity_limit_ = request->velocity_limit;
        current_limit_ = request->current_limit;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "Setting limits: velocity=%.2f rev/s, current=%.2f A", 
            velocity_limit_, current_limit_);
    }
    srv_set_limits_evt_.set();
    response->success = true;
}

void ODriveCanNode::request_state_callback() {
    uint32_t axis_state;
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state = axis_state_;
    }

    struct can_frame frame;

    if (axis_state != 0) {
        // Clear errors if requested state is not IDLE
        frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
        write_le<uint8_t>(0, frame.data);
        frame.can_dlc = 1;
        can_intf_.send_can_frame(frame);
    }

    // Set state
    frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
    write_le<uint32_t>(axis_state, frame.data);
    frame.can_dlc = 4;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::request_clear_errors_callback() {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
    write_le<uint8_t>(0, frame.data);
    frame.can_dlc = 1;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::request_set_limits_callback() {
    float vel_limit, cur_limit;
    {
        std::lock_guard<std::mutex> guard(limits_mutex_);
        vel_limit = velocity_limit_;
        cur_limit = current_limit_;
    }
    
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kSetLimits;
    write_le<float>(vel_limit, frame.data);        // Velocity limit [rev/s]
    write_le<float>(cur_limit, frame.data + 4);    // Current limit [A]
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
    
    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "Sent limits CAN frame: vel=%.2f, cur=%.2f", vel_limit, cur_limit);
}

void ODriveCanNode::ctrl_msg_callback() {

    uint32_t control_mode;
    struct can_frame frame;
    
    {
        std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
        control_mode = ctrl_msg_.control_mode;
    }
    
    // MIT control uses its own protocol - skip kSetControllerMode for MIT
    if (control_mode != ControlMode::kMITControlMode) {
        frame.can_id = node_id_ << 5 | kSetControllerMode;
        {
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<uint32_t>(ctrl_msg_.control_mode, frame.data);
            write_le<uint32_t>(ctrl_msg_.input_mode,   frame.data + 4);
        }
        frame.can_dlc = 8;
        can_intf_.send_can_frame(frame);
        frame = can_frame{};
    }
    
    switch (control_mode) {
        case ControlMode::kVoltageControl: {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "Voltage Control Mode (0) is not currently supported");
            return;
        }
        case ControlMode::kTorqueControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_torque");
            frame.can_id = node_id_ << 5 | kSetInputTorque;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_torque, frame.data);
            frame.can_dlc = 4;
            break;
        }
        case ControlMode::kVelocityControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_vel");
            frame.can_id = node_id_ << 5 | kSetInputVel;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_vel,       frame.data);
            write_le<float>(ctrl_msg_.input_torque, frame.data + 4);
            frame.can_dlc = 8;
            break;
        }
        case ControlMode::kPositionControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_pos");
            frame.can_id = node_id_ << 5 | kSetInputPos;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_pos,  frame.data);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_vel) * 1000)),    frame.data + 4);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_torque) * 1000)), frame.data + 6);
            frame.can_dlc = 8;
            break;
        }
        case ControlMode::kMITControlMode: {
            // MIT Control Mode (SteadyWin GIM6010-8 Motor)
            // Reference: SteadyWin GIM6010-8 Motor Manual rev2.2 - CAN MIT Protocol
            // 
            // MIT protocol packs position, velocity, Kp, Kd, and torque into 8 bytes:
            // Byte 0: position[15:8]
            // Byte 1: position[7:0]
            // Byte 2: velocity[11:4]
            // Byte 3: velocity[3:0] | kp[11:8]
            // Byte 4: kp[7:0]
            // Byte 5: kd[11:4]
            // Byte 6: kd[3:0] | torque[11:8]
            // Byte 7: torque[7:0]
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "MIT control mode");
            
            // MIT Control CAN ID: (node_id << 5) | kMITControl
            frame.can_id = node_id_ << 5 | kMITControl;
            
            uint16_t p_int, v_int, kp_int, kd_int, t_int;
            {
                std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
                // Convert float values to unsigned integers with proper scaling
                p_int = float_to_uint(ctrl_msg_.input_pos, MIT_P_MIN, MIT_P_MAX, 16);
                v_int = float_to_uint(ctrl_msg_.input_vel, MIT_V_MIN, MIT_V_MAX, 12);
                kp_int = float_to_uint(ctrl_msg_.input_kp, MIT_KP_MIN, MIT_KP_MAX, 12);
                kd_int = float_to_uint(ctrl_msg_.input_kd, MIT_KD_MIN, MIT_KD_MAX, 12);
                t_int = float_to_uint(ctrl_msg_.input_torque, MIT_T_MIN, MIT_T_MAX, 12);
            }
            
            // Pack data into CAN frame (big-endian format for MIT protocol)
            frame.data[0] = (p_int >> 8) & 0xFF;           // position[15:8]
            frame.data[1] = p_int & 0xFF;                   // position[7:0]
            frame.data[2] = (v_int >> 4) & 0xFF;           // velocity[11:4]
            frame.data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);  // velocity[3:0] | kp[11:8]
            frame.data[4] = kp_int & 0xFF;                  // kp[7:0]
            frame.data[5] = (kd_int >> 4) & 0xFF;          // kd[11:4]
            frame.data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F);  // kd[3:0] | torque[11:8]
            frame.data[7] = t_int & 0xFF;                   // torque[7:0]
            
            frame.can_dlc = 8;
            
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), 
                "MIT cmd: pos=%.3f vel=%.3f kp=%.3f kd=%.3f torque=%.3f",
                uint_to_float(p_int, MIT_P_MIN, MIT_P_MAX, 16),
                uint_to_float(v_int, MIT_V_MIN, MIT_V_MAX, 12),
                uint_to_float(kp_int, MIT_KP_MIN, MIT_KP_MAX, 12),
                uint_to_float(kd_int, MIT_KD_MIN, MIT_KD_MAX, 12),
                uint_to_float(t_int, MIT_T_MIN, MIT_T_MAX, 12));
            break;
        }
        default: 
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported control_mode: %d", control_mode);
            return;
    }

    can_intf_.send_can_frame(frame);
}

inline bool ODriveCanNode::verify_length(const std::string&name, uint8_t expected, uint8_t length) {
    bool valid = expected == length;
    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
    if (!valid) RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
}
