#include "wheel_motor.hpp"
#include <cstring>

namespace {

uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    const float span = x_max - x_min;
    if (span == 0.0f || bits <= 0 || bits > 16) {
        return 0;
    }

    const float clamped = (x < x_min) ? x_min : ((x > x_max) ? x_max : x);
    const uint32_t max_int = (1u << static_cast<uint32_t>(bits)) - 1u;
    const float scaled = (clamped - x_min) * static_cast<float>(max_int) / span;
    return static_cast<uint16_t>(scaled);
}

} // namespace

const WheelMotorBase::Info_t motor_info_DMH3510{
    .motor_current_limit = 3.2,
    .motor_torque_limit = 0.45,
    .broad_current_limit = 2,
    .raw_reduce_rate = 1,
};

WheelMotorBase::WheelMotorBase(Type type, const Config_t& config, const Info_t& info)
    : type_(type), info_(info), config_(config) {}

void WheelMotorBase::publish_feedback_ports() {
    angle_output_port_ = angle_;
    velocity_output_port_ = vel_;
    torque_output_port_ = torque_;
    current_output_port_ = current_;
}

void WheelMotorBase::reset_ports() {
    angle_output_port_.reset();
    velocity_output_port_.reset();
    torque_output_port_.reset();
    current_output_port_.reset();
}

MotorDMH3510::MotorDMH3510(const Config_t& config)
    : WheelMotorBase(kTypeDMH3510, config, motor_info_DMH3510) {}

void MotorDMH3510::pack_velocity_data(float velocity, uint8_t* tx_data) {
    writing_register_ = false;

    std::memcpy(tx_data, &velocity, sizeof(float));
}

void MotorDMH3510::pack_mit_data(float position, float velocity, float kp, float kd, float torque_ff, uint8_t* tx_data) {
    static constexpr float kMITKpMin = 0.0f;
    static constexpr float kMITKpMax = 500.0f;
    static constexpr float kMITKdMin = 0.0f;
    static constexpr float kMITKdMax = 5.0f;

    const uint16_t pos_tmp = float_to_uint(position, -parameter_.pmax, parameter_.pmax, 16);
    const uint16_t vel_tmp = float_to_uint(velocity, -parameter_.vmax, parameter_.vmax, 12);
    const uint16_t kp_tmp = float_to_uint(kp, kMITKpMin, kMITKpMax, 12);
    const uint16_t kd_tmp = float_to_uint(kd, kMITKdMin, kMITKdMax, 12);
    const uint16_t tor_tmp = float_to_uint(torque_ff, -parameter_.tmax, parameter_.tmax, 12);

    tx_data[0] = static_cast<uint8_t>((pos_tmp >> 8) & 0xFF);
    tx_data[1] = static_cast<uint8_t>(pos_tmp & 0xFF);
    tx_data[2] = static_cast<uint8_t>((vel_tmp >> 4) & 0xFF);
    tx_data[3] = static_cast<uint8_t>(((vel_tmp & 0x0F) << 4) | ((kp_tmp >> 8) & 0x0F));
    tx_data[4] = static_cast<uint8_t>(kp_tmp & 0xFF);
    tx_data[5] = static_cast<uint8_t>((kd_tmp >> 4) & 0xFF);
    tx_data[6] = static_cast<uint8_t>(((kd_tmp & 0x0F) << 4) | ((tor_tmp >> 8) & 0x0F));
    tx_data[7] = static_cast<uint8_t>(tor_tmp & 0xFF);

    writing_register_ = false;
}

float MotorDMH3510::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    const float span = x_max - x_min;
    if (span == 0.0f) {
        return x_min;
    }
    return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + x_min;
}

void MotorDMH3510::parse_feedback_data(const uint8_t rx_data[8]) {

    state_ = static_cast<State>((rx_data[0] & 0xF0) >> 4);

    const uint16_t pos_uint = (rx_data[1] << 8) | rx_data[2];
    const uint16_t vel_uint = (rx_data[3] << 4) | (rx_data[4] >> 4);
    const uint16_t tor_uint = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    float pos = uint_to_float(pos_uint, -parameter_.pmax, parameter_.pmax, 16);
    float vel = uint_to_float(vel_uint, -parameter_.vmax, parameter_.vmax, 12);
    torque_ = uint_to_float(tor_uint, -parameter_.tmax, parameter_.tmax, 12) * config_.direction;

    angle_ = config_.direction * pos * 180.0f / 3.1415926535f;
    vel_ = config_.direction * vel * 30.0f / 3.1415926535f;

    enabled_ = (state_ == kStateMotorEnable);
    publish_feedback_ports();
}

void MotorDMH3510::build_set_mode_msg(Mode mode, can_Message_t& msg) {
    if (mode < kModeMITControl || mode > kModeMixedControl) {
        return;
    }
    mode_ = static_cast<Mode>(mode);
    build_write_register_msg(0x0A, static_cast<uint32_t>(mode), msg);
}

void MotorDMH3510::build_enable_msg(can_Message_t& msg) {
    writing_register_ = false;

    msg.id = config_.control_id;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFC;
}

void MotorDMH3510::build_disable_msg(can_Message_t& msg) {
    writing_register_ = false;

    msg.id = config_.control_id;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFD;
}

void MotorDMH3510::build_clear_error_msg(can_Message_t& msg) {
    writing_register_ = false;

    msg.id = config_.control_id;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFB;
}

void MotorDMH3510::build_save_zero_msg(can_Message_t& msg) {
    writing_register_ = false;

    msg.id = config_.control_id;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFE;
}

void MotorDMH3510::build_set_acc_msg(float acc, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &acc, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegACC, data, msg);
}

void MotorDMH3510::build_set_dec_msg(float dec, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &dec, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegDEC, data, msg);
}

void MotorDMH3510::build_set_velocity_kp_msg(float kp, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &kp, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegKP_ASR, data, msg);
}

void MotorDMH3510::build_set_velocity_ki_msg(float ki, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &ki, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegKI_ASR, data, msg);
}

void MotorDMH3510::build_set_pmax_msg(float pmax, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &pmax, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegPMAX, data, msg);
}

void MotorDMH3510::build_set_vmax_msg(float vmax, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &vmax, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegVMAX, data, msg);
}

void MotorDMH3510::build_set_tmax_msg(float tmax, can_Message_t& msg) {
    uint32_t data = 0;
    std::memcpy(&data, &tmax, sizeof(float));
    const_cast<MotorDMH3510*>(this)->build_write_register_msg(kRegTMAX, data, msg);
}

void MotorDMH3510::build_write_register_msg(uint8_t rid, uint32_t data, can_Message_t& msg) {
    const uint16_t esc_id = static_cast<uint16_t>(config_.control_id);
    msg.id = 0x7FF;
    msg.isExt = false;
    msg.rtr = false;
    msg.len = 8;
    msg.buf[0] = static_cast<uint8_t>(esc_id & 0xFF);
    msg.buf[1] = static_cast<uint8_t>((esc_id >> 8) & 0xFF);
    msg.buf[2] = 0x55;
    msg.buf[3] = rid;
    msg.buf[4] = static_cast<uint8_t>(data & 0xFF);
    msg.buf[5] = static_cast<uint8_t>((data >> 8) & 0xFF);
    msg.buf[6] = static_cast<uint8_t>((data >> 16) & 0xFF);
    msg.buf[7] = static_cast<uint8_t>((data >> 24) & 0xFF);
    writing_register_ = true;
}

uint32_t MotorDMH3510::command_can_id() const {
    switch (mode_) {
        case kModeMITControl:
            return static_cast<uint32_t>(config_.control_id);
        case kModePositionVelocityControl:
            return static_cast<uint32_t>(config_.control_id) + 0x100;
        case kModeVelocityControl:
            return static_cast<uint32_t>(config_.control_id) + 0x200;
        case kModeMixedControl:
            return static_cast<uint32_t>(config_.control_id) + 0x300;
        default:
            break;
    }
    return static_cast<uint32_t>(config_.control_id);
}


