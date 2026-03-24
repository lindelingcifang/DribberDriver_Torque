#include "dribble_motor.hpp"
#include <cstring>

const DibbleMotorBase::Info_t motor_info_M2006{
    .motor_current_limit = 10,
    .motor_torque_limit = 1,
    .broad_current_limit = 1,
    .raw_reduce_rate = 36,
};

DibbleMotorBase::DibbleMotorBase(Type type, const Config_t& config, const Info_t& info)
    : type_(type), info_(info), config_(config) {}

MotorM2006::MotorM2006(const Config_t& config)
    : DibbleMotorBase(kTypeM2006, config, motor_info_M2006) {
        config_.ex_reduce_rate = config_.remove_built_in_reducer ? 1 / info_.raw_reduce_rate : 1.0f;
    }

void MotorM2006::parse_feedback_data(const uint8_t rx_data[8]) {
    last_raw_angle_ = raw_angle_;

    const uint16_t angle_raw = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
    const int16_t speed_rpm = static_cast<int16_t>((static_cast<uint16_t>(rx_data[2]) << 8) | rx_data[3]);
    const int16_t torque_raw = static_cast<int16_t>((static_cast<uint16_t>(rx_data[4]) << 8) | rx_data[5]);

    float angle_float = static_cast<float>(angle_raw) * 360.0f / 8192.0f;
    float vel_float = static_cast<float>(speed_rpm);
    float torque_float = static_cast<float>(torque_raw);

    angle_ = config_.direction * angle_float * config_.ex_reduce_rate;
    vel_ = config_.direction * vel_float * config_.ex_reduce_rate;
    torque_ = config_.direction * torque_float * config_.ex_reduce_rate;
    enabled_ = true;
}

uint32_t MotorM2006::command_can_id() const {
    return static_cast<uint32_t>(config_.control_id);
}