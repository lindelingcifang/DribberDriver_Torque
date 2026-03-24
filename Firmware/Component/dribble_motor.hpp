#ifndef DRIBBLE_MOTOR_HPP
#define DRIBBLE_MOTOR_HPP

#include "Task/utils.hpp"
#include "Communication/can/can_helpers.hpp"
#include <cstdint>

class DibbleMotorBase {
public:
    enum Type {
        kTypeM2006 = 0,
    };

    struct Config_t {
        uint8_t control_id; // 1~255
        uint8_t feedback_id;
        float direction;
        float ex_reduce_rate;
        bool remove_built_in_reducer;
        float limit_vel_curr_proportion = 0.9f;
    };

    struct Info_t {
        float motor_current_limit;
        float motor_torque_limit;
        float broad_current_limit;
        float raw_reduce_rate;
    };

    virtual ~DibbleMotorBase() = default;

    virtual void parse_feedback_data(const uint8_t rx_data[8]) = 0;

    virtual uint32_t command_can_id() const = 0;
    virtual uint32_t feedback_can_id() const { return static_cast<uint32_t>(config_.feedback_id); }

    float get_angle() const { return angle_; }
    float get_velocity() const { return vel_; }
    float get_current() const { return current_; }
    float get_torque() const { return torque_; }
    Type get_type() const { return type_; }
    bool is_enabled() const { return enabled_; }

protected:
    DibbleMotorBase(Type type, const Config_t& config, const Info_t& info);

    float angle_ = 0.0f;
    float vel_ = 0.0f;
    float current_ = 0.0f;
    float torque_ = 0.0f;

    float raw_angle_ = 0.0f;
    float raw_vel_ = 0.0f;
    float raw_current_ = 0.0f;
    float last_raw_angle_ = 0.0f;

    Type type_;
    Info_t info_;
    Config_t config_;
    bool enabled_ = false;
};

class MotorM2006 : public DibbleMotorBase {
public:
    explicit MotorM2006(const Config_t& config);

    void parse_feedback_data(const uint8_t rx_data[8]) override;

    uint32_t command_can_id() const override;
};

extern const DibbleMotorBase::Info_t motor_info_M2006;

#endif // DRIBBLE_MOTOR_HPP