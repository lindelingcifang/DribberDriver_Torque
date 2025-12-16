#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include "Task/utils.hpp"
#include <cstdint>

class Motor {
public:
    enum Type {
        kTypeDM3519 = 0,
        kTypeM2006 = 1
    };

    enum Direction {
        kDirFwd = 1,
        kDirRev = -1
    };

    struct Parameter_t {
        uint8_t id; // 1~4
        Direction direction;
        float ex_reduce_rate; // 额外减速比
        bool remove_built_in_reducer;
        bool need_limit_vel;
        float max_vel; // rpm
        float limit_vel_curr_proportion = 0.9;
    };

    struct Info_t {
        uint32_t rx_id;
        uint32_t tx_id;
        float raw_input_limit;
        float motor_current_limit;
        float motor_torque_limit;
        float broad_current_limit;
        float raw_reduce_rate;
    };

    struct Feedback_t {
        uint32_t motor_id;
        uint8_t data[8];
        uint8_t len;
    };

    enum DM3519_State : uint8_t
    {
        kStateCodeMotorDisable = 0x0,        // 电机失能
        kStateCodeMotorEnable = 0x1,         // 电机使能
        kStateCodeMotorSensorError = 0x5,    // 读取传感器错误
        kStateCodeMotorParameterError = 0x6, // 读取电机参数错误
        kStateCodeOverVolt = 0x8,            // 过压
        kStateCodeUnderVolt = 0x9,           // 欠压
        kStateCodeOverCurr = 0xA,            // 过流
        kStateCodeMosOverTemp = 0xB,         // MOS过温
        kStateCodeCoilOverTemp = 0xC,        // 电机线圈过温
        kStateCodeCommLoss = 0xD,            // 通信丢失
        kStateCodeOverload = 0xE,            // 过载
    };

    enum DM3519_Cmd
    {
        kCmdNormal = 0x0,   // 正常指令
        kCmdClearErr = 0x1, // 清除错误
    };

    Motor(const Type type, const Parameter_t& parameter);
    ~Motor() {};
    void set_current_input(float current);
    void set_torque_input(float torque);
    void encode(uint8_t* tx_data);
    void decode(const uint8_t rx_data[8]);

    float get_angle() const { return angle_; }
    float get_velocity() const { return vel_; }
    float get_current() const { return current_; }
    float get_torque() const { return torque_; }
    uint32_t get_tx_id() const { return info_.tx_id; }
    uint32_t get_rx_id() const { return info_.rx_id; }
    DM3519_State get_state() const { return state_; }

protected:
    float angle_ = 0.0f;   // degrees
    float vel_ = 0.0f;     // rpm
    float current_ = 0.0f; // Amperes
    float torque_ = 0.0f;  // Nm

    int16_t cmd_raw_input_ = 0;
    int16_t actual_raw_input_ = 0;
    float raw_angle_ = 0;
    float raw_vel_ = 0;
    float raw_current_ = 0;
    float last_raw_angle_ = 0;
    float reduce_rate_ = 1;
    DM3519_State state_ = kStateCodeMotorEnable;
    DM3519_Cmd cmd_ = kCmdNormal;

    Info_t info_;
    Parameter_t parameter_;

private:
    int16_t limit_vel();
};

const Motor::Info_t motor_info_DM3519{
    .rx_id = 0x200,
    .tx_id = 0x200,
    .raw_input_limit = 16384,
    .motor_current_limit = 20.5,
    .motor_torque_limit = 7.8,
    .broad_current_limit = 6,
    .raw_reduce_rate = 3591.0 / 187.0,
};

const Motor::Info_t motor_info_M2006{
    .rx_id = 0x200,
    .tx_id = 0x1FF,
    .raw_input_limit = 10000,
    .motor_current_limit = 10,
    .motor_torque_limit = 1,
    .broad_current_limit = 1,
    .raw_reduce_rate = 36,
};

#endif // __MOTOR_HPP
    

