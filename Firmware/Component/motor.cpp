#include "motor.hpp"

Motor::Motor(const Type type, const Parameter_t& parameter) : parameter_(parameter){
    // Initialize motor based on type
    switch (type) {
        case kTypeDM3519:
            // Set DM3519 specific parameters
            info_ = motor_info_DM3519;
            info_.rx_id = 0x200 + parameter_.id;
            break;
        case kTypeM2006:
            // Set M2006 specific parameters
            info_ = motor_info_M2006;
            info_.rx_id = 0x200 + parameter_.id;
            break;
        default:
            // Unknown type, set default parameters
            info_.rx_id = 0;
            info_.tx_id = 0;
            info_.raw_input_limit = 0.0f;
            info_.motor_current_limit = 0.0f;
            info_.motor_torque_limit = 0.0f;
            info_.broad_current_limit = 0.0f;
            info_.raw_reduce_rate = 1.0f;
            break;
    }
    if (parameter_.remove_built_in_reducer)
        {
            reduce_rate_ = ((float)parameter_.ex_reduce_rate);
        }
        else
        {
            reduce_rate_ = ((float)parameter_.ex_reduce_rate) * info_.raw_reduce_rate;
        }
}

void Motor::set_current_input(float curr_input_t)
{
    curr_input_t = limit<float>(curr_input_t, info_.broad_current_limit);

    cmd_raw_input_ = parameter_.direction * (int16_t)(curr_input_t / info_.motor_current_limit * info_.raw_input_limit);

    if (parameter_.need_limit_vel)
    {
        actual_raw_input_ = limit_vel();
    }
    else
    {
        actual_raw_input_ = cmd_raw_input_;
    }
}

void Motor::set_torque_input(float torq_input_t)
{
    // 移除减速箱相当于多了一个 （1 / 原减速比） 的减速箱，这里的力矩是相对于带有减速箱的，和decode那里不同
    if (parameter_.remove_built_in_reducer)
    {
        set_current_input(torq_input_t / (((float)parameter_.ex_reduce_rate )/ info_.raw_reduce_rate) / info_.motor_torque_limit * info_.motor_current_limit);
    }
    else
    {
        set_current_input(torq_input_t / ((float)parameter_.ex_reduce_rate ) / info_.motor_torque_limit * info_.motor_current_limit);
    }
}

int16_t Motor::limit_vel(void)
{
    if ((vel_ > parameter_.max_vel) || (vel_ < -parameter_.max_vel))
    {
        return actual_raw_input_ * parameter_.limit_vel_curr_proportion;
    }
    else
    {
        return cmd_raw_input_;
    }
}

void Motor::encode(uint8_t* tx_data)
{
    uint8_t data_H = 0x00;
    uint8_t data_L = 0x00;

    data_H = (uint8_t)((actual_raw_input_ >> 8) & 0xFF);
    data_L = (uint8_t)(actual_raw_input_ & 0xFF);

    if (parameter_.id <= 4)
    {
        tx_data[(parameter_.id - 1) * 2] = data_H;
        tx_data[(parameter_.id - 1) * 2 + 1] = data_L;
    }
    else if (parameter_.id <= 8)
    {
        tx_data[(parameter_.id - 5) * 2] = data_H;
        tx_data[(parameter_.id - 5) * 2 + 1] = data_L;
    }
}

void Motor::decode(const uint8_t rx_data[8])
{
    last_raw_angle_ = raw_angle_;

    raw_angle_ = 360.0 * ((((float)((((int16_t)rx_data[0]) << 8) | (int16_t)rx_data[1]))) / 8191);
    raw_vel_ = (int16_t)(((rx_data[2]) << 8) | rx_data[3]);
    raw_current_ = (int16_t)((rx_data[4] << 8) | rx_data[5]);

    float delta_raw_angle = raw_angle_ - last_raw_angle_;

    if (delta_raw_angle < -180.0)
    {
        delta_raw_angle = delta_raw_angle + 360.0;
    }

    if (delta_raw_angle > 180.0)
    {
        delta_raw_angle = delta_raw_angle - 360.0;
    }

    angle_ = static_cast<float>(parameter_.direction) * normalize<float>(static_cast<float>(parameter_.direction) * angle_ + delta_raw_angle / static_cast<float>(reduce_rate_), -180, 180);
    vel_ = static_cast<float>(parameter_.direction) * raw_vel_ / static_cast<float>(reduce_rate_);
    current_ = static_cast<float>(parameter_.direction) * raw_current_;
    torque_ = (float)(current_ / info_.motor_current_limit * info_.motor_torque_limit * ((float)reduce_rate_));
}

