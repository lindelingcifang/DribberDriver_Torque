#ifndef __CHASSIS_CONTROLLER_HPP
#define __CHASSIS_CONTROLLER_HPP

#include "component.hpp"
#include <array>
#include <cstddef>

class MixedLesoChassisController {
public:
    MixedLesoChassisController();

    InputPort<float>* vel_x_feedback_input_port() { return &vel_x_feedback_input_port_; }
    InputPort<float>* vel_y_feedback_input_port() { return &vel_y_feedback_input_port_; }
    InputPort<float>* yaw_feedback_input_port() { return &yaw_feedback_input_port_; }
    InputPort<float>* omega_z_feedback_input_port() { return &omega_z_feedback_input_port_; }
    InputPort<float>* dist_x_feedback_input_port() { return &dist_x_feedback_input_port_; }
    InputPort<float>* dist_y_feedback_input_port() { return &dist_y_feedback_input_port_; }
    InputPort<float>* dist_yaw_feedback_input_port() { return &dist_yaw_feedback_input_port_; }

    OutputPort<float>* wheel_torque_ff_output_port(std::size_t index) {
        return (index < wheel_torque_ff_output_ports_.size()) ? &wheel_torque_ff_output_ports_[index] : nullptr;
    }

    void set_reference(const float vel_ref[3], const float acc_ref[3]);
    void step(float dt_s);
    void reset();

private:
    bool inverse3x3(const float in[3][3], float out[3][3]) const;
    void precompute_mappings();

    InputPort<float> vel_x_feedback_input_port_;
    InputPort<float> vel_y_feedback_input_port_;
    InputPort<float> yaw_feedback_input_port_;
    InputPort<float> omega_z_feedback_input_port_;
    InputPort<float> dist_x_feedback_input_port_;
    InputPort<float> dist_y_feedback_input_port_;
    InputPort<float> dist_yaw_feedback_input_port_;

    std::array<OutputPort<float>, 4> wheel_torque_ff_output_ports_ = {
        OutputPort<float>(0.0f),
        OutputPort<float>(0.0f),
        OutputPort<float>(0.0f),
        OutputPort<float>(0.0f),
    };

    float vel_ref_[3] = {0.0f, 0.0f, 0.0f};
    float acc_ref_[3] = {0.0f, 0.0f, 0.0f};

    float last_vel_x_feedback_ = 0.0f;
    float last_vel_y_feedback_ = 0.0f;
    float last_yaw_feedback_ = 0.0f;
    float last_omega_z_feedback_ = 0.0f;
    float last_dist_x_feedback_ = 0.0f;
    float last_dist_y_feedback_ = 0.0f;
    float last_dist_yaw_feedback_ = 0.0f;

    float j1_pinv_[4][3] = {{0.0f}};
};

#endif // __CHASSIS_CONTROLLER_HPP
