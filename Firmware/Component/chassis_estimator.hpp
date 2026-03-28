#ifndef __CHASSIS_ESTIMATOR_HPP
#define __CHASSIS_ESTIMATOR_HPP

#include "component.hpp"
#include <array>
#include <cstddef>

class ChassisEstimator {
public:
    ChassisEstimator();

    InputPort<float>* wheel_velocity_input_port(std::size_t index) {
        return (index < wheel_velocity_input_ports_.size()) ? &wheel_velocity_input_ports_[index] : nullptr;
    }

    InputPort<float>* imu_yaw_input_port() { return &imu_yaw_input_port_; }
    InputPort<float>* imu_omega_z_input_port() { return &imu_omega_z_input_port_; }

    OutputPort<float>* vel_x_output_port() { return &vel_x_output_port_; }
    OutputPort<float>* vel_y_output_port() { return &vel_y_output_port_; }
    OutputPort<float>* yaw_output_port() { return &yaw_output_port_; }
    OutputPort<float>* omega_z_output_port() { return &omega_z_output_port_; }
    OutputPort<float>* dist_x_output_port() { return &dist_x_output_port_; }
    OutputPort<float>* dist_y_output_port() { return &dist_y_output_port_; }
    OutputPort<float>* dist_yaw_output_port() { return &dist_yaw_output_port_; }

    void set_reference_acc(const float acc_ref[3]);
    void step(float dt_s);
    void reset();

private:
    bool inverse3x3(const float in[3][3], float out[3][3]) const;
    void precompute_mappings();

    std::array<InputPort<float>, 4> wheel_velocity_input_ports_;
    InputPort<float> imu_yaw_input_port_;
    InputPort<float> imu_omega_z_input_port_;

    OutputPort<float> vel_x_output_port_{0.0f};
    OutputPort<float> vel_y_output_port_{0.0f};
    OutputPort<float> yaw_output_port_{0.0f};
    OutputPort<float> omega_z_output_port_{0.0f};
    OutputPort<float> dist_x_output_port_{0.0f};
    OutputPort<float> dist_y_output_port_{0.0f};
    OutputPort<float> dist_yaw_output_port_{0.0f};

    float acc_ref_[3] = {0.0f, 0.0f, 0.0f};

    // 2nd-order LESO states for vx/vy: [z1=velocity, z2=disturbance]
    float vel_obs_[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    // 3rd-order LESO states for yaw: [z1=yaw, z2=omega, z3=disturbance]
    float yaw_obs_[3] = {0.0f, 0.0f, 0.0f};

    float j2_pinv_[3][4] = {{0.0f}};

    float last_wheel_vel_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float last_yaw_rad_ = 0.0f;
    float last_omega_z_rad_s_ = 0.0f;
};

#endif // __CHASSIS_ESTIMATOR_HPP
