#include "chassis_controller.hpp"
#include "control_params.hpp"
#include <algorithm>
#include <cmath>

volatile float torque_ff_debug = 0;
volatile float f_task_x_debug = 0;
volatile float f_task_y_debug = 0;
volatile float f_task_yaw_debug = 0;

namespace {

constexpr float kPi = 3.1415926535f;

} // namespace

MixedLesoChassisController::MixedLesoChassisController() {
    precompute_mappings();
}

void MixedLesoChassisController::set_reference(const float vel_ref[3], const float acc_ref[3]) {
    for (int i = 0; i < 3; ++i) {
        vel_ref_[i] = vel_ref[i];
        acc_ref_[i] = acc_ref[i];
    }
}

void MixedLesoChassisController::step(float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    const std::optional<float> vel_x_feedback = vel_x_feedback_input_port_.any();
    const std::optional<float> vel_y_feedback = vel_y_feedback_input_port_.any();
    const std::optional<float> yaw_feedback = yaw_feedback_input_port_.any();
    const std::optional<float> omega_z_feedback = omega_z_feedback_input_port_.any();
    const std::optional<float> dist_x_feedback = dist_x_feedback_input_port_.any();
    const std::optional<float> dist_y_feedback = dist_y_feedback_input_port_.any();
    const std::optional<float> dist_yaw_feedback = dist_yaw_feedback_input_port_.any();

    last_vel_x_feedback_ = vel_x_feedback.has_value() ? *vel_x_feedback : last_vel_x_feedback_;
    last_vel_y_feedback_ = vel_y_feedback.has_value() ? *vel_y_feedback : last_vel_y_feedback_;
    last_yaw_feedback_ = yaw_feedback.has_value() ? *yaw_feedback : last_yaw_feedback_;
    last_omega_z_feedback_ = omega_z_feedback.has_value() ? *omega_z_feedback : last_omega_z_feedback_;
    last_dist_x_feedback_ = dist_x_feedback.has_value() ? *dist_x_feedback : last_dist_x_feedback_;
    last_dist_y_feedback_ = dist_y_feedback.has_value() ? *dist_y_feedback : last_dist_y_feedback_;
    last_dist_yaw_feedback_ = dist_yaw_feedback.has_value() ? *dist_yaw_feedback : last_dist_yaw_feedback_;

    (void)last_yaw_feedback_;

    const float f_task[3] = {
        control_config::kRobotMassKg *
            (acc_ref_[0] + control_config::kVelFeedbackGainX * (vel_ref_[0] - last_vel_x_feedback_) - last_dist_x_feedback_),
        control_config::kRobotMassKg *
            (acc_ref_[1] + control_config::kVelFeedbackGainY * (vel_ref_[1] - last_vel_y_feedback_) - last_dist_y_feedback_),
        control_config::kRobotInertiaKgM2 *
            (acc_ref_[2] + control_config::kVelFeedbackGainYaw * (vel_ref_[2] - last_omega_z_feedback_) -
             last_dist_yaw_feedback_),
    };

    f_task_x_debug = f_task[0];
    f_task_y_debug = f_task[1];
    f_task_yaw_debug = f_task[2];


    for (int wheel = 0; wheel < 4; ++wheel) {
        float tau = 0.0f;
        for (int row = 0; row < 3; ++row) {
            tau += j1_pinv_[wheel][row] * f_task[row];
        }
        tau = std::clamp(tau, -control_config::kWheelTorqueFfLimitNm, control_config::kWheelTorqueFfLimitNm);
        wheel_torque_ff_output_ports_[wheel] = tau;
    }

    torque_ff_debug = wheel_torque_ff_output_ports_[0].any().value_or(0.0f);
}

void MixedLesoChassisController::reset() {
    last_vel_x_feedback_ = 0.0f;
    last_vel_y_feedback_ = 0.0f;
    last_yaw_feedback_ = 0.0f;
    last_omega_z_feedback_ = 0.0f;
    last_dist_x_feedback_ = 0.0f;
    last_dist_y_feedback_ = 0.0f;
    last_dist_yaw_feedback_ = 0.0f;

    for (int i = 0; i < 4; ++i) {
        wheel_torque_ff_output_ports_[i] = 0.0f;
    }
}

bool MixedLesoChassisController::inverse3x3(const float in[3][3], float out[3][3]) const {
    const float a = in[0][0], b = in[0][1], c = in[0][2];
    const float d = in[1][0], e = in[1][1], f = in[1][2];
    const float g = in[2][0], h = in[2][1], i = in[2][2];

    const float A = (e * i - f * h);
    const float B = -(d * i - f * g);
    const float C = (d * h - e * g);
    const float D = -(b * i - c * h);
    const float E = (a * i - c * g);
    const float F = -(a * h - b * g);
    const float G = (b * f - c * e);
    const float H = -(a * f - c * d);
    const float I = (a * e - b * d);

    const float det = a * A + b * B + c * C;
    if (std::fabs(det) < 1e-8f) {
        return false;
    }

    const float inv_det = 1.0f / det;
    out[0][0] = A * inv_det;
    out[0][1] = D * inv_det;
    out[0][2] = G * inv_det;
    out[1][0] = B * inv_det;
    out[1][1] = E * inv_det;
    out[1][2] = H * inv_det;
    out[2][0] = C * inv_det;
    out[2][1] = F * inv_det;
    out[2][2] = I * inv_det;
    return true;
}

void MixedLesoChassisController::precompute_mappings() {
    const float r = control_config::kWheelRadiusM;
    const float l = control_config::kWheelCenterDistanceM;
    const float d = control_config::kCenterToComDistanceM;
    const float alpha = control_config::kWheelAlphaRad;
    const float beta = control_config::kWheelBetaRad;

    const float ca = std::cos(alpha);
    const float sa = std::sin(alpha);
    const float cb = std::cos(beta);
    const float sb = std::sin(beta);
    const float inv_r = (r > 1e-9f) ? (1.0f / r) : 0.0f;

    const float j1[3][4] = {
        {inv_r * ca, inv_r * (-ca), inv_r * (-cb), inv_r * cb},
        {inv_r * (-sa), inv_r * (-sa), inv_r * sb, inv_r * sb},
        {inv_r * (-l - d * sa), inv_r * (-l - d * sa), inv_r * (-l + d * sb), inv_r * (-l + d * sb)},
    };

    float j1_j1t[3][3] = {{0.0f}};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 4; ++k) {
                j1_j1t[row][col] += j1[row][k] * j1[col][k];
            }
        }
    }

    float j1_j1t_inv[3][3] = {{0.0f}};
    if (inverse3x3(j1_j1t, j1_j1t_inv)) {
        for (int wheel = 0; wheel < 4; ++wheel) {
            for (int row = 0; row < 3; ++row) {
                float acc = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    acc += j1[k][wheel] * j1_j1t_inv[k][row];
                }
                j1_pinv_[wheel][row] = acc;
            }
        }
    }

}
