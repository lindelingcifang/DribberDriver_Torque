#include "robot.hpp"
#include <cstring>
#include <cmath>

// Constants
static constexpr float DT = 1.0f / 1000.0f;

// Wheel geometry
static constexpr float WHEEL_ANGLE_FORWARD = 65.0f / 180.0f * PI;
static constexpr float WHEEL_ANGLE_BACKWARD = 37.0f / 180.0f * PI;

// Pre-computed trigonometric values
static constexpr float SIN_WHEEL_ANGLE_FWD = 0.9063f;   // sinf(WHEEL_ANGLE_FORWARD)
static constexpr float COS_WHEEL_ANGLE_FWD = 0.4226f;   // cosf(WHEEL_ANGLE_FORWARD)
static constexpr float SIN_WHEEL_ANGLE_BWD = 0.6018f;   // sinf(WHEEL_ANGLE_BACKWARD)
static constexpr float COS_WHEEL_ANGLE_BWD = 0.7986f;   // cosf(WHEEL_ANGLE_BACKWARD)

// Robot dynamics parameters
static constexpr float ROBOT_MASS = 2.19692f;           // kg
static constexpr float ROBOT_INERTIA = 2.29587357e-6f; // kg*m^2
static constexpr float ROBOT_RADIUS = 0.073f;           // m (avg of 85mm and 61mm)
static constexpr float WHEEL_INERTIA = 8.38e-5f;        // kg*m^2
static constexpr float WHEEL_RADIUS = 0.0285f;          // m
static constexpr float WHEEL_PARALLEL_RESISTANCE = 0.016f;
static constexpr float VERTICAL_RESISTANCE = 0.0f;
static constexpr float ACC_THRESHOLD[3] = {2.0f, 2.0f, 20.0f};

// IK solve matrix (pseudoinverse of A)
static constexpr float IK_SOLVE_PINV_A[4][3] = {
    {0.3829f, 0.4094f, 0.3270f},
    {-0.3829f, 0.4094f, 0.3270f},
    {-0.2542f, -0.4094f, 0.1730f},
    {0.2542f, -0.4094f, 0.1730f}
};

// Wheel velocity angle matrices
static constexpr float WHEEL_VX_ANGLE[4] = {
    SIN_WHEEL_ANGLE_FWD, -SIN_WHEEL_ANGLE_FWD, 
    -SIN_WHEEL_ANGLE_BWD, SIN_WHEEL_ANGLE_BWD
};

static constexpr float WHEEL_VY_ANGLE[4] = {
    COS_WHEEL_ANGLE_FWD, COS_WHEEL_ANGLE_FWD, 
    -COS_WHEEL_ANGLE_BWD, -COS_WHEEL_ANGLE_BWD
};

// Wheel VW angle (computed at compile time)
static constexpr float WHEEL_VW_ANGLE[4] = {
    0.3270f, 0.3270f, 0.1730f, 0.1730f
};

// Motor parameters for wheel motors
static const Motor::Parameter_t WHEEL_MOTOR_PARAMS[4] = {
    {.id = 1, .direction = Motor::kDirFwd, .ex_reduce_rate = 1, 
     .remove_built_in_reducer = true, .need_limit_vel = true, 
     .max_vel = 500, .limit_vel_curr_proportion = 0.8f},
    {.id = 2, .direction = Motor::kDirFwd, .ex_reduce_rate = 1, 
     .remove_built_in_reducer = true, .need_limit_vel = true, 
     .max_vel = 500, .limit_vel_curr_proportion = 0.8f},
    {.id = 3, .direction = Motor::kDirFwd, .ex_reduce_rate = 1, 
     .remove_built_in_reducer = true, .need_limit_vel = true, 
     .max_vel = 500, .limit_vel_curr_proportion = 0.8f},
    {.id = 4, .direction = Motor::kDirFwd, .ex_reduce_rate = 1, 
     .remove_built_in_reducer = true, .need_limit_vel = true, 
     .max_vel = 500, .limit_vel_curr_proportion = 0.8f}
};

static const Motor::Parameter_t DRIBBLER_MOTOR_PARAMS = {
    .id = 5, .direction = Motor::kDirRev, .ex_reduce_rate = 20, 
    .remove_built_in_reducer = false
};

// PID parameters for wheel motors (position control)
static const PID::Parameter_t WHEEL_PID_PARAMS[4] = {
    {.kp = 0, .ki = 0, .kd = 0, 
     .output_limit = motor_info_DM3519.broad_current_limit,
     .integ_limit = motor_info_DM3519.broad_current_limit, .dt = DT},
    {.kp = 0, .ki = 0, .kd = 0, 
     .output_limit = motor_info_DM3519.broad_current_limit,
     .integ_limit = motor_info_DM3519.broad_current_limit, .dt = DT},
    {.kp = 0, .ki = 0, .kd = 0, 
     .output_limit = motor_info_DM3519.broad_current_limit,
     .integ_limit = motor_info_DM3519.broad_current_limit, .dt = DT},
    {.kp = 0, .ki = 0, .kd = 0, 
     .output_limit = motor_info_DM3519.broad_current_limit,
     .integ_limit = motor_info_DM3519.broad_current_limit, .dt = DT}
};

// PID parameters for wheel velocity control
static const PID::Parameter_t WHEEL_VEL_PID_PARAMS[4] = {
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = DT},
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = DT},
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = DT},
    {.kp = 0, .ki = 0, .kd = 0, .output_limit = 4, .integ_limit = 4, .dt = DT}
};

// PID parameters for dribbler
static const PID::Parameter_t DRIBBLER_PID_PARAMS = {
    .kp = 0.001f, .ki = 0.005f, .kd = 0,
    .output_limit = motor_info_M2006.broad_current_limit,
    .integ_limit = motor_info_M2006.broad_current_limit, .dt = DT
};

// TD (tracking differentiator) parameters for dribbler
static const TD::Parameter_t DRIBBLER_TD_PARAMS = {
    .r = 2000, .h = 0.01f, .dt = DT,
    .is_cycle = false, .cycle_low = -180.0f, .cycle_high = 180.0f
};

// TD parameters for wheel motors
static const TD::Parameter_t WHEEL_TD_PARAMS[4] = {
    {.r = 200000, .h = 0.01f, .dt = DT, .is_cycle = false, 
     .cycle_low = -180.0f, .cycle_high = 180.0f},
    {.r = 200000, .h = 0.01f, .dt = DT, .is_cycle = false, 
     .cycle_low = -180.0f, .cycle_high = 180.0f},
    {.r = 200000, .h = 0.01f, .dt = DT, .is_cycle = false, 
     .cycle_low = -180.0f, .cycle_high = 180.0f},
    {.r = 200000, .h = 0.01f, .dt = DT, .is_cycle = false, 
     .cycle_low = -180.0f, .cycle_high = 180.0f}
};

// IMU data buffer for transmission
static float imu_data_buffer[9] = {0};

Robot::Robot() {
    // Initialize wheel motors
    for (int i = 0; i < 4; i++) {
        wheel_motor[i] = new Motor(Motor::kTypeDM3519, WHEEL_MOTOR_PARAMS[i]);
        wheel_PID_controllers[i] = new PID(WHEEL_PID_PARAMS[i]);
        wheel_vel_PID_controllers[i] = new PID(WHEEL_VEL_PID_PARAMS[i]);
        wheel_filter[i] = new TD(WHEEL_TD_PARAMS[i], 0);
    }

    // Initialize dribbler
    dribbler = new Motor(Motor::kTypeM2006, DRIBBLER_MOTOR_PARAMS);
    dribbler_PID_controller = new PID(DRIBBLER_PID_PARAMS);
    dribbler_filter = new TD(DRIBBLER_TD_PARAMS, 0);
}

Robot::~Robot() {
    for (int i = 0; i < 4; i++) {
        delete wheel_motor[i];
        delete wheel_PID_controllers[i];
        delete wheel_vel_PID_controllers[i];
        delete wheel_filter[i];
    }
    delete dribbler;
    delete dribbler_PID_controller;
    delete dribbler_filter;
}

void Robot::pi_decode_spi() {
    memcpy(&SpiRx, spi_rx_data, sizeof(SpiRx));

    // Decode robot velocity commands from Pi (if implemented)
    // for (uint8_t i = 0; i < 2; i++) {
    //     robot_vel[i] = SpiRx.vel[i] / 1000.0f;
    // }
    // robot_vel[2] = SpiRx.vel[2] / 100.0f;
}

void Robot::pi_encode_spi() {
    SpiTx.infrare_flag = (infra_ADC1_val > 0.5f) ? 1 : 0;
    SpiTx.getBall = false;
    SpiTx.imu_online = true;
    SpiTx.battery_vol = static_cast<int16_t>(bat_ADC2_val * 5);
    SpiTx.cap_vol = static_cast<int16_t>(cap_ADC3_val * 100);

    // Encode wheel motor velocities
    for (uint8_t i = 0; i < 4; i++) {
        SpiTx.wheel[i] = static_cast<int16_t>(wheel_motor[i]->get_velocity() * 10);
        SpiTx.wheel_ref[i] = static_cast<int16_t>(motor_vel[i] * 10);
    }

    // Encode IMU data
    for (uint8_t i = 0; i < 9; i++) {
        SpiTx.imu_data[i] = static_cast<int16_t>(imu_data_buffer[i] * 100);
    }

    memcpy(spi_tx_data, &SpiTx, sizeof(SpiTx));
}

void Robot::ik_solve() {
    float force[3] = {0};

    // Calculate forces from accelerations
    force[0] = robot_acc[0] * ROBOT_MASS + 
              2 * VERTICAL_RESISTANCE * (COS_WHEEL_ANGLE_FWD + COS_WHEEL_ANGLE_BWD);
    force[1] = robot_acc[1] * ROBOT_MASS + 
              2 * VERTICAL_RESISTANCE * (SIN_WHEEL_ANGLE_FWD + SIN_WHEEL_ANGLE_BWD);
    force[2] = robot_acc[2] * ROBOT_INERTIA / ROBOT_RADIUS;

    // Solve for motor forces using pseudoinverse
    for (uint8_t i = 0; i < 4; i++) {
        motor_F[i] = 0;
        for (uint8_t j = 0; j < 3; j++) {
            motor_F[i] += IK_SOLVE_PINV_A[i][j] * force[j];
        }
    }

    // Calculate motor accelerations and velocities
    for (uint8_t i = 0; i < 4; i++) {
        motor_acc[i] = (robot_acc[0] * WHEEL_VX_ANGLE[i] + 
                       robot_acc[1] * WHEEL_VY_ANGLE[i] + 
                       robot_acc[2] * WHEEL_VW_ANGLE[i] * ROBOT_RADIUS) / WHEEL_RADIUS +
                      wheel_vel_PID_controllers[i]->calc(motor_vel[i], wheel_motor[i]->get_velocity());

        motor_vel[i] = ((robot_real_vel[0] * WHEEL_VX_ANGLE[i] + 
                        robot_real_vel[1] * WHEEL_VY_ANGLE[i] + 
                        robot_real_vel[2] * WHEEL_VW_ANGLE[i] * ROBOT_RADIUS) / WHEEL_RADIUS) 
                       * 30.0f / PI;
    }

    // Calculate motor torques with resistance compensation
    for (int i = 0; i < 4; i++) {
        float resistance = 0;
        if (motor_vel[i] > 0.001f) {
            resistance = WHEEL_PARALLEL_RESISTANCE;
        } else if (motor_vel[i] < -0.001f) {
            resistance = -WHEEL_PARALLEL_RESISTANCE;
        }

        motor_torq[i] = WHEEL_INERTIA * motor_acc[i] + 
                       motor_F[i] * WHEEL_RADIUS + resistance;
    }
}

void Robot::motion_planner(const double _dt) {
    double dt_s = _dt / 1000000.0;  // Convert microseconds to seconds

    for (uint8_t i = 0; i < 3; i++) {
        // Calculate acceleration
        robot_acc[i] = (robot_vel[i] - last_robot_real_vel[i]) / dt_s;

        // Limit acceleration
        if (robot_acc[i] > ACC_THRESHOLD[i]) {
            robot_acc[i] = ACC_THRESHOLD[i];
        } else if (robot_acc[i] < -ACC_THRESHOLD[i]) {
            robot_acc[i] = -ACC_THRESHOLD[i];
        }

        // Update velocity
        robot_real_vel[i] = last_robot_real_vel[i] + robot_acc[i] * dt_s;
        last_robot_real_vel[i] = robot_real_vel[i];
    }
}