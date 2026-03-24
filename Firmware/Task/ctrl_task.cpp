#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"

namespace {

enum WheelCommandMode : uint8_t {
    kWheelCommandVelocity = 0,
    kWheelCommandMIT = 1,
};

static constexpr WheelCommandMode kWheelCommandMode = kWheelCommandVelocity;

struct MITCommandConfig {
    float kp;
    float kd;
    float torque_ff;
};

static constexpr MITCommandConfig kMITRunConfig{0.0f, 0.5f, 0.0f};
static constexpr MITCommandConfig kMITSafeConfig{2.0f, 0.1f, 0.0f};
static constexpr float kRadPerRpm = 3.1415926535f / 30.0f;
static constexpr float kDegToRad = 3.1415926535f / 180.0f;

WheelMotorBase::Mode active_motor_mode() {
    return (kWheelCommandMode == kWheelCommandMIT)
               ? WheelMotorBase::kModeMITControl
               : WheelMotorBase::kModeVelocityControl;
}

bool build_wheel_command(Robot& robot, uint8_t index, bool safe_output, can_Message_t& out_msg) {
    if (index >= 4) {
        return false;
    }

    WheelMotorBase* motor = robot.wheel_motors[index];
    const WheelMotorBase::Mode required_mode = active_motor_mode();
    if (motor->get_mode() != required_mode) {
        motor->build_set_mode_msg(required_mode, out_msg);
        return true;
    }

    out_msg.id = motor->command_can_id() + 0x200;
    out_msg.isExt = false;
    out_msg.rtr = false;

    if (kWheelCommandMode == kWheelCommandMIT) {
        uint8_t tx_data[8] = {0};
        const float velocity_ref = safe_output ? 0.0f : (robot.motor_vel[index] * kRadPerRpm);
        const float position_ref = motor->get_angle() * kDegToRad;
        const MITCommandConfig cfg = safe_output ? kMITSafeConfig : kMITRunConfig;
        motor->pack_mit_data(position_ref, velocity_ref, cfg.kp, cfg.kd, cfg.torque_ff, tx_data);
        out_msg.len = 8;
        memcpy(out_msg.buf, tx_data, out_msg.len);
        return true;
    }

    uint8_t tx_data[4] = {0};
    const float velocity_ref = safe_output ? 0.0f : (robot.motor_vel[index] * kRadPerRpm);
    motor->pack_velocity_data(velocity_ref, tx_data);
    out_msg.len = 4;
    memcpy(out_msg.buf, tx_data, out_msg.len);
    return true;
}

} // namespace

// Debug variables
float dribblerInput;
uint8_t dribbler_mode;
float debug_pose, debug_vel;
float debug_K, debug_D, debug_M, debug_angle_ref;
bool debug_enable;
volatile uint32_t wait_us_debug = 0;
volatile uint32_t exec_time_us = 0;

float wheelInput[4];
float debug_motor_vel[4];
float debug_motor_acc[4];

void motor_init() {
    volatile uint8_t i;
    for (i = 0; i < 4; ++i) {
        can_Message_t msg;
        robot.wheel_motors[i]->build_set_mode_msg(active_motor_mode(), msg);
        can2_bus.send_message(msg);
        osDelay(20);
        robot.wheel_motors[i]->build_set_acc_msg(MotorDMH3510::Parameter_t().acc, msg);
        can2_bus.send_message(msg);
        osDelay(20);
        robot.wheel_motors[i]->build_set_dec_msg(MotorDMH3510::Parameter_t().dec, msg);
        can2_bus.send_message(msg);
        osDelay(20);
        if (kWheelCommandMode == kWheelCommandVelocity) {
            robot.wheel_motors[i]->build_set_velocity_kp_msg(MotorDMH3510::Parameter_t().kp_asr, msg);
            can2_bus.send_message(msg);
            osDelay(20);
            robot.wheel_motors[i]->build_set_velocity_ki_msg(MotorDMH3510::Parameter_t().ki_asr, msg);
            can2_bus.send_message(msg);
            osDelay(20);
        }
        robot.wheel_motors[i]->build_enable_msg(msg);
        can2_bus.send_message(msg);
        osDelay(20);
        can_Message_t wheel_msg;
        if (build_wheel_command(robot, i, true, wheel_msg)) {
            can2_bus.send_message(wheel_msg);
        }
        osDelay(20);
    }
}

extern "C" {
    
// Control task implementation - 1kHz real-time loop
void StartCrtlTask(void *argument) {
    // Wait for system initialization
    osDelay(100);

    // Initialize motors (set control mode, PID gains, enable)
    motor_init();
    
    uint32_t ctrl_start_tick = 0;
    uint32_t ctrl_end_tick = 0;
    
    for(;;) {
        // Wait for TIM2 interrupt to trigger (1kHz)
        if (osSemaphoreAcquire(sem_ctrl_triggerHandle, osWaitForever) == osOK) {
            ctrl_start_tick = TIM2->CNT;
            
            // Acquire robot state mutex
            if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
                
                // Motion planning: compute acceleration from velocity setpoints
                robot.motion_planner(TIM2_PERIOD_CLOCKS);  // microseconds

                // Inverse kinematics: compute wheel torques
                robot.ik_solve();
                
                can_Message_t wheel_msgs[4];


                for (uint8_t i = 0; i < 4; i++) {
                    robot.wheel_filter[i]->calc(robot.wheel_motors[i]->get_velocity());
                    debug_motor_vel[i] = robot.wheel_filter[i]->get_data();
                    debug_motor_acc[i] = robot.wheel_filter[i]->get_diff() * 3.1415926f / 30.0f;
                    
                    const bool safe_output = !robot.wheel_motors[i]->is_enabled() || !robot.watchdog_check();
                    build_wheel_command(robot, i, safe_output, wheel_msgs[i]);
                    if (safe_output) {
                        robot.wheel_PID_controllers[i]->reset();
                    }

                    
                }
            
                if (osSemaphoreAcquire(sem_can_txHandle, 10) == osOK) {
                    for (int i = 0; i < 4; i++) {
                        wait_us_debug = 0;
                        while (!can2_bus.send_message(wheel_msgs[i])) {
                            // Microsecond-level busy wait using TIM2 (1 tick = 1 us)
                            // uint32_t wait_start = TIM2->CNT;
                            // while ((TIM2->CNT - wait_start) < 10) {
                            //     // Busy wait before retrying to prevent task starvation
                            // }
                            // wait_us_debug += 10;
                        }
                    }
                    osSemaphoreRelease(sem_can_txHandle);
                }

                
                // robot.dribbler_filter->calc(robot.dribbler->get_velocity());
                // debug_pose = robot.dribbler_filter->get_data();
                // debug_vel = robot.dribbler_filter->get_diff();
                
                // if (debug_enable) {
                //     dribblerInput = debug_K * (debug_angle_ref - robot.dribbler->get_angle()) + debug_D * debug_pose + debug_M * debug_vel;
                // } else {
                //     dribblerInput = 0;
                // }
                // robot.dribbler->set_torque_input(dribblerInput);
                // robot.dribbler->encode(can2_tx_data);
                // can2_tx_data[2] = dribbler_mode;
                
                // Release mutex
                osMutexRelease(mtx_robot_stateHandle);
            }
            
            ctrl_end_tick = TIM2->CNT;
            // Control loop execution time can be monitored here
            exec_time_us = (ctrl_end_tick >= ctrl_start_tick) ? (ctrl_end_tick - ctrl_start_tick) : (0xFFFFFFFF - ctrl_start_tick + ctrl_end_tick);
        }
    }
}

} // extern "C"
