#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"

// Debug variables
float dribblerInput;
uint8_t dribbler_mode;
float debug_pose, debug_vel;
float debug_K, debug_D, debug_M, debug_angle_ref;
bool debug_enable;

float wheelInput[4];
float debug_motor_vel[4];
float debug_motor_acc[4];
bool wheel_enable;

extern "C" {
    
// Control task implementation - 1kHz real-time loop
void StartCrtlTask(void *argument) {
    // Wait for system initialization
    osDelay(100);
    
    uint32_t ctrl_start_tick = 0;
    uint32_t ctrl_end_tick = 0;
    
    for(;;) {
        // Wait for TIM2 interrupt to trigger (1kHz)
        if (osSemaphoreAcquire(sem_ctrl_triggerHandle, osWaitForever) == osOK) {
            ctrl_start_tick = TIM2->CNT;
            
            // Acquire robot state mutex
            if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
                
                // Motion planning: compute acceleration from velocity setpoints
                robot.motion_planner(1000);  // dt = 1000 microseconds (1ms)
                
                // Inverse kinematics: compute wheel torques
                robot.ik_solve();
                
                // PID control for each wheel
                PID::Parameter_t wheel_pid_parameter;
                PID::Parameter_t wheel_vel_pid_parameter;
                
                for (uint8_t i = 0; i < 4; i++) {
                    robot.wheel_filter[i]->calc(robot.wheel_motor[i]->get_velocity());
                    debug_motor_vel[i] = robot.wheel_filter[i]->get_data();
                    debug_motor_acc[i] = robot.wheel_filter[i]->get_diff() * 3.1415926f / 30.0f;
                    
                    if (!wheel_enable) {
                        robot.wheel_motor[i]->set_torque_input(0);
                        robot.wheel_PID_controllers[i]->reset();
                    } else {
                        if (robot.motor_acc[i] < 0.1f && robot.motor_acc[i] > -0.1f) {
                            robot.wheel_PID_controllers[i]->reset();
                        } else {
                            robot.wheel_vel_PID_controllers[i]->reset();
                        }
                        
                        wheel_vel_pid_parameter.kp = robot.wheel_vel_PID[0];
                        wheel_vel_pid_parameter.ki = robot.wheel_vel_PID[1];
                        wheel_vel_pid_parameter.kd = robot.wheel_vel_PID[2];
                        robot.wheel_vel_PID_controllers[i]->set_parameter(wheel_vel_pid_parameter);
                        
                        wheel_pid_parameter.kp = robot.wheel_PID[0];
                        wheel_pid_parameter.ki = robot.wheel_PID[1];
                        wheel_pid_parameter.kd = robot.wheel_PID[2];
                        robot.wheel_PID_controllers[i]->set_parameter(wheel_pid_parameter);
                        wheelInput[i] = robot.wheel_PID_controllers[i]->calc(robot.motor_acc[i], debug_motor_acc[i]) + robot.motor_torq[i];
                        robot.wheel_motor[i]->set_torque_input(wheelInput[i]);
                    }
                }
                
                // Encode and send motor commands via CAN2
                uint8_t can2_tx_data[8];
                for (uint8_t i = 0; i < 4; i++) {
                    robot.wheel_motor[i]->encode(can2_tx_data);
                }
                
                // Use ZCAN to send motor commands
                if (osSemaphoreAcquire(sem_can_txHandle, 5) == osOK) {
                    can_Message_t tx_msg;
                    tx_msg.id = robot.wheel_motor[3]->get_tx_id();
                    tx_msg.isExt = false;
                    tx_msg.rtr = false;
                    tx_msg.len = 8;
                    memcpy(tx_msg.buf, can2_tx_data, 8);
                    can2_bus.send_message(tx_msg);
                    osSemaphoreRelease(sem_can_txHandle);
                }
                
                robot.dribbler_filter->calc(robot.dribbler->get_velocity());
                debug_pose = robot.dribbler_filter->get_data();
                debug_vel = robot.dribbler_filter->get_diff();
                
                if (debug_enable) {
                    dribblerInput = debug_K * (debug_angle_ref - robot.dribbler->get_angle()) + debug_D * debug_pose + debug_M * debug_vel;
                } else {
                    dribblerInput = 0;
                }
                robot.dribbler->set_torque_input(dribblerInput);
                robot.dribbler->encode(can2_tx_data);
                can2_tx_data[2] = dribbler_mode;
                
                // Release mutex
                osMutexRelease(mtx_robot_stateHandle);
            }
            
            ctrl_end_tick = TIM2->CNT;
            // Control loop execution time can be monitored here
            volatile uint32_t exec_time = (ctrl_end_tick >= ctrl_start_tick) ? (ctrl_end_tick - ctrl_start_tick) : (0xFFFFFFFF - ctrl_start_tick + ctrl_end_tick);
        }
    }
}

} // extern "C"
