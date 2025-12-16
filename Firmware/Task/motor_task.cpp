#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include <cstring>

extern "C" {

// MotorRxTask - Process motor feedback messages
void StartMotorRxTask(void *argument) {
    osDelay(100);  // Wait for initialization
    
    Motor::Feedback_t fb;
    
    for(;;) {
        // Block waiting for motor feedback from queue
        if (osMessageQueueGet(q_motor_fbHandle, &fb, NULL, osWaitForever) == osOK) {
            
            // Acquire robot state mutex
            if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
                
                // Decode motor feedback based on ID
                // IDs 0x201-0x204 are wheel motors (IDs 1-4)
                // ID 0x205 is dribbler (ID 5)
                
                if (fb.motor_id >= 0x201 && fb.motor_id <= 0x204) {
                    uint8_t motor_idx = fb.motor_id - 0x201;
                    if (motor_idx < 4 && robot.wheel_motor[motor_idx] != nullptr) {
                        robot.wheel_motor[motor_idx]->decode(fb.data);
                    }
                } else if (fb.motor_id == 0x205) {
                    if (robot.dribbler != nullptr) {
                        robot.dribbler->decode(fb.data);
                    }
                }
                
                osMutexRelease(mtx_robot_stateHandle);
            }
        }
    }
}

} // extern "C"
