#include "cmsis_os.h"
#include "freertos_vars.h"
#include "can_callbacks.h"
#include "z_main.h"
#include <cstring>

// Debug variables
volatile uint8_t idx_debug = 0;
volatile float id_debug = 0;

extern "C" {

// MotorRxTask - Process motor feedback messages
void StartMotorRxTask(void *argument) {
    osDelay(100);  // Wait for initialization
    
    can_Message_t fb_msg;
    
    for(;;) {
        // Block waiting for motor feedback from queue
        if (osMessageQueueGet(q_motor_fbHandle, &fb_msg, NULL, osWaitForever) == osOK) {
            id_debug = fb_msg.id; // Debug: output received CAN ID                

            // Match wheel motor feedback by configured feedback CAN ID instead of hardcoded IDs.
            int matched_wheel_idx = -1;
            for (int i = 0; i < 4; ++i) {
                if (robot.wheel_motors[i] != nullptr &&
                    fb_msg.id == robot.wheel_motors[i]->feedback_can_id()) {
                    matched_wheel_idx = i;
                    break;
                }
            }

            idx_debug = matched_wheel_idx; // Debug: output matched wheel index

            if (matched_wheel_idx >= 0) {
                if (!robot.wheel_motors[matched_wheel_idx]->is_writing_register()) {
                    robot.wheel_motors[matched_wheel_idx]->parse_feedback_data(fb_msg.buf);
                    // if (!robot.wheel_motors[matched_wheel_idx]->is_enabled()) {
                    //     can_Message_t msg;
                    //     robot.wheel_motors[matched_wheel_idx]->build_clear_error_msg(msg);
                    //     osSemaphoreAcquire(sem_can_txHandle, osWaitForever);
                    //     can2_bus.send_message(msg);
                    //     osSemaphoreRelease(sem_can_txHandle);
                    //     robot.wheel_motors[matched_wheel_idx]->build_enable_msg(msg);
                    //     osSemaphoreAcquire(sem_can_txHandle, osWaitForever);
                    //     can2_bus.send_message(msg);
                    //     osSemaphoreRelease(sem_can_txHandle);
                    // }
                }
            } else if (robot.dribbler != nullptr && fb_msg.id == robot.dribbler->feedback_can_id()) {
                robot.dribbler->parse_feedback_data(fb_msg.buf);
            }

        }
    }
}

} // extern "C"
