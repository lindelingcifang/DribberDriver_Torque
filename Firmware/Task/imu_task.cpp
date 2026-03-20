#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include <cstring>

float imu_last_time = 0;
float imu_dt = 0;

// IMU data structure
struct ImuData {
    float data[9];  // ax, ay, az, gx, gy, gz, roll, pitch, yaw
};

extern "C" {

// ImuRxTask - Process IMU data via UART
void StartImuRxTask(void *argument) {
    osDelay(100);  // Wait for initialization
    
    for(;;) {
        // IMU data arrives via UART DMA - processed in UART callback
        // This task monitors IMU health and processes decoded data
        
        if (osSemaphoreAcquire(sem_imu_readyHandle, osWaitForever) == osOK) {
            float current_time = TIM11->CNT / 1000000.0f;  // Current time in seconds
            imu_dt = (current_time > imu_last_time) ? (current_time - imu_last_time) : (current_time + (0.01f - imu_last_time));
            imu_last_time = current_time;
            // Acquire robot state mutex
            if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
                
                // Decode IMU data from UART buffer
                imu.decode(robot.imu_rx_data);
                
                // // Signal IMU data ready
                // osSemaphoreRelease(sem_imu_readyHandle);
                
                osMutexRelease(mtx_robot_stateHandle);
            }
            
        }
    }
}

} // extern "C"
