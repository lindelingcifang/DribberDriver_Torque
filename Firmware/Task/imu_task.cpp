#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include <cstring>

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
        
        // Acquire robot state mutex
        if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
            
            // Decode IMU data from UART buffer
            imu.decode(robot.imu_rx_data);
            
            // Signal IMU data ready
            osSemaphoreRelease(sem_imu_readyHandle);
            
            osMutexRelease(mtx_robot_stateHandle);
        }
        
        // Run at moderate rate (~200Hz)
        osDelay(5);
    }
}

} // extern "C"
