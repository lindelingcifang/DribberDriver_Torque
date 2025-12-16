#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include "spi.h"
#include <cstring>

extern "C" {

// SpiExchangeTask - Handle SPI communication with CM4
void StartSpiExchangeTask(void *argument) {
    osDelay(100);  // Wait for initialization
    TickType_t last_wake_time = xTaskGetTickCount();

    for(;;) {
        // Acquire SPI buffer mutex
        if (osMutexAcquire(mtx_spi_bufferHandle, 10) == osOK) {
            
            // Acquire robot state mutex for encoding
            if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
                
                // Encode data to send to CM4
                robot.pi_encode_spi();
                
                // Decode data received from CM4
                robot.pi_decode_spi();

                HAL_SPI_TransmitReceive_DMA(&hspi1, robot.spi_tx_data, robot.spi_rx_data, SPI_LENGTH);
                
                osMutexRelease(mtx_robot_stateHandle);
            }
            
            osMutexRelease(mtx_spi_bufferHandle);
        }
        
        // Exchange at 100Hz
        // osDelay(10);
        vTaskDelayUntil(&last_wake_time, 10);
    }
}

} // extern "C"
