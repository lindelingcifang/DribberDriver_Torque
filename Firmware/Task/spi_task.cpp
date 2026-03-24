#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include "spi.h"
#include "main.h"
#include <cstring>

volatile uint32_t spi_trigger_dt_us = 0;
volatile uint32_t spi_trigger_last_cnt = 0;
volatile uint32_t spi_trigger_count = 0;

extern "C" {

// SpiExchangeTask - Handle SPI communication with CM4
void StartSpiExchangeTask(void *argument) {
    osDelay(100);  // Wait for initialization
    spi_trigger_last_cnt = TIM13->CNT;

    // Start first SPI transfer, then run as interrupt-triggered loop.
    HAL_SPI_TransmitReceive_DMA(&hspi1, robot.spi_tx_data, robot.spi_rx_data, SPI_LENGTH);

    for(;;) {
        // Wait for SPI DMA transfer complete interrupt to trigger this task.
        if (osSemaphoreAcquire(sem_spi_triggerHandle, osWaitForever) == osOK) {
            uint32_t current_cnt = TIM13->CNT;
            spi_trigger_dt_us = (current_cnt >= spi_trigger_last_cnt)
                                ? (current_cnt - spi_trigger_last_cnt)
                                : (current_cnt + ((1u << 16) - spi_trigger_last_cnt) + 1u);
            spi_trigger_last_cnt = current_cnt;
            spi_trigger_count++;

            // Acquire SPI buffer mutex
            if (osMutexAcquire(mtx_spi_bufferHandle, 10) == osOK) {

                // Acquire robot state mutex for encoding/decoding
                if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {

                    // Encode data to send to CM4
                    robot.pi_encode_spi();

                    // Decode data received from CM4
                    robot.pi_decode_spi();

                    // Feed the watchdog
                    robot.watchdog_feed();

                    // Start next SPI DMA exchange.
                    HAL_SPI_TransmitReceive_DMA(&hspi1, robot.spi_tx_data, robot.spi_rx_data, SPI_LENGTH);

                    osMutexRelease(mtx_robot_stateHandle);
                }

                osMutexRelease(mtx_spi_bufferHandle);
            }
        }
    }
}

} // extern "C"
