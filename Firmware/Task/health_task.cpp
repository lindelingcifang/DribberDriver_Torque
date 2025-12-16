#include "cmsis_os.h"
#include "freertos_vars.h"
#include "z_main.h"
#include "adc.h"
#include "iwdg.h"

extern "C" {

// HealthTask - Watchdog, ADC monitoring, error detection
void StartHealthTask(void *argument) {
    osDelay(150);  // Wait for system startup
    
    uint32_t error_count = 0;
    
    for(;;) {
        // Feed watchdog
        HAL_IWDG_Refresh(&hiwdg);

        float infra_ADC1_val = 0.0f;
        float bat_ADC2_val = 0.0f;
        float cap_ADC3_val = 0.0f;
        
        // Read ADC values
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) {
            infra_ADC1_val = (float)HAL_ADC_GetValue(&hadc1) / 4096.0f * 3.3f;
        }
        
        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 1);
        if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC)) {
            bat_ADC2_val = (float)HAL_ADC_GetValue(&hadc2) / 4096.0f * 3.3f * bat_k;
        }
        
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 1);
        if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc3), HAL_ADC_STATE_REG_EOC)) {
            cap_ADC3_val = (float)HAL_ADC_GetValue(&hadc3) / 4096.0f * 3.3f * cap_k;
        }

        // Update robot state with mutex protection
        if (osMutexAcquire(mtx_robot_stateHandle, 10) == osOK) {
            robot.infra_ADC1_val = infra_ADC1_val;
            robot.bat_ADC2_val = bat_ADC2_val;
            robot.cap_ADC3_val = cap_ADC3_val;
            
            osMutexRelease(mtx_robot_stateHandle);
        }
        
        // Check for CAN errors
        if (can1_bus.error_ != ZCAN::ERROR_NONE) {
            error_count++;
        }
        if (can2_bus.error_ != ZCAN::ERROR_NONE) {
            error_count++;
        }
        
        // Run at 10Hz
        osDelay(100);
    }
}

} // extern "C"
