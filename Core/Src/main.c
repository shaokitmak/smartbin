/* Sensor Drivers and Implementation */
#include "main.h"

// Private function prototypes 
float HCSR04_Read(GPIO_TypeDef* TRIG_PORT, uint16_t TRIG_PIN, GPIO_TypeDef* ECHO_PORT, uint16_t ECHO_PIN);

// Ultrasonic sensor reading function using microsecond timer
float HCSR04_Read(GPIO_TypeDef* TRIG_PORT, uint16_t TRIG_PIN, GPIO_TypeDef* ECHO_PORT, uint16_t ECHO_PIN) {
    // 1. Send 10us Trigger
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    while (__HAL_TIM_GET_COUNTER(&htim3) < 10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // 2. Wait for Echo to go HIGH
    uint32_t timeout = 2000;
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET && timeout > 0) timeout--;
    if (timeout == 0) return -1.0f;

    uint32_t t_start = __HAL_TIM_GET_COUNTER(&htim3);

    // 3. Wait for Echo to go LOW
    timeout = 30000;
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET && timeout > 0) timeout--;
    uint32_t t_stop = __HAL_TIM_GET_COUNTER(&htim3);

    // 4. Calculate distance
    uint32_t diff = (t_stop >= t_start) ? (t_stop - t_start) : (65535 - t_start + t_stop);
    return (float)(diff * 0.034f / 2.0f);
}