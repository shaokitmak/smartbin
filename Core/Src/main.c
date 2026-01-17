/* Actuator Control (Servo & Status LEDs) */
#include "main.h"

void Update_Servo_Position(uint32_t now) {
    if (now - last_servo_move > 15) {
        if (current_pos < target_pos) current_pos += 20;
        else if (current_pos > target_pos) current_pos -= 20;

        // Safety clamp for SG90 servo
        if (current_pos < 500)  current_pos = 500;
        if (current_pos > 2500) current_pos = 2500;

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos);
        last_servo_move = now;
    }
}

void Set_Bin_Indicators(uint8_t open) {
    HAL_GPIO_WritePin(LED_OPEN_GPIO_Port, LED_OPEN_Pin, open ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CLOSED_GPIO_Port, LED_CLOSED_Pin, open ? GPIO_PIN_RESET : GPIO_PIN_SET);
}