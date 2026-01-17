/* Person 2: LCD Interface and Buzzer Notification System */
#include "main.h"
#include "i2c-lcd.h"
#include <stdio.h>

void Update_Fill_Level(uint32_t now) {
    if (now - last_fill_time > 500) {
        float raw_dist = HCSR04_Read(TRIG2_GPIO_Port, TRIG2_Pin, ECHO2_GPIO_Port, ECHO2_Pin);
        if (raw_dist < 0) raw_dist = 16.0f;

        FillDistance = (uint16_t)raw_dist;
        FillPercentage = 100 - (((FillDistance - 3) * 100) / (16 - 3));
        if (FillPercentage < 0) FillPercentage = 0;
        if (FillPercentage > 100) FillPercentage = 100;

        char fillBuffer[16];
        sprintf(fillBuffer, "Fill: %d%%      ", FillPercentage);
        lcd_gotoxy(&my_lcd, 0, 1);
        lcd_puts(&my_lcd, fillBuffer);
        
        last_fill_time = now;
    }
}

void Process_Buzzer(uint32_t now) {
    if (buzzer_beeps_remaining > 0 && now >= buzzer_next_toggle) {
        HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
        buzzer_beeps_remaining--;
        buzzer_next_toggle = now + buzzer_interval;
    } else if (buzzer_beeps_remaining == 0) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
}