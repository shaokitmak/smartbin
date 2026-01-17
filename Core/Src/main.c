/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    BIN_CLOSED = 0,
    BIN_OPEN_WAITING,
    BIN_MANUAL_OPEN
} BinState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
I2C_LCD_HandleTypeDef my_lcd;

// Sensor Values
uint16_t Distance = 0;
uint16_t FillDistance = 0;
int FillPercentage = 0;

// Timing Tracking
uint32_t last_sensor_time = 0;
uint32_t motion_timestamp = 0;
uint32_t last_servo_move = 0;
static uint32_t last_fill_time = 0;

// State Variables
int current_pos = 1500;
int target_pos = 1500;
BinState_t current_state = BIN_CLOSED;
uint8_t was_full = 0;

// Buzzer Variables
uint8_t  buzzer_beeps_remaining = 0;
uint32_t buzzer_next_toggle = 0;
uint16_t buzzer_interval = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

float HCSR04_Read(GPIO_TypeDef* TRIG_PORT, uint16_t TRIG_PIN, GPIO_TypeDef* ECHO_PORT, uint16_t ECHO_PIN);
void Update_Servo_Position(uint32_t now);
void Set_Bin_Indicators(uint8_t open);
void Update_Fill_Level(uint32_t now);
void buzzer_start_sequence(uint8_t times, uint16_t duration_ms);
void Process_Buzzer(uint32_t now);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* Force initial servo position */
  current_pos = 1500;
  target_pos  = 1500;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos);
  HAL_Delay(400);      // Allow servo to lock position

  HAL_TIM_Base_Start(&htim3);
  HAL_Delay(200);

  // Setup LCD configuration
  my_lcd.hi2c = &hi2c1;
  HAL_Delay(100);
  my_lcd.address = 0x27<<1;

  HAL_Delay(500);          // Let LCD fully power up
  lcd_init(&my_lcd);
  lcd_clear(&my_lcd);

  HAL_GPIO_WritePin(LED_CLOSED_GPIO_Port, LED_CLOSED_Pin, GPIO_PIN_SET); // Red LED ON

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
    {
        uint32_t now = HAL_GetTick();

        // 1. SENSOR POLLING (Motion Sensor)
        if ((now - last_sensor_time > 150) && (current_state == BIN_CLOSED)) {
            float d = HCSR04_Read(TRIG1_GPIO_Port, TRIG1_Pin, ECHO1_GPIO_Port, ECHO1_Pin);
            Distance = (d <= 2.0f || d > 400.0f) ? 999 : (uint16_t)d;
            last_sensor_time = now;
        }

        // 2. FILL LEVEL (LCD Update)
        Update_Fill_Level(now);

        // 3. STATE MACHINE
        switch (current_state) {
            case BIN_CLOSED:
                {

                    uint8_t hand_detected = (Distance >= 3 && Distance < 20);

                    if (hand_detected  && now > 2000) {
                        target_pos = 500; // Open Lid
                        motion_timestamp = now;
                        Set_Bin_Indicators(1);
                        buzzer_start_sequence(2, 100);
                        current_state = BIN_OPEN_WAITING;
                    }
                }
                break;

            case BIN_OPEN_WAITING: // State 1
            	if (now - motion_timestamp > 10000) {
            	    target_pos = 1500;
            	    Set_Bin_Indicators(0);
            	    Distance = 999;
            	    current_state = BIN_CLOSED;
            	}
                break;

			case BIN_MANUAL_OPEN: // State 3
				if (now - motion_timestamp > 10000) {
					 target_pos = 1500;
					 Set_Bin_Indicators(0);
					 Distance=999;
					 current_state = BIN_CLOSED;
				}
				break;
        }

        Update_Servo_Position(now);
        Process_Buzzer(now);
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG1_Pin|TRIG2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_OPEN_Pin|LED_CLOSED_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIG1_Pin TRIG2_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin|TRIG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO1_Pin ECHO2_Pin */
  GPIO_InitStruct.Pin = ECHO1_Pin|ECHO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_OPEN_Pin LED_CLOSED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED_OPEN_Pin|LED_CLOSED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_press = 0;

    if (GPIO_Pin == GPIO_PIN_3) {
        uint32_t now = HAL_GetTick();

        // debounce: 300 ms
        if (now - last_press < 300) return;
        last_press = now;

        // only allow manual open from closed state
        if (current_state == BIN_CLOSED) {
            current_state = BIN_MANUAL_OPEN;
            target_pos = 500;
            motion_timestamp = now;
            Set_Bin_Indicators(1);
            buzzer_start_sequence(2, 100);
        }
    }
}


// Trigger a beep sequence
void buzzer_start_sequence(uint8_t times, uint16_t duration_ms) {
    buzzer_beeps_remaining = times * 2; // 2 toggles per beep (ON then OFF)
    buzzer_interval = duration_ms;
    buzzer_next_toggle = HAL_GetTick();
}

void Process_Buzzer(uint32_t now) {
    if (buzzer_beeps_remaining > 0) {
        if (now >= buzzer_next_toggle) {
            HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
            buzzer_beeps_remaining--;
            buzzer_next_toggle = now + buzzer_interval;
        }
    } else {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
}

// open led and close led function
void Set_Bin_Indicators(uint8_t open) {
    if (open) {
        HAL_GPIO_WritePin(LED_OPEN_GPIO_Port, LED_OPEN_Pin, GPIO_PIN_SET);    // White ON
        HAL_GPIO_WritePin(LED_CLOSED_GPIO_Port, LED_CLOSED_Pin, GPIO_PIN_RESET); // Red OFF
    } else {
        HAL_GPIO_WritePin(LED_OPEN_GPIO_Port, LED_OPEN_Pin, GPIO_PIN_RESET);  // White OFF
        HAL_GPIO_WritePin(LED_CLOSED_GPIO_Port, LED_CLOSED_Pin, GPIO_PIN_SET);   // Red ON
    }
}

// update lcd with fill level function
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

        char line0[17];

        if (FillPercentage >= 100) {
            snprintf(line0, 17, "FULL! EMPTY BIN ");
            if (was_full == 0) {
                buzzer_start_sequence(3, 200);
                was_full = 1;
            }
        } else {
            snprintf(line0, 17, "                ");
            was_full = 0;
        }

        lcd_gotoxy(&my_lcd, 0, 0);
        lcd_puts(&my_lcd, line0);

        last_fill_time = now;
    }
}

// ultrasonic sensor reading function
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
    uint32_t diff;
    if (t_stop >= t_start)
        diff = t_stop - t_start;
    else
        diff = 65535 - t_start + t_stop;

    return (float)(diff * 0.034f / 2.0f);
}

// servo position updating function
void Update_Servo_Position(uint32_t now)
{
    if (now - last_servo_move > 15) {

        if (current_pos < target_pos) current_pos += 20;
        else if (current_pos > target_pos) current_pos -= 20;

        // safety clamp
        if (current_pos < 500)  current_pos = 500;
        if (current_pos > 2500) current_pos = 2500;

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos);
        last_servo_move = now;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
