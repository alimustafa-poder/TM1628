#include "main.h"
#include "stm32f0xx_hal_tim.h"

TIM_HandleTypeDef htim1;

#define PWM_DUTY 80  // percent (0-100)
#define PWM_PERIOD 65535  // For 20kHz, depends on timer clock

// Hall pins: assumed PA6, PA7, PB0
#define READ_HALL()  (((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) << 2) | \
                       (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) << 1) | \
                       (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0))) & 0x07)

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
void set_phase(uint8_t hall);
void Error_Handler(void);



int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    // Start all 3 complementary PWM channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

//    __HAL_TIM_MOE_DISABLE(&htim1);  // Disable all outputs temporarily

//    // Disable all outputs
//    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
//                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
//                    TIM_CCER_CC3E | TIM_CCER_CC3NE);
//
//    // Clear OCM bits and set proper PWM mode
//    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
//    TIM1->CCMR1 |= TIM_CCMR1_OC1M_1;  // PWM mode for channel 1
//
//    // Set polarity: CCxP = 0 (normal), CCxNP = 1 (inverted for complementary channel)
//    TIM1->CCER &= ~TIM_CCER_CC1P;  // Normal polarity for channel 1 (high-side)
//    TIM1->CCER |= TIM_CCER_CC1NP;  // Inverted polarity for complementary channel 1 (low-side)
//
//    TIM1->CCER &= ~TIM_CCER_CC2P;  // Normal polarity for channel 2 (high-side)
//    TIM1->CCER |= TIM_CCER_CC2NP;  // Inverted polarity for complementary channel 2 (low-side)
//
//    TIM1->CCER &= ~TIM_CCER_CC3P;  // Normal polarity for channel 3 (high-side)
//    TIM1->CCER |= TIM_CCER_CC3NP;  // Inverted polarity for complementary channel 3 (low-side)
//
//    // Enable the PWM outputs
//    TIM1->CCER |= TIM_CCER_CC3E;   // Enable channel C (high-side)
//    TIM1->CCER |= TIM_CCER_CC2NE;  // Enable complementary channel B (low-side, inverted)
//
//    __HAL_TIM_MOE_ENABLE(&htim1);  // Enable main output
//
//    // Set duty cycle if needed
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (PWM_DUTY * PWM_PERIOD) / 100);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (PWM_DUTY * PWM_PERIOD) / 100);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (PWM_DUTY * PWM_PERIOD) / 100);


//        TIM1->CCER |= (TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP);

    while (1)
    {
        uint8_t hall = READ_HALL();

        set_phase(hall);// Update active IGBTs based on rotor position
        HAL_Delay(1); // Polling rate
    }
}


void set_phase(uint8_t hall)
{
	__HAL_TIM_MOE_DISABLE(&htim1);
	__HAL_TIM_DISABLE(&htim1);  // Stop counter safely


    // Disable all high-side and low-side outputs
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);

    // Clear any previous polarity misconfiguration (optional redundancy)
    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP |
                    TIM_CCER_CC2P | TIM_CCER_CC2NP |
                    TIM_CCER_CC3P | TIM_CCER_CC3NP);

    // Set complementary output polarities (CCxNP = 1, active low for low-side)
    TIM1->CCER |= (TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP);

    switch(hall)
    {
    case 0b001:  // Step 1: C+, B-
        TIM1->CCER |= TIM_CCER_CC3E;    // C high-side on
        TIM1->CCER |= TIM_CCER_CC2NE;   // B low-side on
        break;

    case 0b101:  // Step 2: A+, B-
        TIM1->CCER |= TIM_CCER_CC1E;    // A high-side on
        TIM1->CCER |= TIM_CCER_CC2NE;   // B low-side on
        break;

    case 0b100:  // Step 3: A+, C-
        TIM1->CCER |= TIM_CCER_CC1E;    // A high-side
        TIM1->CCER |= TIM_CCER_CC3NE;   // C low-side
        break;

    case 0b110:  // Step 4: B+, C-
        TIM1->CCER |= TIM_CCER_CC2E;    // B high-side
        TIM1->CCER |= TIM_CCER_CC3NE;   // C low-side
        break;

    case 0b010:  // Step 5: B+, A-
        TIM1->CCER |= TIM_CCER_CC2E;    // B high-side
        TIM1->CCER |= TIM_CCER_CC1NE;   // A low-side
        break;

    case 0b011:  // Step 6: C+, A-
        TIM1->CCER |= TIM_CCER_CC3E;    // C high-side
        TIM1->CCER |= TIM_CCER_CC1NE;   // A low-side
        break;

    default:
        // Invalid Hall state or motor stop — leave all disabled
        break;
    }

    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim1);

}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 32767;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 255;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure GPIOA6 (Hall sensor A)
      GPIO_InitStruct.Pin = GPIO_PIN_6;  // Configure pin 6 of GPIOA
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Set as input
      GPIO_InitStruct.Pull = GPIO_NOPULL;  // No pull-up or pull-down resistor
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // Initialize GPIOA6

      // Configure GPIOA7 (Hall sensor B)
      GPIO_InitStruct.Pin = GPIO_PIN_7;  // Configure pin 7 of GPIOA
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // Initialize GPIOA7

      // Configure GPIOB (Hall sensor C, assuming GPIOB pin number)
      GPIO_InitStruct.Pin = GPIO_PIN_0;  // Configure pin 0 of GPIOB (or another pin as required)
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // Initialize GPIOB0 (adjust as necessary)

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // TIM1 CH1N/CH2N/CH3N complementary outputs
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
