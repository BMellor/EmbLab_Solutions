
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);


int main(void) {
    HAL_Init();             // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config();   //Configure the system clock

    // #### Enable Peripheral Clocks in RCC ####
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    
    // #### Configure GPIOC (LED) Pins ####
    GPIOC->MODER |= (1 << 14) | (1 << 12);        // Set PC7 and PC6 to output mode
    GPIOC->MODER &= ~((1 << 19) | (1 << 17));
    GPIOC->OTYPER &= ~((1 << 7) | (1 << 6));      // Set push-pull output
    GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 12));   // Set low speed
    GPIOC->PUPDR &= ~((1 << 15) | (1 << 14) | (1 << 13) | (1 << 12));   // Set no-pull up/down
    // Explicitly clearing bits for example (don't really need unless reconfiguring already used pins)
    
    // #### Configure GPIOA (Button) Pin ####
    GPIOA->MODER &= ~((1 << 1) | (1 << 0));      // Set PA0 to input mode, low speed and pull-down
    GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 12));
    GPIOC->PUPDR |= ~(1 << 1);
    GPIOC->PUPDR &= ~(1 << 0);

    // #### Start "Blinky" Program ####
    GPIOC->ODR |= (1 << 6);   // Start PC6 high
    uint32_t debouncer = 0;
    
    while (1) {
        HAL_Delay(1); // Delay 1ms
        debouncer = (debouncer << 1); 
        if (GPIOA->IDR & (1 << 0)) {                // If PA0 is set
            debouncer |= 0x01;                      // Set lowest bit of bit-vector
            if (debouncer == 0x7FFFFFFF) {          // This triggers once when transitioning to steady high
                GPIOC->ODR ^= (1 << 7) | (1 << 6);  // Toggle both PC7 and PC6
            }
        }
    } // End while
    
} // End main()

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
