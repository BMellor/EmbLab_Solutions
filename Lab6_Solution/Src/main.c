/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

#define _BV(x) (1<<x)

/* Private variables ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/
void ErrorLoop(uint16_t);

int main(void) {
    HAL_Init();             // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config();   //Configure the system clock
    
    /* Enable Peripheral Clocks in RCC ---------------------------------------*/
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    
    /* Configure GPIOC (LED & ADC) Pins ---------------------------------------*/
    GPIOC->MODER |= _BV(18) | _BV(16) | _BV(14) | _BV(12) | _BV(0) | _BV(1);   // Set PC9-PC6 to output mode, PC0 to analog (ADC_IN10)
        
    /* Configure GPIOA (DAC) Pins ---------------------------------------------*/
    GPIOA->MODER |= _BV(8) | _BV(9);    // Set PA4 to analog (DAC_OUT1)
      
    /* Configure and Start ADC ------------------------------------------------*/
    ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_RES_1;   // 8-bit mode, continuous conversion
    ADC1->CHSELR = ADC_CHSELR_CHSEL10;                // Enable channel 10 (PC0)
    
    ADC1->CR |= ADC_CR_ADCAL;           // Start Calibration!
    while(ADC1->CR & ADC_CR_ADCAL) {}   // Wait until calibration is complete
    
    ADC1->CR |= ADC_CR_ADEN;            // Enable the ADC
    while(ADC1->ISR & ADC_ISR_ADRDY) {} // Wait until warmup is complete  
    
    ADC1->CR |= ADC_CR_ADSTART;         // Start the continuous conversion  
        
        
    /* Configure DAC ----------------------------------------------------------*/
    DAC1->CR |= DAC_CR_TSEL1;   // Set software trigger on channel 1 (bits 3-5)
    DAC1->CR |= DAC_CR_EN1;     // Enable DAC channel 1
        
    uint8_t value;
    // Sine Wave: 8-bit, 32 samples/cycle
    uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
        232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102}; 
    uint8_t index = 0;
        
    while (1) {
        value = ADC1->DR; // Save ADC value
        if(value > 240) {
            GPIOC->BSRR = _BV(6) | _BV(9) | _BV(7) | _BV(8); // Set lights on/off, start with highest value and work down
        } else if(value > 180) {
            GPIOC->BSRR = _BV(6) | _BV(9) | _BV(7) | _BV(24);
        } else if(value > 120) {
            GPIOC->BSRR = _BV(6) | _BV(9) | _BV(28) | _BV(24);
        } else if(value > 60) {
            GPIOC->BSRR = _BV(6) | _BV(25) | _BV(23) | _BV(24);
        } else {
            GPIOC->BSRR = _BV(22) | _BV(25) | _BV(23) | _BV(24);
        }
        
        DAC->DHR8R1 = sine_table[index]; // Write new value to DAC (software trigger will automatically update output)
        if( ++index > 31) {
            index = 0;
        }
        
        HAL_Delay(1); // Sets frequency of output wave (1000 Hz / 32 values  = 31.25 Hz)
    }
  
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {

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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
