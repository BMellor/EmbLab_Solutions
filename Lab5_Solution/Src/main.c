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
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    
    /* Configure GPIOC (LED & CS) Pins ---------------------------------------*/
    GPIOC->MODER |= (1 << 18) | (1 << 16) | (1 << 14) | (1 << 12) | (1 << 0);    // Set PC9-PC6 and PC0 to output mode
    GPIOC->BSRR = (1 << 0);    // Set PC0 (CS) line high, selects I2C mode on gyro
    
    /* Configure GPIOB (I2C) Pins --------------------------------------------*/
    GPIOB->MODER  |= (1 << 23) | (1 << 27) | (1 << 28);  // Set PB11 & PB13 to AF Mode, PB14 to ouput
    GPIOB->OTYPER |= (1 << 11) | (1 << 13);              // Set PB11 & PB13 to open-drain output type
    GPIOB->PUPDR  |= (1 << 22) | (1 << 26);              // Set internal pull-up resistors on PB 11 & PB13
    GPIOB->AFR[1] = 0x00501000;                          // Set AF1 on PB11(I2C2_SDA) & AF5 on PB13(I2C2_SCL)
    GPIOB->BSRR = (1 << 14);                             // Set PB14 (address select) line high
    
    //PB11 is SDA -> goes to PB15
    // PB14 needs to be pulled high as output
    // PB13 is SCL can be directly connected
    
    /* Configure I2C2 Peripheral ---------------------------------------------*/
    I2C2->TIMINGR = (1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);    // Set the timing register from table
    I2C2->CR1 |= I2C_CR1_PE;    // Enable peripheral
    
    /* Read Gyro ID (WHO_AM_I) Reg -------------------------------------------*/
        GPIOC->BSRR = (1 << 6) | (1 << 23); // Set RED LED (just for show)
        
        // Write register address
        I2C2->CR2 = (0x6B << 1) | (1 << 16) | I2C_CR2_START;    // Set SADD, NBYTES & START, clear all other bits (including RD_WRN -> write mode) 
        while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));   // Wait until either TXIS or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(100); // Error condition, no ACK
        } 
        I2C2->TXDR = 0x0F;  // WHO_AM_I register address
        while(!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)));   // Wait until either TC or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(50); // Error condition, no ACK on data
        } 
        
        // Read register contents
        I2C2->CR2 = (0x6B << 1) | (1 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START;    // Set SADD, NBYTES & START, clear all other bits (set RD_WRN -> read mode) 
        while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));   // Wait until either RXNE or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(100); // Error condition, no ACK
        } 

        while(!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)));   // Wait until either TC or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(50); // Error condition, no ACK on data
        } 
        I2C2->CR2 |= I2C_CR2_STOP; // Release I2C Bus
        
        if( I2C2->RXDR != 0xD4) {
             ErrorLoop(500); // Error condition, corrupt ID
        }
        GPIOC->BSRR = (1 << 22) | (1 << 7); // Set GREEN LED and clear RED (just for show)
    
    /* Initialize Gyroscope -----------------------------------------------*/
        GPIOC->BSRR = (1 << 6) | (1 << 23); // Set RED LED (just for show)
        I2C2->CR2 = (0x6B << 1) | (2 << 16) | I2C_CR2_START;    // Set SADD, NBYTES & START, clear all other bits (including RD_WRN -> write mode) 
        while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));   // Wait until either TXIS or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(100); // Error condition, no ACK
        } 
        I2C2->TXDR = 0x20;  // CR1 register address
        while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));   // Wait until either TXIS or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(100); // Error condition, no ACK
        } 
        I2C2->TXDR = (1 << 3) | (1 << 1) | (1 << 0);  // CR1 write value
        while(!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)));   // Wait until either TC or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(50); // Error condition, no ACK on data
        } 
        I2C2->CR2 |= I2C_CR2_STOP; // Release I2C Bus
        GPIOC->BSRR = (1 << 22) | (1 << 7); // Set GREEN LED and clear RED (just for show)
        
    uint8_t lower;
    int16_t total;
        
    while (1) {
        HAL_Delay(100);
        total = 0;
        
        /* Read Gyroscope Value-----------------------------------------------*/
        GPIOC->BSRR = (1 << 6) | (1 << 23); // Set RED LED (just for show)
        
        // Write register address
        I2C2->CR2 = (0x6B << 1) | (1 << 16) | I2C_CR2_START;    // Set SADD, NBYTES & START, clear all other bits (including RD_WRN -> write mode) 
        while(!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));   // Wait until either TXIS or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(100); // Error condition, no ACK
        } 
        I2C2->TXDR = 0xA8;  // OUT_X_L register address (with signal to read multiple regs)
        while(!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)));   // Wait until either TC or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(50); // Error condition, no ACK on data
        } 
        
        // Read lower register contents
        I2C2->CR2 = (0x6B << 1) | (2 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START;    // Set SADD, NBYTES & START, clear all other bits (set RD_WRN -> read mode) 
        while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));   // Wait until either RXNE or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(100); // Error condition, no ACK
        } 
        lower = I2C2->RXDR; // Save lower byte
        
        
        // Read upper register contents
        while(!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));   // Wait until either RXNE or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(100); // Error condition, no ACK
        }
        total = (I2C2->RXDR << 8) | lower; // Save upper byte
        
        while(!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)));   // Wait until either TC or NACKF flags are set
        if(I2C2->ISR & I2C_ISR_NACKF) {      
            ErrorLoop(50); // Error condition, no ACK on data
        } 
        I2C2->CR2 |= I2C_CR2_STOP; // Release I2C Bus
        GPIOC->BSRR = (1 << 22) | (1 << 7); // Set GREEN LED and clear RED (just for show)
        
        // Detect rotation and turn on LEDs
        if(total > 10000) {
            GPIOC->BSRR = (1 << 9) | (1 << 24); // Set ORANGE LED 
        } else if(total < -10000) {
            GPIOC->BSRR = (1 << 25) | (1 << 8); // Set GREEN LED 
        } 
    }
  
}

void ErrorLoop(uint16_t rate) {
     while(1) {
        GPIOC->ODR ^= (1 << 6);    // Flash RED LED infinitely 
        HAL_Delay(rate);
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
