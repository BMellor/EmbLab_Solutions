
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

#define _BV(x) (1 << x)
#define TRUE 1
#define FALSE 0

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

int printStr(char*);
void printChar(char);

volatile _Bool rx_flag; // C-standard defines _Bool as smallest integer datatype (not an official type/keyword though)
volatile uint8_t data;

int main(void) {
    
    HAL_Init(); /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    SystemClock_Config(); /* Configure the system clock */

     /* Enable Peripheral Clocks in RCC ---------------------------------------*/
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    
    /* Configure GPIOC (LED & USART3) Pins ------------------------------------*/
    uint32_t MODE = _BV(18) | _BV(16) | _BV(14) | _BV(12);  // Set PC6 through PC8 to output
             MODE |= _BV(9) | _BV(11);                      // Set PC4 & PC5 to alternate function
    GPIOC->MODER = MODE;  
    GPIOC->AFR[0] = _BV(16) | _BV(20);  // Set PC4 & PC5 to AF1 (USART3 TX & RX)

    /* Configure USART3 to 115200 Baud and set up RXNEIE interrupt ------------*/
    USART3->BRR = HAL_RCC_GetHCLKFreq()/115200; // Set Baud to 115200 (0.06% error)
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // Enable TX, RX, and RX register not empty interrupt
    USART3->CR1 |= USART_CR1_UE;  // Start USART (locks config bits)
    NVIC_EnableIRQ(USART3_4_IRQn); // Enable shared USART3 interrupt
    NVIC_SetPriority(USART3_4_IRQn, 1); // Set to high priority
    
    enum {
        NONE = 0,
        RED = 6,
        BLUE = 7,
        ORANGE = 8,        
        GREEN = 9
    } color = NONE;
    
    printStr("CMD? ");
    
    while (1) {
        __WFI(); // Wait until interrupt occurs before checking data
        
        if(rx_flag) {
            
            if(color == NONE) {
                switch(data) {
                    case 'r':
                    case 'R':    
                        color = RED;
                        printStr("Red ");
                        break;
                    case 'b':
                    case 'B':
                        color = BLUE;
                        printStr("Blue ");
                        break;
                    case 'o':
                    case 'O':
                        color = ORANGE;
                        printStr("Orange ");
                        break; 
                    case 'g':
                    case 'G':
                        color = GREEN;
                        printStr("Green ");
                        break;
                    default:
                        color = NONE;
                        printStr("*Unknown color! (r,b,g,o)");
                }
            } else {
                switch(data) {
                    case '0':
                        GPIOC->BSRR = _BV((color+16));
                        printStr("Off");
                        break;
                    case '1':
                        GPIOC->BSRR = _BV(color);
                        printStr("On");
                        break;
                    case '2':
                        GPIOC->ODR ^= _BV(color);
                        printStr("Toggle");
                        break;
                    default:
                        printStr("*Bad option! (0-2)");
                }
                color = NONE;
            }
            
            if( color == NONE) {
                printStr("\n\rCMD? ");
            }
            
            rx_flag = FALSE;
        }
    }
}

// USART Interrupt Handler
void USART3_4_IRQHandler(void) {
    if(USART3->ISR & USART_ISR_RXNE) { // Triggered by new data in RX register
        data = USART3->RDR;  // Automatically clears interrupt RXNE flag
        rx_flag = TRUE;
    }
}

int printStr(char* str) {
    char* start = str;
    while(*str) {  // Loop until null char encountered
        printChar(*str);
        str++;
    }
    return str-start; // Return number of bytes sent
}

void printChar(char c) {
    while(!(USART3->ISR & USART_ISR_TXE)); // Loop until the TX register is empty (TXE bit is set)
    USART3->TDR = c;
}

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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
