/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void transmitOneChar(uint8_t ch) {
  while ((USART3->ISR & USART_ISR_TXE) == 0) {
	}
	USART3->TDR = ch;
}

void transmitCharArray (char *arr) {
  while (*arr != '\0') {
		transmitOneChar(*arr);
		arr++;
	}
}

void initLEDs(void) {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
	// blue LED (PC7), channel 2, red LED PC6, Channel 1, green LED PC9, yellow LED PC8
	// set the MODER, 01: General purpose output mode
	// init PC6 MODER
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER &= ~(1 << 13);
  // init PC7 MODER
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER &= ~(1 << 15);
	// init PC8 MODER
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 17);
	// init PC9 MODER
	GPIOC->MODER |= (1 << 18);
	GPIOC->MODER &= ~(1 << 19);
	
	// Set the pins to low speed in the OSPEEDR register
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 13));
	GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 15));
	GPIOC->OSPEEDR &= ~((1 << 16) | (1 << 17));
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 19));
	
	// Set LED to no pull-up/down resistors in the PUPDR register
	// 00: No pull-up, pull-down
	GPIOC->PUPDR &= ~((1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15));
	
	// set PC6-9 to 1
	GPIOC->ODR |= (1 << 6);
	GPIOC->ODR |= (1 << 7);
	GPIOC->ODR |= (1 << 8);
	GPIOC->ODR |= (1 << 9);
}

char readErrorMsg[30] = "Char not match any LED";

void readCharAndToggleLED(void) {
  while((USART3->ISR & USART_ISR_RXNE) == 0) {
	}
	uint16_t data = USART3->RDR;
	if (data == 'r') {
	  GPIOC->ODR ^= (1 << 6);
	} else if (data == 'b') {
	  GPIOC->ODR ^= (1 << 7);
	} else if (data == 'o') {
		GPIOC->ODR ^= (1 << 8);
	} else if (data == 'g') {
	  GPIOC->ODR ^= (1 << 9);
	} else {
	  transmitCharArray(readErrorMsg);
	}
}

void initUsart3(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
	// set pc4 to AF mode, 0x10
	GPIOC->MODER |= (1 << 9);
	GPIOC->MODER &= ~(1 << 8);
	// set pc5 to AF mode, 0x10
	GPIOC->MODER |= (1 << 11);
	GPIOC->MODER &= ~(1 << 10);
	
	// set PC4 AFRL to 0001: AF1
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFRL4_Pos);
	// set PC5 AFRL to 0001: AF1
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFRL5_Pos);
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	uint32_t fClk = HAL_RCC_GetHCLKFreq();
	
	// set baud rate
	uint32_t baudRate = 115200;
	uint32_t usartBRR = fClk / baudRate;
	USART3->BRR = usartBRR;

	// enable the transmitter and receiver hardware of USART3
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	
	// Enable USART peripheral.
	USART3->CR1 |= USART_CR1_UE;
}

void blockingTransmission(void) {
	char arr[20] = "rbogborg";
  transmitCharArray(arr);
  HAL_Delay(100);
}

void blockingReception(void) {
	readCharAndToggleLED();
}

volatile uint16_t usartReceivedData = 0;
volatile uint8_t usartRcvIntrFlag = 0;

void interruptBasedReceptionInit(void) {
	// Enable the receive register not empty interrupt.
	USART3->CR1 |= USART_CR1_RXNEIE;
	// Enable and set the USART interrupt priority in the NVIC.
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 3);
	
}

char intrRecptPromptMsg[5] = "CMD";
char RecvLEDErrMsg[20] = "LED color not found";
char RecvLEDControlErrMsg[20] = "Control cmd is wrong";

void controlLEDHelper(uint16_t first, uint16_t second) {
  if (second == '0') {
    GPIOC->ODR &= ~(1 << first);
	} else if (second == '1') {
	  GPIOC->ODR |= (1 << first);
	} else {
		GPIOC->ODR ^= (1 << first);
	}
}

void controlLED(uint16_t first, uint16_t second) {
  if (first == 'r') {
	  controlLEDHelper(6, second);
	} else if (first == 'b') {
		controlLEDHelper(7, second);
  } else if (first == 'o') {
	  controlLEDHelper(8, second);
	} else {
		controlLEDHelper(9, second);
	}
}

void interruptBasedReceptionLoop(void) {
  if (usartRcvIntrFlag == 1) {
	  uint16_t firstArg = usartReceivedData;
		usartRcvIntrFlag = 0;
		if (firstArg != 'r' && firstArg != 'g' && firstArg != 'b' && firstArg != 'o') {
		  transmitCharArray(RecvLEDErrMsg);
			return;
	  }
		while (usartRcvIntrFlag == 0) {
	  }
		uint16_t secondArg = usartReceivedData;
		usartRcvIntrFlag = 0;
		if (secondArg != '0' && secondArg != '1' && secondArg != '2') {
		  transmitCharArray(RecvLEDControlErrMsg);
			return;
		}
    controlLED(firstArg, secondArg);
	} else {
	  transmitCharArray(intrRecptPromptMsg);
	}
}

void USART3_4_IRQHandler(void) {
	while((USART3->ISR & USART_ISR_RXNE) == 0) {
	}
	usartReceivedData = USART3->RDR;
	usartRcvIntrFlag = 1;
}

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	initUsart3();
  initLEDs();
	char helloMsg[20] = "hello usart";
	//transmitCharArray(helloMsg);
	
  while (1)
  {
    /* USER CODE END WHILE */
    // transmitCharArray(helloMsg);
		//transmitOneChar('h');
    /* USER CODE BEGIN 3 */
		readCharAndToggleLED();
  }
  /* USER CODE END 3 */
}

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
