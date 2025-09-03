/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "string.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_hsem.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cstdio>
#include <stdio.h>

#include <cstring>
#include <iostream>
#include "../../../Common/_common.h"
//#include "shared_memory.h"
#include "../Lib/CPDevVM/src/vm_arduino.h"
//#include "../Lib/CPDev_XCPcodes/2021/mach32b/32b/cpyMem.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define xcp_code test
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
VMArduino cpdev;

<<<<<<< HEAD
/*PROJEKT2*/
const unsigned char test[] = { 0x1C, 0x15, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x1C, 0x15, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x1C, 0x15, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x1C, 0x16, 0x00, 0x00, 0x00, 0x00, 0xEC,
		0x00, 0x00, 0x00, 0x1C, 0x16, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x1C, 0x1D, 0x33,
		0x00, 0x00, 0x00, 0x1C, 0x15, 0x0A, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x15,
		0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x06, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
		0x00, 0x1C, 0x13, 0x05, 0x10, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x02, 0x0E,
		0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x00, 0x1C, 0x15, 0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C,
		0x15, 0x06, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x17, 0x0A, 0x00, 0x00, 0x00,
		0x1C, 0x00, 0xEA, 0x00, 0x00, 0x00, 0x05, 0x10, 0x0E, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
		0x1C, 0x02, 0x0E, 0x00, 0x00, 0x00, 0xEA, 0x00, 0x00, 0x00, 0x1C, 0x17, 0x0F, 0x00, 0x00, 0x00,
		0x02, 0x0B, 0x06, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x11, 0x0B,
		0x0E, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x1C, 0x02, 0x0E, 0x00,
		0x00, 0x00, 0xEA, 0x00, 0x00, 0x00, 0x1C, 0x15, 0x05, 0x00, 0x00, 0x00, 0x01, 0x01, 0x1C, 0x1F,
		0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x1C, 0x13, 0x1C, 0x16, 0x06, 0x00,
		0x00, 0x00, 0x43, 0x00, 0x00, 0x00, 0x1C, 0x15, 0x1A, 0x00, 0x00, 0x00, 0x02, 0x03, 0x00, 0x1C,
		0x13, 0x1C, 0x1F, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x07,
		0x00, 0x00, 0x00, 0x04, 0x88, 0x13, 0x00, 0x00, 0x1C, 0x16, 0x06, 0x00, 0x00, 0x00, 0x63, 0x00,
		0x00, 0x00, 0x1C, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x02,
		0x19, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x1C, 0x02, 0x19, 0x00,
		0x00, 0x00, 0x54, 0x01, 0x00, 0x00, 0x1C, 0x15, 0x02, 0x00, 0x00, 0x00, 0x01, 0x01, 0x1C, 0x00,
		0x5C, 0x01, 0x00, 0x00, 0x1C, 0x15, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x13};
=======
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d

FlagStatus uFlagNotif = RESET;
#define SHD_RAM_START_ADDR                  0x38000000
#define SHD_RAM_LEN                         0x0000FFFF
#define SV 400
#define HSEM_PROCESS_ID 0U
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
extern "C" void SystemClock_Config(void);
extern "C" void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
<<<<<<< HEAD
//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif /* __GNUC__ */
=======
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
<<<<<<< HEAD
//extern "C"
//{
//
////// STM32Cube (newlib-nano + GCC) woła __io_putchar z syscalls.c:_write()
////int __io_putchar(int ch)
////{
////	uint8_t c = static_cast<uint8_t>(ch);
////	HAL_UART_Transmit(&huart1, &c, 1, 1000);
////	return ch;
////}
////
////// (opcjonalnie) jeśli używasz scanf/getchar:
////int __io_getchar(void)
////{
////	uint8_t c;
////	HAL_UART_Receive(&huart1, &c, 1, HAL_MAX_DELAY);
////	return (int) c;
////}
//
//}
//extern "C"
=======
extern "C" {

// STM32Cube (newlib-nano + GCC) woła __io_putchar z syscalls.c:_write()
int __io_putchar(int ch)
{
    uint8_t c = static_cast<uint8_t>(ch);
    HAL_UART_Transmit(&huart1, &c, 1, 1000);
    return ch;
}

// (opcjonalnie) jeśli używasz scanf/getchar:
int __io_getchar(void)
{
    uint8_t c;
    HAL_UART_Receive(&huart1, &c, 1, HAL_MAX_DELAY);
    return (int)c;
}

}  extern "C"
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */
	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;
	/* USER CODE END Boot_Mode_Sequence_0 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	timeout = 0xFFFF;
	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0))
		;
	if (timeout < 0)
	{
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	 HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0, 0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0))
		;
	if (timeout < 0)
	{
		Error_Handler();
	}
	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
//	HAL_UART_Transmit(&huart1, (uint8_t*) "CM7 Started\r\n", 13, 700);
	if (cpdev.VMP_LoadProgramFromArray(xcp_code) != 0)
	{
//		printf("Cannot load program into VM");
		HAL_Delay(1000);

		while (1)
			;
	} else
	{
//		printf("VM loaded successfully\n");
		cpdev.task_cycle = 400;
		cpdev.WM_Initialize(WM_MODE_FIRST_START | WM_MODE_NORMAL);

	}

<<<<<<< HEAD
	WM_BOOL IN1, OUT2, RST;
	WM_INT CNT;
	uint32_t lastStartCycle;
	/*  	WM_BOOL LED;*/

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if ((HAL_GetTick() - lastStartCycle > cpdev.task_cycle))
		{
=======
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, (uint8_t*) "CM7 Started\r\n", 13, 700);
  	if (cpdev.VMP_LoadProgramFromArray(xcp_code) != 0) {
  		printf("Cannot load program into VM");
  		HAL_Delay(1000);

  		while (1)
  			;
  	} else {
  		printf("VM loaded successfully\n");
  		cpdev.task_cycle = 400;
  		cpdev.WM_Initialize(WM_MODE_FIRST_START | WM_MODE_NORMAL);

  	}

    WM_BOOL IN1, OUT1, OUT2, RST;
/*  	WM_BOOL LED;*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

/*if((HAL_GetTick() - lastStartCycle > cpdev.task_cycle)){*/

	  if (!cpdev.bRunMode) {
	      cpdev.WM_Shutdown();continue; }

/*	  	Program drugi
 * cpdev.VMP_PreCycle(void);  */
	  if (HAL_HSEM_Take(HSEM_ID_0, HSEM_PROCESS_ID) == HAL_OK)
	  	    	  	{
		  	  	  IN1 = ptrStat[0];
		  	  	  OUT1 = ptrStat[1];
	  	    	  	HAL_HSEM_Release(HSEM_ID_0, HSEM_PROCESS_ID);
	  	    	  	}

	  	  	cpdev.WM_SetData(0, 1, &IN1);


	    cpdev.WM_RunCycle();
/*	  	cpdev.VMP_PostCycle();*/

	    	 cpdev.WM_GetData(1, 1, (WM_BYTE*) &OUT2);
	    	 cpdev.WM_GetData(2, 1, (WM_BYTE*) &RST);

	    	if (HAL_HSEM_Take(HSEM_ID_0, HSEM_PROCESS_ID) == HAL_OK)
	    	  	{
	    		ptrStat[2] = OUT2;
	    		ptrStat[5] = RST ;

	    	  	HAL_HSEM_Release(HSEM_ID_0, HSEM_PROCESS_ID);
	    	  	}

	    	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (GPIO_PinState) OUT1);
	    	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (GPIO_PinState) OUT2);
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d

			if (!cpdev.bRunMode)
			{
				cpdev.WM_Shutdown();
				continue;
			}

<<<<<<< HEAD
			lastStartCycle = HAL_GetTick();

			/*	  	Program drugi
			 * cpdev.VMP_PreCycle(void);  */

			if (HAL_HSEM_Take(HSEM_ID_0, HSEM_PROCESS_ID) == HAL_OK)
			{
				IN1 = ptrStat[0];
				memcpy(&CNT, ptrStat+3, sizeof(WM_INT));

				HAL_HSEM_Release(HSEM_ID_0, HSEM_PROCESS_ID);
			}

			cpdev.WM_SetData(0, sizeof(WM_BOOL), &IN1);

			cpdev.WM_RunCycle();
			/*	  	cpdev.VMP_PostCycle();*/

			cpdev.WM_GetData(2, sizeof(WM_BOOL), (WM_BYTE*) &OUT2);
			cpdev.WM_GetData(5, sizeof(WM_BOOL), (WM_BYTE*) &RST);

			if (HAL_HSEM_Take(HSEM_ID_0, HSEM_PROCESS_ID) == HAL_OK)
			{
				ptrStat[2] = OUT2;
				ptrStat[5] = RST;

				HAL_HSEM_Release(HSEM_ID_0, HSEM_PROCESS_ID);
			}

			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (GPIO_PinState) OUT2);
		}


		/* USER CODE END WHILE */
	}
	/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
/*}*/
=======
	    		cpdev.task_cycle = 300;


    /* USER CODE END WHILE */
  	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
/*}*/

>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
	{
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 48;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
			| RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.PLL2.PLL2M = 2;
	PeriphClkInitStruct.PLL2.PLL2N = 12;
	PeriphClkInitStruct.PLL2.PLL2P = 2;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOI, LED1_Pin | LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : JOY_UP_Pin JOY_DOWN_Pin JOY_SEL_Pin */
	GPIO_InitStruct.Pin = JOY_UP_Pin | JOY_DOWN_Pin | JOY_SEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

	/*Configure GPIO pin : CEC_CK_MCO1_Pin */
	GPIO_InitStruct.Pin = CEC_CK_MCO1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	HAL_GPIO_Init(CEC_CK_MCO1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin LED2_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

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
