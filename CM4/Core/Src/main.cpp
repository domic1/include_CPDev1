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
#include "stm32h7xx_hal_hsem.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cstdio>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_hsem.h"
#include "../../../Common/_common.h"
<<<<<<< HEAD
//#include "../Lib/CPDev_XCPcodes/2021/mach32b/32b/cpyMem.h"
=======
#include "../Lib/CPDev_XCPcodes/2021/mach32b/32b/cpyMem.h"
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d
#include "../Lib/CPDevVM/src/vm_arduino.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define xcp_code test

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/

<<<<<<< HEAD
=======

>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
<<<<<<< HEAD
=======

>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;


/* USER CODE BEGIN PV */
VMArduino cpdev;

<<<<<<< HEAD
/*Projekt1 */

const unsigned char test[] = { 0x1C, 0x15, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x1C, 0x15, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00,
		0x00, 0x1C, 0x15, 0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x16, 0x00, 0x00, 0x00, 0x00, 0x54,
		0x01, 0x00, 0x00, 0x1C, 0x16, 0x00, 0x00, 0x00, 0x00, 0x83, 0x01, 0x00, 0x00, 0x1C, 0x1D, 0x33,
		0x00, 0x00, 0x00, 0x1C, 0x15, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x05, 0x00, 0x00,
		0x00, 0x02, 0x00, 0x00, 0x1C, 0x15, 0x07, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x0A, 0x00,
		0x00, 0x00, 0x02, 0x01, 0x00, 0x1C, 0x13, 0x1C, 0x02, 0x01, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
		0x00, 0x1C, 0x15, 0x05, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x1C, 0x00, 0xE4, 0x00, 0x00, 0x00,
		0x1C, 0x02, 0x00, 0x00, 0x00, 0x00, 0xAC, 0x00, 0x00, 0x00, 0x05, 0x10, 0x09, 0x00, 0x00, 0x00,
		0x07, 0x00, 0x00, 0x00, 0x1C, 0x02, 0x09, 0x00, 0x00, 0x00, 0xAC, 0x00, 0x00, 0x00, 0x1C, 0x15,
		0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x1C, 0x00, 0xB4, 0x00, 0x00, 0x00, 0x1C, 0x15, 0x08, 0x00,
		0x00, 0x00, 0x01, 0x00, 0x1C, 0x02, 0x08, 0x00, 0x00, 0x00, 0xE4, 0x00, 0x00, 0x00, 0x14, 0x02,
		0x08, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x1C, 0x02, 0x08, 0x00,
		0x00, 0x00, 0xE4, 0x00, 0x00, 0x00, 0x01, 0x22, 0x05, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
		0x0A, 0x00, 0x00, 0x00, 0x11, 0x02, 0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0x00,
		0x00, 0x00, 0x1C, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x13,
		0x1C, 0x15, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x1C, 0x13, 0x1C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x01, 0x00, 0x00, 0x05, 0x10, 0x03, 0x00,
		0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x1C, 0x02, 0x03, 0x00, 0x00, 0x00, 0x3E, 0x01, 0x00, 0x00,
		0x1C, 0x15, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x1C, 0x00, 0x46, 0x01, 0x00, 0x00, 0x1C, 0x15,
		0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x1F, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x1C, 0x13, 0x1C, 0x16, 0x0B, 0x00, 0x00, 0x00, 0x43, 0x00, 0x00, 0x00, 0x1C, 0x16,
		0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x1C, 0x15, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x00,
		0x1C, 0x15, 0x17, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x18, 0x00, 0x00, 0x00, 0x02, 0x00,
		0x00, 0x1C, 0x13, 0x1C, 0x1F, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C,
		0x16, 0x06, 0x00, 0x00, 0x00, 0x12, 0x01, 0x00, 0x00, 0x1C, 0x1F, 0x0A, 0x00, 0x00, 0x00, 0x07,
		0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x1F, 0x0B, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01,
		0x00, 0x1C, 0x1F, 0x0C, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x15, 0x0D,
		0x00, 0x00, 0x00, 0x02, 0x0A, 0x00, 0x1C, 0x16, 0x0B, 0x00, 0x00, 0x00, 0x67, 0x00, 0x00, 0x00,
		0x1C, 0x1F, 0x17, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1C, 0x1F, 0x18, 0x00,
		0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x02, 0x00, 0x1C, 0x1F, 0x01, 0x00, 0x00, 0x00, 0x17, 0x00,
		0x00, 0x00, 0x01, 0x00, 0x1C, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x02, 0x00,
		0x1C, 0x13 };

=======
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d
/* Align X to 4 bytes */
#define MEM_ALIGN(x)                        (((x) + 0x00000003) & ~0x00000003)

/* Shared RAM between 2 cores is SRAM4 in D3 domain */
#define SHD_RAM_START_ADDR                  0x38000000
#define SHD_RAM_LEN                         0x0000FFFF
#define HSEM_PROCESS_ID 0U

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

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

<<<<<<< HEAD
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */
=======
  /* USER CODE BEGIN 1 */
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */

<<<<<<< HEAD
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/* Activate HSEM notification for Cortex-M4*/
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	/*
	 Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
	 perform system initialization (system clock config, external memory configuration.. )
	 */
	HAL_PWREx_ClearPendingEvent();
	HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
	/* Clear HSEM flag */
	__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
=======
/* USER CODE BEGIN Boot_Mode_Sequence_1 */

  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d

	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

//  volatile uint8_t *ptrStat = (volatile uint8_t *)SHD_RAM_START_ADDR;
<<<<<<< HEAD
	/* USER CODE END Init */
=======

  /* USER CODE END Init */
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

<<<<<<< HEAD
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
//	HAL_UART_Transmit(&huart1, (uint8_t*) "CM4 Started\r\n", 13, 700);
	if (cpdev.VMP_LoadProgramFromArray(xcp_code) != 0)
	{
//		printf("Cannot load program into VM");
		HAL_Delay(1000);

		while (1)
			;
	} else
	{
//		printf("VM loaded successfully\n");
		cpdev.task_cycle = 300;
		cpdev.WM_Initialize(WM_MODE_FIRST_START | WM_MODE_NORMAL);

	}
	/*WM_BOOL LED;*/
	WM_INT CNT;
	WM_BOOL IN1, OUT1, RST;
	uint32_t lastStartCycle;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if ((HAL_GetTick() - lastStartCycle > cpdev.task_cycle))
		{
			if (!cpdev.bRunMode)
			{
				cpdev.WM_Shutdown();
				continue;
			}

			lastStartCycle = HAL_GetTick();

			/*		 	  	  	VMP_PreCycle(void);
			 Program pierwszy */

			IN1 = !HAL_GPIO_ReadPin(JOY_LEFT_GPIO_Port, JOY_LEFT_Pin);

			cpdev.WM_SetData(0, sizeof(WM_BOOL), &IN1);

			if (HAL_HSEM_Take(HSEM_ID_0, HSEM_PROCESS_ID) == HAL_OK)
			{
				RST = ptrStat[5];

				HAL_HSEM_Release(HSEM_ID_0, HSEM_PROCESS_ID);
			}
			/*		 	  	cpdev.VMP_PreCycle(void); */
			cpdev.WM_RunCycle();
			/*		 	  	cpdev.VMP_PostCycle();*/

			cpdev.WM_GetData(1, sizeof(WM_BOOL), (WM_BYTE*) &OUT1);
			cpdev.WM_GetData(3, sizeof(WM_INT), (WM_BYTE*) &CNT);

			if (HAL_HSEM_Take(HSEM_ID_0, HSEM_PROCESS_ID) == HAL_OK)
			{
				ptrStat[0] = IN1;
				ptrStat[1] = OUT1;
				memcpy(ptrStat+3, &CNT,  sizeof(WM_INT));

				HAL_HSEM_Release(HSEM_ID_0, HSEM_PROCESS_ID);
			}

			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (GPIO_PinState) OUT1);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (GPIO_PinState) RST);
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
=======
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, (uint8_t*) "CM4 Started\r\n", 13, 700);
      	  if (cpdev.VMP_LoadProgramFromArray(xcp_code) != 0) {
        		printf("Cannot load program into VM");
        		HAL_Delay(1100);

        		while (1)
        			;
        	} else {
        		printf("VM loaded successfully\n");
        		cpdev.task_cycle = 300;
        		cpdev.WM_Initialize(WM_MODE_FIRST_START | WM_MODE_NORMAL);

        	}
/*WM_BOOL LED;*/
WM_BYTE CNT;
WM_BOOL IN1, OUT1, RST;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/*if(cpdev.bRunMode){
	cpdev.WM_RunCycle();
}else{
	cpdev.WM_Shutdown();
}
cpdev.WM_GetData(0, 1, (WM_BYTE*) &LED);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (GPIO_PinState) LED);
	HAL_Delay(400);

}*/
	 if (!cpdev.bRunMode) {
		 	  		  cpdev.WM_Shutdown();continue; }

/*		 	  	  	VMP_PreCycle(void);
		 	  Program pierwszy */
		 	  //ZPISUJEMY CZAS


		 	  	 IN1 = HAL_GPIO_ReadPin(JOY_LEFT_GPIO_Port, JOY_LEFT_Pin);

		 	  	cpdev.WM_SetData(0, 1, &IN1);


		 	  	if (HAL_HSEM_Take(HSEM_ID_0, HSEM_PROCESS_ID) == HAL_OK)
		 	  		{
		 	  		RST = ptrStat[5];


		 	  		HAL_HSEM_Release(HSEM_ID_0, HSEM_PROCESS_ID);
		 	  		}
/*		 	  	cpdev.VMP_PreCycle(void); */
		 	  	cpdev.WM_RunCycle();
/*		 	  	cpdev.VMP_PostCycle();*/

		 	  cpdev.WM_GetData(1, 1, (WM_BYTE*) &OUT1);
		 	  cpdev.WM_GetData(4, 2, (WM_BYTE*) &CNT);

		 	  	if (HAL_HSEM_Take(HSEM_ID_0, HSEM_PROCESS_ID) == HAL_OK)
		 	  		{
		 	  		ptrStat[0]= IN1;
		 	  		ptrStat[1] = OUT1 ;
		 	  		ptrStat[4] = CNT ;

		 	  		HAL_HSEM_Release(HSEM_ID_0, HSEM_PROCESS_ID);
		 	  		}


		 	  			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (GPIO_PinState) OUT1);
		 	  			ptrStat[10]=10;

		 	  			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (GPIO_PinState) RST);

if(ptrStat[10] ==10){
	printf("Its working");


}





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d
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
<<<<<<< HEAD
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
=======
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d
void MX_USART1_UART_Init(void)
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

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

<<<<<<< HEAD
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOI, LED3_Pin | LED4_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : JOY_RIGHT_Pin JOY_LEFT_Pin */
	GPIO_InitStruct.Pin = JOY_RIGHT_Pin | JOY_LEFT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

	/*Configure GPIO pins : LED3_Pin LED4_Pin */
	GPIO_InitStruct.Pin = LED3_Pin | LED4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
=======
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, LED3_Pin|LED4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : JOY_RIGHT_Pin JOY_LEFT_Pin */
    GPIO_InitStruct.Pin = JOY_RIGHT_Pin|JOY_LEFT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
>>>>>>> e39e29e63c7700784c01c46106d6e4dc94b73a5d
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
