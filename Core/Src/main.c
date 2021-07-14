/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "fatfs_sd.h"
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
volatile int index_counter = 0;
volatile char single_char;
volatile char one_line[150];


FATFS fs;
FIL fil;
FRESULT fresult;

char buffer[1024];
UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_FATFS_Init();
	MX_SPI2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	HAL_UART_Receive_IT(&huart3, &single_char, 1);

	printf("Hello!\r\n");

	fresult = f_mount(&fs, "", 0);
	if (fresult != FR_OK) {
		printf("Error when mounting SD card.\r\n");
	} else {
		printf("SD card mounted successfully.\r\n");
	}
	HAL_Delay(10);
	// Check free space
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	printf("SD card total size: %.2f GB\r\n", total / 1024.0 / 1024.0);

	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	printf("SD card free space: %.2f GB\r\n", free_space / 1024.0 / 1024.0);

	// Open file to write/ create a file if it doesn't exist
	fresult = f_open(&fil, "sleep.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	HAL_Delay(10);
	// Writing text
	fresult = f_write(&fil, "Hello World!\r\n", strlen("Hello World!\r\n"),
			&bw);
	fresult = f_write(&fil, "2. line\r\n", strlen("2. line\r\n"), &bw);

	// Close file
	fresult = f_close(&fil);
	printf("file1.txt created and the data is written.\r\n");

#define AT_BUFF_SIZE	100
	char at_buff[AT_BUFF_SIZE] = { '\0' };

	sprintf(at_buff, "AT+CPIN=9526\r\n");
	HAL_UART_Transmit(&huart3, at_buff, strlen(at_buff), 0xFFFF);
	memset(at_buff, '\0', AT_BUFF_SIZE);
	HAL_Delay(13000);

	sprintf(at_buff, "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
	HAL_UART_Transmit(&huart3, at_buff, strlen(at_buff), 0xFFFF);
	memset(at_buff, '\0', AT_BUFF_SIZE);
	HAL_Delay(2000);

	sprintf(at_buff, "AT+SAPBR=3,1,\"APN\",\"internet.vodafone.net\"\r\n");
	HAL_UART_Transmit(&huart3, at_buff, strlen(at_buff), 0xFFFF);
	memset(at_buff, '\0', AT_BUFF_SIZE);
	HAL_Delay(2000);

	sprintf(at_buff, "AT+SAPBR=1,1\r\n");
	HAL_UART_Transmit(&huart3, at_buff, strlen(at_buff), 0xFFFF);
	memset(at_buff, '\0', AT_BUFF_SIZE);
	HAL_Delay(2000);

	sprintf(at_buff, "AT+HTTPINIT\r\n");
	HAL_UART_Transmit(&huart3, at_buff, strlen(at_buff), 0xFFFF);
	memset(at_buff, '\0', AT_BUFF_SIZE);
	HAL_Delay(2000);

	sprintf(at_buff, "AT+HTTPPARA=\"CID\",1\r\n");
	HAL_UART_Transmit(&huart3, at_buff, strlen(at_buff), 0xFFFF);
	memset(at_buff, '\0', AT_BUFF_SIZE);
	HAL_Delay(2000);

	uint16_t binary_size = 15420;
	const uint16_t chunk_size = 512;
	uint16_t total_chunks = binary_size / chunk_size;
	uint16_t remainder = binary_size % chunk_size;

	if (remainder != 0) {
		total_chunks += 1;
		printf("Total chunks: %d (%d complete and %d remaining bytes.\r\n",
				total_chunks, total_chunks - 1, remainder);
	} else {
		printf("Total chunks: %d (exactly, without remaining bytes).\r\n",
				total_chunks);
	}

	uint16_t from = 0;
	uint16_t to = from + chunk_size;

	for (int i = 0; i < total_chunks; i++) {

		printf("Chunk [%d/%d] (from byte %d to byte %d)\r\n", i + 1,
				total_chunks, from, to);
		sprintf(at_buff,
				"AT+HTTPPARA=\"URL\",\"http://30bed96aa41a.ngrok.io/fota/chunks/bytes/%d-%d\"\r\n",
				from, to);

		if (i == (total_chunks - 2)) {
			from = to;
			to = to + remainder;
		} else {
			from = to;
			to = to + chunk_size;
		}

		HAL_UART_Transmit(&huart3, at_buff, strlen(at_buff), 0xFFFF);
		memset(at_buff, '\0', AT_BUFF_SIZE);
		HAL_Delay(1000);

		sprintf(at_buff, "AT+HTTPACTION=0\r\n");
		HAL_UART_Transmit(&huart3, at_buff, strlen(at_buff), 0xFFFF);
		memset(at_buff, '\0', AT_BUFF_SIZE);
		HAL_Delay(1000);

		sprintf(at_buff, "AT+HTTPREAD\r\n");
		HAL_UART_Transmit(&huart3, at_buff, strlen(at_buff), 0xFFFF);
		memset(at_buff, '\0', AT_BUFF_SIZE);
		HAL_Delay(1000);
		printf("=============================================\r\n");
	}

	printf("Performing self-reset in 2 seconds...\r\n");
	HAL_Delay(2000);
	HAL_NVIC_SystemReset();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		printf("Hi from application!\r\n");
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(1500);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_USART3;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == GPIO_PIN_13) {
		printf("Blue pushbutton called.\r\n");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *husart) {
	if (husart->Instance == USART3) {
		if (single_char != '\n' && index_counter < sizeof(one_line) - 1) {
			one_line[index_counter] = single_char;
			index_counter++;
		} else {
			one_line[index_counter] = '\0';
			printf("%s ", one_line);
			index_counter = 0;
		}
		HAL_UART_Receive_IT(&huart3, &single_char, 1);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
