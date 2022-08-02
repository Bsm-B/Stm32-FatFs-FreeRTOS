/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions
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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TSC_HandleTypeDef htsc;

UART_HandleTypeDef huart4;

PCD_HandleTypeDef hpcd_USB_FS;

osThreadId BlinkTaskHandle;
osThreadId SDinfoHandle;
osThreadId SDManagerHandle;
osMutexId SDCardMutexHandle;
/* USER CODE BEGIN PV */
FATFS FatFs;   // FATFS handle
FRESULT fres;  // Common result code
FILINFO fno;	  // Structure holds information
FATFS *getFreeFs; 	  // Read information
FIL fil;
DIR dir;			  // Directory object structure
DWORD free_clusters;  // Free Clusters
DWORD free_sectors;	  // Free Sectors
DWORD total_sectors;  // Total Sectors
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART4_UART_Init(void);
void StartBlinkTask(void const *argument);
void StartSDinfo(void const *argument);
void StartSDManager(void const *argument);

/* USER CODE BEGIN PFP */
void ls(char*);
void dmesg(FRESULT);
void printk(const char*, ...);
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
	MX_I2C2_Init();
	MX_SPI2_Init();
	MX_TSC_Init();
	MX_USB_PCD_Init();
	MX_SPI1_Init();
	MX_USART4_UART_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of SDCardMutex */
	osMutexDef(SDCardMutex);
	SDCardMutexHandle = osMutexCreate(osMutex(SDCardMutex));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of BlinkTask */
	osThreadDef(BlinkTask, StartBlinkTask, osPriorityNormal, 0, 128);
	BlinkTaskHandle = osThreadCreate(osThread(BlinkTask), NULL);

	/* definition and creation of SDinfo */
	osThreadDef(SDinfo, StartSDinfo, osPriorityHigh, 0, 140);
	SDinfoHandle = osThreadCreate(osThread(SDinfo), NULL);

	/* definition and creation of SDManager */
	osThreadDef(SDManager, StartSDManager, osPriorityRealtime, 0, 300);
	SDManagerHandle = osThreadCreate(osThread(SDManager), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x20303E5D;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TSC Initialization Function
 * @param None
 * @retval None
 */
static void MX_TSC_Init(void) {

	/* USER CODE BEGIN TSC_Init 0 */

	/* USER CODE END TSC_Init 0 */

	/* USER CODE BEGIN TSC_Init 1 */

	/* USER CODE END TSC_Init 1 */
	/** Configure the TSC peripheral
	 */
	htsc.Instance = TSC;
	htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
	htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
	htsc.Init.SpreadSpectrum = DISABLE;
	htsc.Init.SpreadSpectrumDeviation = 1;
	htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
	htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
	htsc.Init.MaxCountValue = TSC_MCV_8191;
	htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
	htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
	htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
	htsc.Init.MaxCountInterrupt = DISABLE;
	htsc.Init.ChannelIOs = TSC_GROUP1_IO3 | TSC_GROUP2_IO3 | TSC_GROUP3_IO2;
	htsc.Init.ShieldIOs = 0;
	htsc.Init.SamplingIOs = TSC_GROUP1_IO4 | TSC_GROUP2_IO4 | TSC_GROUP3_IO3;
	if (HAL_TSC_Init(&htsc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TSC_Init 2 */

	/* USER CODE END TSC_Init 2 */

}

/**
 * @brief USART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART4_UART_Init(void) {

	/* USER CODE BEGIN USART4_Init 0 */

	/* USER CODE END USART4_Init 0 */

	/* USER CODE BEGIN USART4_Init 1 */

	/* USER CODE END USART4_Init 1 */
	huart4.Instance = USART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART4_Init 2 */

	/* USER CODE END USART4_Init 2 */

}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void) {

	/* USER CODE BEGIN USB_Init 0 */

	/* USER CODE END USB_Init 0 */

	/* USER CODE BEGIN USB_Init 1 */

	/* USER CODE END USB_Init 1 */
	hpcd_USB_FS.Instance = USB;
	hpcd_USB_FS.Init.dev_endpoints = 8;
	hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_Init 2 */

	/* USER CODE END USB_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
	NCS_MEMS_SPI_Pin | EXT_RESET_Pin | LD3_Pin | LD6_Pin | LD4_Pin | LD5_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
	 LD4_Pin LD5_Pin */
	GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin | EXT_RESET_Pin | LD3_Pin | LD6_Pin
			| LD4_Pin | LD5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT1_Pin | MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void printk(const char *fmt, ...) {
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart4, (uint8_t*) buffer, len, -1);

}

void ls(char *path) {

	printk("Files/Folder List:\n");
	fres = f_opendir(&dir, path);

	if (fres == FR_OK) {
		while (1) {

			fres = f_readdir(&dir, &fno);

			if ((fres != FR_OK) || (fno.fname[0] == 0)) {
				break;
			}
			printk("\n %c%c%c%c%c %u-%02u-%02u, %02u:%02u %10d %s/%s",
					((fno.fattrib & AM_DIR) ? 'D' : '-'),
					((fno.fattrib & AM_RDO) ? 'R' : '-'),
					((fno.fattrib & AM_SYS) ? 'S' : '-'),
					((fno.fattrib & AM_HID) ? 'H' : '-'),
					((fno.fattrib & AM_ARC) ? 'A' : '-'),
					((fno.fdate >> 9) + 1980), (fno.fdate >> 5 & 15),
					(fno.fdate & 31), (fno.ftime >> 11), (fno.ftime >> 5 & 63),
					(int) fno.fsize, path, fno.fname);
		}
	}

}

void dmesg(FRESULT fres) {

	switch (fres) {
	case FR_OK:
		printk("Succeeded \n");
		break;
	case FR_DISK_ERR:
		printk("A hard error occurred in the low level disk I/O layer \n");
		break;
	case FR_INT_ERR:
		printk("Assertion failed \n");
		break;
	case FR_NOT_READY:
		printk("The physical drive cannot work");
		break;
	case FR_NO_FILE:
		printk("Could not find the file \n");
		break;
	case FR_NO_PATH:
		printk("Could not find the path \n");
		break;
	case FR_INVALID_NAME:
		printk("The path name format is invalid \n");
		break;
	case FR_DENIED:
		printk("Access denied due to prohibited access or directory full \n");
		break;
	case FR_EXIST:
		printk("Exist or access denied due to prohibited access \n");
		break;
	case FR_INVALID_OBJECT:
		printk("The file/directory object is invalid \n");
		break;
	case FR_WRITE_PROTECTED:
		printk("The physical drive is write protected \n");
		break;
	case FR_INVALID_DRIVE:
		printk("The logical drive number is invalid \n");
		break;
	case FR_NOT_ENABLED:
		printk("The volume has no work area");
		break;
	case FR_NO_FILESYSTEM:
		printk("There is no valid FAT volume");
		break;
	case FR_MKFS_ABORTED:
		printk("The f_mkfs() aborted due to any parameter error \n");
		break;
	case FR_TIMEOUT:
		printk(
				"Could not get a grant to access the volume within defined period \n");
		break;
	case FR_LOCKED:
		printk(
				"The operation is rejected according to the file sharing policy \n");
		break;
	case FR_NOT_ENOUGH_CORE:
		printk("LFN working buffer could not be allocated \n");
		break;
	case FR_TOO_MANY_OPEN_FILES:
		printk("Number of open files > _FS_SHARE \n");
		break;
	case FR_INVALID_PARAMETER:
		printk("Given parameter is invalid \n");
		break;
	default:
		printk("An error occured. (%d)\n", fres);
	}

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlinkTask */
/**
 * @brief  Function implementing the BlinkTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlinkTask */
void StartBlinkTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		printk("\nStartBlinkTask!\n");
		HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
		printk("\n\[Blink]: LED Blink\n");
		osDelay(1000);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSDinfo */
/**
 * @brief Provides the general state of the volume with the files in the root
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSDinfo */
void StartSDinfo(void const *argument) {
	/* USER CODE BEGIN StartSDinfo */
	/* Infinite loop */
	for (;;) {
		printk("\nStartSDinfoTask!\n");
		osMutexWait(SDCardMutexHandle, portMAX_DELAY);
		printk("\n[SDinfo]: Mount SDCard");
		fres = f_mount(&FatFs, "", 1); // 1 -> Mount now
		if (fres == FR_OK) {
			// Get some statistics from the SD card
			fres = f_getfree("", &free_clusters, &getFreeFs);
			// Formula comes from ChaN's documentation
			total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
			free_sectors = free_clusters * getFreeFs->csize;
			printk(
					"\n[SDinfo]: SD Card Status:\r\n\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n",
					total_sectors / 2, free_sectors / 2);
			osDelay(100);
			// List files and folder in root
			printk("\n[SDinfo]: ");
			ls("");
		} else {
			printk("\n[SDinfo]: ");
			dmesg(fres);
		}
		f_mount(NULL, "", 0);
		printk("\n[SDinfo]: Unmount SDCard");
		osMutexRelease(SDCardMutexHandle);
		osDelay(1200);
	}
	/* USER CODE END StartSDinfo */
}

/* USER CODE BEGIN Header_StartSDManager */
/**
 * @brief Doing some operations at SD card (read, write, rename...)
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSDManager */
void StartSDManager(void const *argument) {
	/* USER CODE BEGIN StartSDManager */
	/* Infinite loop */
	for (;;) {
		printk("\nStartSDManagerTask!\n");
		osMutexWait(SDCardMutexHandle, portMAX_DELAY);
		printk("\n[SDManager]: Mount SDCard");
		fres = f_mount(&FatFs, "", 1); // 1 -> Mount now
		if (fres == FR_OK) {

			// Create Folder
			fres = f_mkdir("DEMO");
			if (fres == FR_OK) {
				printk("\n[SDManager]: Create Folder");
			} else {
				printk("\n[SDManager][Create_Folder]: ");
				dmesg(fres);
			}

			BYTE readBuf[30];

			// Read File
			fres = f_open(&fil, "test.txt", FA_OPEN_ALWAYS);
			if (fres == FR_OK) {

				TCHAR *rres = f_gets((TCHAR*) readBuf, 30, &fil);
				if (rres != 0) {
					printk("\n[SDManager]: Read File");
					printk("\n[SDManager]: 'test.txt' contents: %s\r\n",
							readBuf);

				}

			} else {

				printk("\n[SDManager][Read_File]: ");
				dmesg(fres);

			}
			f_close(&fil);

			osDelay(100);

			// Write File
			fres = f_open(&fil, "/DEMO/write.txt",
			FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
			if (fres == FR_OK) {
				//Copy in a string
				strncpy((char*) readBuf, "I hate Java!", 12);
				UINT bytesWrote;
				fres = f_write(&fil, readBuf, 12, &bytesWrote);
				if (fres == FR_OK) {
					printk("\n[SDManager]: Write File");
					printk("\n[SDManager]: Wrote %i bytes to 'write.txt'!\r\n",
							bytesWrote);
				} else {
					printk("\n[SDManager][Write_File]: ");
					dmesg(fres);
				}
			} else {
				printk("\n[SDManager][Create_File]: ");
				dmesg(fres);
			}
			f_close(&fil);

		} else {
			printk("\n[SDManager]: ");
			dmesg(fres);
		}

		f_mount(NULL, "", 0);
		printk("\n[SDManager]: Unmount SDCard");
		osMutexRelease(SDCardMutexHandle);
		osDelay(1000);
	}

	/* USER CODE END StartSDManager */
}

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
