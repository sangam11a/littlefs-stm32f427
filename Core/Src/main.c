/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdarg.h"
#include "lfs_util.h"
#include "lfs.h"
#include "MT25Q.h"
#include "nor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define CDC_USB_DEBUG
#define UART_DEBUG
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define sector_size 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void myprintf(const char *fmt, ...);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//uint8_t tx[]={'S','A','N','G','A','M'};

uint16_t address = 0x00;
uint8_t status_reg=0;
uint8_t READ_FLAG=0;
int tx[70];

uint8_t DEBUG_DATA_RX_FLAG = 0;
// variables used by the filesystem
typedef struct{
	uint32_t secCount;
	uint32_t bootCount;
}app_count_t;
	lfs_file_t File;
		char Text[20];
		app_count_t Counter = {0};
		lfs_t Lfs;
		nor_t Nor;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART7_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to read data from a file in LittleFS
void read_file_from_littlefs(lfs_t *lfs, const char *filename) {
    lfs_file_t file;
    HAL_UART_Transmit(&huart7, filename,sizeof(filename),1000);
    // Open the file for reading
    int err = lfs_file_open(lfs, &file, filename, LFS_O_RDONLY);
    if (err < 0) {
        printf("Failed to open file: %s\n", filename);
        return;
    }

    // Get the file size
    lfs_soff_t file_size = lfs_file_size(lfs, &file);
    if (file_size < 0) {
        printf("Failed to get file size for: %s\n", filename);
        lfs_file_close(lfs, &file);
        return;
    }

    // Allocate a buffer to hold the file data
    char *buffer = malloc(file_size);
    if (buffer == NULL) {
        printf("Failed to allocate buffer for reading file: %s\n", filename);
        lfs_file_close(lfs, &file);
        return;
    }

    // Read the file content into the buffer
    lfs_ssize_t bytes_read = lfs_file_read(lfs, &file, buffer, file_size);
    if (bytes_read < 0) {
        printf("Failed to read file: %s\n", filename);
    } else {
    	char x;
        // Successfully read the file, print its content (if it's text data)
        HAL_UART_Transmit(&huart7, buffer, (int)bytes_read,1000);
        for(int i=0;i<(int) bytes_read;){
        	printf(buffer[i]);
        	x=buffer[i];
        	i++;
        }


        printf("File Content (%s):\n%.*s\n", filename, (int)bytes_read, buffer);
    }

    // Clean up
    free(buffer);
    lfs_file_close(lfs, &file);
}

volatile uint8_t DmaEnd = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	DmaEnd = 1;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	DmaEnd = 1;
}

void nor_delay_us(uint32_t us){
//	if (us >= __HAL_TIM_GET_AUTORELOAD(&htim2)){
//		us = __HAL_TIM_GET_AUTORELOAD(&htim2) - 1;
//	}
//	__HAL_TIM_SET_COUNTER(&htim2, 0);
//	HAL_TIM_Base_Start(&htim2);
//	while (__HAL_TIM_GET_COUNTER(&htim2) < us);
//	HAL_TIM_Base_Stop(&htim2);
	HAL_Delay(1000);
}

void nor_cs_assert(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
}

void nor_cs_deassert(){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
}

void nor_spi_tx(uint8_t *pData, uint32_t Size){
//	HAL_SPI_Transmit(&hspi3, pData, Size, 100);
	DmaEnd = 0;
	HAL_SPI_Transmit(&hspi3, pData, Size, 1000);
//	while (DmaEnd == 0);
}

void nor_spi_rx(uint8_t *pData, uint32_t Size){
//	HAL_SPI_Receive(&hspi3, pData, Size, 100);
	DmaEnd = 0;
	HAL_SPI_Receive(&hspi3, pData, Size, 1000);
//	DmaEnd =0;
//	while (DmaEnd == 0);
}

void __init_nor(){
	Nor.config.CsAssert = nor_cs_assert;
	Nor.config.CsDeassert = nor_cs_deassert;
	Nor.config.DelayUs = nor_delay_us;
	Nor.config.SpiRxFxn = nor_spi_rx;
	Nor.config.SpiTxFxn = nor_spi_tx;

	if (NOR_Init(&Nor) != NOR_OK){ //NOR_Init
		Error_Handler();
	}
}

/** Start LittleFs **/

int _fs_read(const struct lfs_config *c, lfs_block_t block,
            lfs_off_t off, void *buffer, lfs_size_t size){

	if (NOR_ReadSector(&Nor, (uint8_t*)buffer, block, off, size) == NOR_OK){
		return 0;
	}

	return LFS_ERR_IO;
}

int _fs_write(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size){

	if (NOR_WriteSector(&Nor, (uint8_t*)buffer, block, off, size) == NOR_OK){
		return 0;
	}

	return LFS_ERR_IO;
}

int _fs_erase(const struct lfs_config *c, lfs_block_t block){
	if (NOR_EraseSector(&Nor, block) == NOR_OK){
		return 0;
	}

	return LFS_ERR_IO;
}

int _fs_sync(const struct lfs_config *c){
	return 0;
}

// Function to list all files and directories in the filesystem
void list_files(lfs_t *lfs) {
    lfs_dir_t dir;
    struct lfs_info info;

    // Open the root directory
    int err = lfs_dir_open(lfs, &dir, "/");
    if (err) {
        printf("Failed to open directory\n");
        return;
    }

    // Loop through all files in the directory
    while (true) {
        err = lfs_dir_read(lfs, &dir, &info);
        if (err < 0) {
            printf("Failed to read directory\n");
            break;
        }

        // If no more files, break
        if (err == 0) {
            break;
        }
        uint8_t dir[100];
        // Print the type and name of the file
        if (info.type == LFS_TYPE_REG) {
            sprintf(dir,"File: %s\n\0", info.name);
            HAL_UART_Transmit(&huart7, dir, strlen(dir),1000);
        } else if (info.type == LFS_TYPE_DIR) {
        	sprintf(dir,"Directory: %s\n\0", info.name);

            HAL_UART_Transmit(&huart7, dir, strlen(dir),1000);
        }
    }

    // Close the directory
    lfs_dir_close(lfs, &dir);
}

void __init_littefs(){
	// because of static qualifier, this variable
	// will have a dedicated address
	static struct lfs_config LfsConfig = {0};
		int Error;

		LfsConfig.read_size = 256;
		LfsConfig.prog_size = 256;
		LfsConfig.block_size = Nor.info.u16SectorSize;
		LfsConfig.block_count = Nor.info.u32SectorCount;
		LfsConfig.cache_size = Nor.info.u16PageSize;
		LfsConfig.lookahead_size = 8;//Nor.info.u32SectorCount/8;
		LfsConfig.block_cycles = 15000;

		LfsConfig.read = _fs_read;
		LfsConfig.prog = _fs_write;
		LfsConfig.erase = _fs_erase;
		LfsConfig.sync = _fs_sync;

		Error = lfs_mount(&Lfs, &LfsConfig);
		if (Error != LFS_ERR_OK){
			lfs_format(&Lfs, &LfsConfig);
			Error = lfs_mount(&Lfs, &LfsConfig);
			if (Error != LFS_ERR_OK){
				Error_Handler();
			}
		}

}


#define PATH_MAX_LEN 256

// Recursive function to list files and directories with full paths
void list_files_with_size(lfs_t *lfs, const char *path) {
    lfs_dir_t dir;
    struct lfs_info info;

    // Open the directory at the given path
    int err = lfs_dir_open(lfs, &dir, path);
    if (err) {
        printf("Failed to open directory: %s\n", path);
        return;
    }

    // Loop through all files in the directory
    while (true) {
        err = lfs_dir_read(lfs, &dir, &info);
        if (err < 0) {
            printf("Failed to read directory: %s\n", path);
            break;
        }

        // If no more files, break
        if (err == 0) {
            break;
        }

        // Build the full path for the current file/directory
        char full_path[PATH_MAX_LEN];
        snprintf(full_path, sizeof(full_path), "%s/%s", path, info.name);
        char pa[500];
        // Check if the entry is a file or directory
        if (info.type == LFS_TYPE_REG) {
            sprintf(pa,"File: %s, Size: %ld bytes\n", full_path, info.size);
            HAL_UART_Transmit(&huart7, pa, strlen(pa),1000);
        } else if (info.type == LFS_TYPE_DIR && strcmp(info.name, ".") != 0 && strcmp(info.name, "..") != 0) {
            sprintf(pa,"Directory: %s\n", full_path);
            HAL_UART_Transmit(&huart7, pa, strlen(pa),1000);
            // Recursively list the contents of the directory
            list_files_with_size(lfs, full_path);
        }
    }

    // Close the directory
    lfs_dir_close(lfs, &dir);
}
void __init_storage(){
	__init_nor();
	__init_littefs();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_UART7_Init();
  MX_USB_DEVICE_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
//simple
//  //First erase flash memory
//  	Sector_Erase_4B(&hspi3, address, sector_size);
//	//id read
//	Read_ID(&hspi3, &dev_id);
//
//	HAL_Delay(100);

	//status_reg = Status_Reg(&hspi3);
	//  int add = 0;
	//  for(int i=0;i<300;i++){
	//	  Sector_Erase_4B(&hspi3, add, 64);
	//	  add+=65536;
	//  }

	  // myprintf("Starting LittleFS application........\n");
    HAL_Delay(100);

  HAL_UART_Transmit(&huart7,"EPDM is starting *********\n", sizeof("EPDM is starting *********\n"),1000);

  HAL_UART_Transmit(&huart7,"Chip erase starting....\n", sizeof("Chip erase starting....\n"),1000);
//  Chip_Erase(&hspi3);

  HAL_UART_Transmit(&huart7,"Chip erase ending....\n", sizeof("Chip erase ending....\n"),1000);
  HAL_UART_Transmit(&huart7,"Chip erase ending....\n", sizeof("Chip erase ending....\n"),1000);

  __init_storage();
  list_files(&Lfs);
//  char path[200];
  char txt[]="sangam is writing it manually";
  list_files_with_size(&Lfs, "/");
//  		  lfs_file_open(&Lfs, &File, "satHealth.txt", LFS_O_RDWR  |LFS_O_APPEND);
//  		  lfs_file_write(&Lfs, &File, &txt, sizeof(txt));
//  		  lfs_file_close(&Lfs, &File);
  read_file_from_littlefs(&Lfs, "satHealth.txt");
//  read_file_from_littlefs(&Lfs, "sat_health.txt");
  read_file_from_littlefs(&Lfs, "flags.txt");
//  read_file_from_littlefs(&Lfs, "epdm.txt");
//   lfs_file_open(&Lfs, &File, "/sat_health.txt", LFS_O_RDWR );
////   lfs_file_read(&Lfs, &File, &Counter, sizeof(app_count_t));
//
//   lfs_file_read(&Lfs, &File, &tx, sizeof(tx));
//   HAL_UART_Transmit(&huart7,tx,strlen(tx),1000);
//   lfs_file_close(&Lfs, &File);
//
//   lfs_file_open(&Lfs, &File, "flags.txt", LFS_O_RDWR );
//  //   lfs_file_read(&Lfs, &File, &Counter, sizeof(app_count_t));
//
//     lfs_file_read(&Lfs, &File, &tx, sizeof(tx));
//     HAL_UART_Transmit(&huart7,tx,strlen(tx),1000);
//     lfs_file_close(&Lfs, &File);
//
//     lfs_file_open(&Lfs, &File, "epdm.txt", LFS_O_RDWR );
//    //   lfs_file_read(&Lfs, &File, &Counter, sizeof(app_count_t));
//
//       lfs_file_read(&Lfs, &File, &tx, sizeof(tx));
//       HAL_UART_Transmit(&huart7,tx,strlen(tx),1000);
//       lfs_file_close(&Lfs, &File);
   Counter.bootCount += 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { sprintf(Text, "Bt %lu |Ct %lu\n", Counter.bootCount, Counter.secCount);
//	  HAL_UART_Transmit(&huart7,Text, sizeof(Text),1000);
//
//	  HAL_UART_Transmit(&huart7,"*******\n", sizeof("*******\n"),1000);

//		  lfs_file_open(&Lfs, &File, "count.txt", LFS_O_RDWR | LFS_O_CREAT |LFS_O_APPEND);
//		  lfs_file_write(&Lfs, &File, &Counter.secCount, 32);
//		  lfs_file_close(&Lfs, &File);

//		  while ((HAL_GetTick() - HalTickAux) < 1000);
//		  HAL_Delay(1000);

		  Counter.secCount += 1;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, SMSN_FM_CS_OBC_Pin|MSN_FM_MODE_Pin|EN_4V_DCDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAIN_FM_CS_GPIO_Port, MAIN_FM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, MUX_EN_Pin|MPU_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, MAG_DRDY_Pin|EN_4V_Pin|EN_3V3_COM_Pin|CS_MPU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, MAG_INT_Pin|CS_MAG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SMSN_FM_CS_OBC_Pin MSN_FM_MODE_Pin EN_4V_DCDC_Pin */
  GPIO_InitStruct.Pin = SMSN_FM_CS_OBC_Pin|MSN_FM_MODE_Pin|EN_4V_DCDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : MAIN_FM_CS_Pin */
  GPIO_InitStruct.Pin = MAIN_FM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MAIN_FM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_EN_Pin MPU_INT_Pin */
  GPIO_InitStruct.Pin = MUX_EN_Pin|MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_DRDY_Pin EN_4V_Pin EN_3V3_COM_Pin CS_MPU_Pin */
  GPIO_InitStruct.Pin = MAG_DRDY_Pin|EN_4V_Pin|EN_3V3_COM_Pin|CS_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_INT_Pin CS_MAG_Pin */
  GPIO_InitStruct.Pin = MAG_INT_Pin|CS_MAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void myprintf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    char buffer[100];
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    HAL_UART_Transmit(&huart7, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
    va_end(args);
}

int bufferSize(char *buffer) {
    int i = 0;
    while (*buffer++ != '\0')
        i++;
    return i;
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
