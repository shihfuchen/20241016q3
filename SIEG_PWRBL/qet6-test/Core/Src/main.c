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
//#include <stdio.h>
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
CRC_HandleTypeDef hcrc;

FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
#define STM32_FLASH_BASE    0x08000000   	//STM32 FLASHַ
#define FLASH_APP_ADDR		0x08008000	//APP          bank2=0x08040000
#define STM32_FLASH_TAIL		0x0807F000
#define REAL_LENGTH_ADDR		0x08008024
#define LENGTH_ADDR		0x08008020
//#define Write_Max 8  //write 8 byte
//#define NumtoWrite 2048  //write 8 byte

//uint8_t APP_RX_BUF[2048];  //
//uint8_t CRC_BUF[Write_Max];
volatile uint32_t CountAddr = FLASH_APP_ADDR; //不能在callback, everytime fdcan call interrupt will reset
//uint8_t NumtoWrite = 0; //uint8_t == 8bit == unsigned char one address one byte
volatile uint32_t APPLength = 0;  //hal_falsh_program need uint64_t
//volatile uint32_t CountRecvData = 0;
//volatile uint32_t Ram_Flag = 0x20000000;

static FLASH_EraseInitTypeDef EraseInitStruct;

volatile uint8_t load_APP_flag = 0;
volatile uint8_t standby_mode_flag = 0;
volatile uint8_t test_standby_flag = 0;
//volatile int page_num = -1;

/*uint8_t TxACK[8] = {0x41,0x43,0x4b,0,0,0,0,0};
uint8_t TxFW[8] = {2,6,0,2,0x42,0x30,0x30,0x31};
uint8_t TxHW[8] = {4,6,0,2,0x48,0,0,0x31};
uint8_t TxNoCRC[8] = {0x4e,0x6f,0x43,0x52,0x43,0,0,0};
uint8_t TxCRCerror[8] = {0x43,0x52,0x43,0x65,0x72,0x72,0x6f,0x72};
uint8_t TxIDerror[8] = {0x49,0x44,0x65,0x72,0x72,0x6f,0x72,0};
uint8_t TxCommandError[8] = {0x43,0x6d,0x64,0x65,0x72,0x72,0x6f,0x72};
uint8_t RxData[64];*/
uint8_t TxACK[8] = {200,2,0,3,0,0,0,0};
uint8_t TxFW[8] = {2,6,0,2,0x42,0,0,0x01};
uint8_t TxHW[8] = {4,6,0,2,0x48,0,0,0x01};
uint8_t TxNoCRC[8] = {230,2,1,3,0,0,0,0};
uint8_t TxCRCerror[8] = {231,2,1,3,0,0,0,0};
uint8_t TxIDerror[8] = {232,2,1,3,0,0,0,0};
uint8_t TxCommandError[8] = {233,2,1,3,0,0,0,0};
uint8_t TxGoToAPP[8] = {201,2,0,3,0,0,0,0};
uint8_t TxBL[8] = {202,2,0,3,0,0,0,0};
uint8_t Txstandby[8] = {203,2,0,0,0,0,0,0};
uint8_t TxRstOK[8] = {204,2,0,3,0,0,0,0};
uint64_t APPLengthBuffer = 0;
uint8_t RxData[64];
//uint8_t RxDataBuffer[2048];
uint32_t hand_write_app_length = 0;

FDCAN_RxHeaderTypeDef RxHeader; //CAN Bus Transmit Header uint32_t DataLength;
FDCAN_TxHeaderTypeDef TxHeader; //CAN Bus Receive Header
FDCAN_FilterTypeDef FDCAN_Fil1; //CAN Bus Filter

/* CRC 表 */
/*const unsigned int crc_table[256] = {
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
    0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
    0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
    0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
    0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
    0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
    0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
    0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
    0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
    0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
    0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
    0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
    0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
    0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
    0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
    0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
    0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
    0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
    0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
    0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
    0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
    0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
    0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
    0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
    0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
    0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
    0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
    0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
    0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
    0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040,
};*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
typedef void (*iapfun)(void);				//
iapfun jump2app;
void iap_load_app(uint32_t );			// APP
void iap_write_appbin(uint8_t *, uint16_t , uint32_t , uint32_t , uint8_t);	//
//unsigned short do_crc_table(uint8_t *, uint32_t );
void Standby(void);
/*#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_F, GPIO_PIN_15);//這兩行有加沒加都可以
	//HAL_PWREx_DisablePullUpPullDownConfig();
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
  MX_FDCAN1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  SCB->VTOR = FLASH_BASE | 0x00000000;
  __enable_irq();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  FDCAN_Fil1.IdType = FDCAN_STANDARD_ID;
  FDCAN_Fil1.FilterIndex = 0;
  FDCAN_Fil1.FilterType = FDCAN_FILTER_RANGE;
  FDCAN_Fil1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN_Fil1.FilterID1 = 0x000;
  FDCAN_Fil1.FilterID2 = 0x200;

  TxHeader.Identifier = 0x100; //id
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_FRAME_FD_NO_BRS; //FDCAN_FRAME_FD_NO_BRS; //FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN; //FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN_Fil1);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, DISABLE, DISABLE);
//  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_FDCAN_Start(&hfdcan1);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
  	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
  	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
/*	  printf("AA \n\r");
  	HAL_Delay(2000);*/
/*      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
      {
        Error_Handler();
      }*/
  	//*(volatile uint32_t*)Ram_Flag = 0xDEADDAAD;
/*	  if(*(volatile uint32_t*)Ram_Flag == 0xDEADBEEF)
	  {
		  load_APP_flag = 0;
		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxACK) != HAL_OK) //write rear will receive error message
		  {
			Error_Handler();
		  }
		  *(volatile uint32_t*)Ram_Flag = 0x00000000;
	  }
	  if(*(volatile uint32_t*)Ram_Flag == 0xDEADDEAD)
	  {
		  *(volatile uint32_t*)Ram_Flag = 0xABCDDCBA;
		  load_APP_flag = 0;
		  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxACK) != HAL_OK) //write rear will receive error message
	      {
			Error_Handler();
	      }
		  *(volatile uint32_t*)Ram_Flag = 0xABCDABCD;
		  HAL_Delay(200);
		  Standby();
	  }
	  if(load_APP_flag == 1)
	  {
		  load_APP_flag = 0;
		  iap_load_app(FLASH_APP_ADDR);
	  }
	  if(test_standby_flag == 1)
	  {
		  test_standby_flag = 0;
  		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxACK) != HAL_OK) //write rear will receive error message
		  {
		    Error_Handler();
		  }
		  HAL_Delay(1000);
		  Standby();
	  }*/
      if (READ_REG( TAMP->BKP15R) == 0xDEADBEEF)
      {
		  load_APP_flag = 0;
		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxBL) != HAL_OK) //write rear will receive error message
		  {
			Error_Handler();
		  }
		  WRITE_REG( TAMP->BKP15R, 0x00000000);
		  HAL_Delay(100);
		  //*(volatile uint32_t*)Ram_Flag = 0x00000000;

	  }
	  //else if(*(volatile uint32_t*)Ram_Flag == 0xBEEFBEEF){
	  else if (READ_REG( TAMP->BKP15R) == 0xBEEFBEEF)
	  {
		  //*(volatile uint32_t*)Ram_Flag = 0x00000000;
		  load_APP_flag = 0;
		  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
		  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, Txstandby) != HAL_OK) //write rear will receive error message
		  {
			Error_Handler();
	      }
		  //*(volatile uint32_t*)Ram_Flag = 0x00000000;
	      WRITE_REG( TAMP->BKP15R, 0x00000000);
		  HAL_Delay(100);
		  Standby();
	  }
	  //else if(load_APP_flag == 1 && *(volatile uint32_t*)Ram_Flag != 0xDEADBEEF && *(volatile uint32_t*)Ram_Flag != 0xBEEFBEEF){
	  else if (load_APP_flag == 1 && READ_REG( TAMP->BKP15R) != 0xDEADBEEF && READ_REG( TAMP->BKP15R) != 0xBEEFBEEF)
	  {
		  load_APP_flag = 0;
		  iap_load_app(FLASH_APP_ADDR);
	  }
	  else
	  {
	  }
	  //*(volatile uint32_t*)Ram_Flag = 0x00000000;
	  //HAL_Delay(1000);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV8;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 10;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcanRx1, uint32_t RxFifo0ITs)
{
	HAL_FDCAN_GetRxMessage(hfdcanRx1, FDCAN_RX_FIFO0, &RxHeader, RxData);
	switch(RxHeader.Identifier){
		case 0x01:
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxFW) != HAL_OK)
			{
			  Error_Handler();
			}
			break;
		case 0x05:
	    	load_APP_flag = 0;
	    	//RxDataBuffer[CountRecvData] = RxData;
	    	//CountRecvData += 64;
//	    	if (CountRecvData == 2048)
//	    	{
		    	iap_write_appbin(RxData, 64, CountAddr, APPLength, 8);
		    	CountAddr += 64;
		        APPLength += 64;       //RxHeader.DataLength不是64不要用這個  #define FDCAN_DLC_BYTES_64 ((uint32_t)0x0000000FU) 所以只有byte==8時才可以用
		        //CountRecvData = 0;
//	    	}
	        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxACK) != HAL_OK)
	        {
	          Error_Handler();
	        }
			break;
		case 0x06://data不足8byte我會補0
	    	load_APP_flag = 1;
	    	//RxDataBuffer[CountRecvData] = RxData;
	    	//CountRecvData += 64;
			iap_write_appbin(RxData, 64, CountAddr, APPLength, 8);//
			CountAddr += 64;
	        APPLength += 64;
	        //CountRecvData = 0;
			break;
		case 0x07:
			load_APP_flag = 1;
			break;
		case 0x111:
			if(RxData[0]==14){
				WRITE_REG( TAMP->BKP15R, 0xBEEFBEEF);
			}
			else if(RxData[0]==3){
		        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxHW) != HAL_OK)
		        {
		          Error_Handler();
		        }
			}
			else{
				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxCommandError) != HAL_OK)
				{
				  Error_Handler();
				}
			}
	        break;
		case 0x10:
	        uint32_t u32TxFifoRqst = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
	        HAL_FDCAN_AbortTxRequest(&hfdcan1, u32TxFifoRqst);
			load_APP_flag = 0;
			CountAddr = FLASH_APP_ADDR;
	        APPLength = 0;
	        //*(volatile uint32_t*)Ram_Flag = 0x00000000;
	        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxACK) != HAL_OK)
	        {
	          Error_Handler();
	        }
			break;
		default:
	        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxIDerror) != HAL_OK)
	        {
	          Error_Handler();
	        }
	        break;
	}
/*    if(RxHeader.Identifier == 0x111)// 04FF=2K 8*16*16=2048
    {
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxFW) != HAL_OK)
        {
          Error_Handler();
        }
    }

    if(RxHeader.Identifier == 0x03)// 04FF=2K 8*16*16=2048
    {
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxHW) != HAL_OK)
        {
          Error_Handler();
        }
    }

    if(RxHeader.Identifier == 0x05)// 04FF=2K 8*16*16=2048
    {
    	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
//    	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
    	load_APP_flag = 0;
    	iap_write_appbin(RxData, 64, CountAddr, APPLength, 8);
    	CountAddr += 64;
        APPLength += 64;       //RxHeader.DataLength不是64不要用這個  #define FDCAN_DLC_BYTES_64 ((uint32_t)0x0000000FU) 所以只有byte==8時才可以用
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxACK) != HAL_OK)
        {
          Error_Handler();
        }
    }
    if(RxHeader.Identifier == 0x06)//write remain data
    {
    	load_APP_flag = 1;
		iap_write_appbin(RxData, 64, CountAddr, APPLength, 8);//
		CountAddr += 64;
        APPLength += 64;
    }
	if(RxHeader.Identifier == 0x07)
	{
        load_APP_flag = 1;
	}
	if(RxHeader.Identifier == 0x10)
	{
        uint32_t u32TxFifoRqst = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
        HAL_FDCAN_AbortTxRequest(&hfdcan1, u32TxFifoRqst);
		load_APP_flag = 0;
		CountAddr = FLASH_APP_ADDR;
        APPLength = 0;
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxACK) != HAL_OK)
        {
          Error_Handler();
        }

	}*/
/*	if(RxHeader.Identifier == 0x02)
	{
		test_standby_flag = 1;
	}*/
}
void iap_load_app(uint32_t appxaddr)
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	//HAL_Delay(2000);
		uint32_t Length_Addr = *(uint32_t*)REAL_LENGTH_ADDR;//存在08008024 uint32_t代表只讀4個bytes
		//uint32_t crccrc = 0;
	if( Length_Addr != 0xFFFFFFFF ){  //  *STM32_FLASH_TAIL will error  because  is number not address 一次拿32個bit LENGTH_ADDR
		//uint32_t crc_value_2 = do_crc_table( ( uint8_t*)FLASH_APP_ADDR , Length_Addr);
		uint32_t crc_value_2 = HAL_CRC_Calculate(&hcrc, (uint32_t*)FLASH_APP_ADDR, Length_Addr)^0xFFFFFFFF;
		if( *( uint32_t*)(FLASH_APP_ADDR + Length_Addr) != crc_value_2){
/*			crccrc = *( uint32_t*)(FLASH_APP_ADDR + Length_Addr);
			if(Length_Addr == 0x3480){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
			}
			if(crccrc == 0xF313){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
			}
			if(crc_value_2  == 0xF313){
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
			}

			uint8_t test_value = 0;
			int k=0;
			while (crccrc != 0)
			{
				test_value = crccrc % 10;
				TxData[k] = test_value;
				crccrc /= 10;
			    k++;
			}
	        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
	        {
	          Error_Handler();
	        }
	        HAL_Delay(1000);
			k=0;
			while (crc_value_2 != 0)
			{
				test_value = crc_value_2 % 10;
				TxData[k] = test_value;
				crc_value_2 /= 10;
			    k++;
			}
	        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
	        {
	          Error_Handler();
	        }

			HAL_Delay(1000);
			k=0;
			while (Length_Addr != 0)
			{
				test_value = Length_Addr % 10;
				TxData[k] = test_value;
				Length_Addr /= 10;
				k++;
			}*/
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxCRCerror) != HAL_OK)
			{
			  Error_Handler();
			}
			return;
		}
		else{
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxACK) != HAL_OK) //if write rear will receive error mesage
			{
			  Error_Handler();
			}
			HAL_Delay(200);
			//if(((*(volatile uint32_t*)appxaddr)&0x2FFE0000)==0x20000000)	//
			//{
		        /* */
			jump2app=(iapfun)*(volatile uint32_t*)(appxaddr+4);//appxaddr = 0x08008000                                           //(iapfun)函數指針 (vu32*)是轉型態 *取出該地址的內容
			 /* Disable all interrupts */
			HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
			HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
			HAL_FDCAN_DeInit(&hfdcan1);
			HAL_GPIO_DeInit(GPIOF, GPIO_PIN_15);
			//HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0);
			//HAL_GPIO_DeInit(GPIOE, GPIO_PIN_1);
			HAL_DeInit();
			__disable_irq();
																											//__set_PRIMASK(1); //__disable_irq();
			/* Set the clock to the default state */
			HAL_RCC_DeInit();
			//HAL_SuspendTick();//SysTick->CTRL我認為和SysTick->CTRL一樣的意思
			/* Disable Systick timer */
			SysTick->CTRL = 0;//0: counter disabled 1: counter enabled.
			SysTick->LOAD = 0;
			SysTick->VAL = 0;
			/* Clear Interrupt Enable Register & Interrupt Pending Register */
			for (int i = 0; i < sizeof(NVIC->ICER)/sizeof(NVIC->ICER[0]); i++)
			{
				NVIC->ICER[i]=0xFFFFFFFF;
				NVIC->ICPR[i]=0xFFFFFFFF;
			}
																											  /* Re-enable all interrupts */
																									 // __enable_irq();		        //__set_PRIMASK(0);
			/*  RTOS Set the MSP*/
			__set_CONTROL(0);
			__set_MSP(*(volatile uint32_t*)appxaddr);					// Set the MSP
			jump2app();									//
			while (1)
			{
			}
			//}
		}
	}
	else{
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxNoCRC) != HAL_OK)
        {
          Error_Handler();
        }
		return;
	}

}
//NumtoWrite 有多少資料要寫入
//WriteAPPLength 目前app的長度
//WriteToFlashSize STM有兩種，一次寫入32bit跟一次寫入64bit，我是選一次寫入64bit，所以是8byte
void iap_write_appbin(uint8_t *pBuffer, uint16_t NumtoWrite, uint32_t WriteAddr, uint32_t WriteAPPLength, uint8_t WriteToFlashSize)//each byte one address
{
//	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    if (WriteAddr < STM32_FLASH_BASE || (WriteAddr >= STM32_FLASH_TAIL))
    {
    	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    	//HAL_Delay(2000);
        //printf("stack overflow\r\n");
        return;
    }
    uint32_t PageError = 0; ////设置PageError,如果出现错误这个变量会被设置为出错的FLASH地址
    //uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
    //uint32_t Address = 0, PageError = 0; ////设置PageError,如果出现错误这个变量会被设置为出错的FLASH地址
    /*One page (2 Kbytes)STM32G474  有双 Bank 功能，且默认情况下 Option Bytes 中的 DBANK =1*/
    //crc = HAL_CRC_Calculate(&hcrc, (u32 *)pBuffer, NumToWrite)^0x00000000;//word size=32bits

    HAL_FLASH_Unlock();

    if((WriteAPPLength % 2048) == 0){ //two page FLASH_PAGE_SIZE==2048
    	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    	//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
    	//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
  	    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  	    EraseInitStruct.NbPages     = 1;
        if(WriteAddr < 0x08040000){
        	//FLASH_PageErase( (WriteAddr-FLASH_APP_ADDR)/2048, FLASH_BANK_1); //FLASH_ErasePage法二 //error
  		    EraseInitStruct.Page = (WriteAddr-STM32_FLASH_BASE)/FLASH_PAGE_SIZE;
  		    EraseInitStruct.Banks = FLASH_BANK_1;
        	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
        		Error_Handler();
        	}
        }
        else{
  		    EraseInitStruct.Page = (WriteAddr-0x08040000)/FLASH_PAGE_SIZE;
  		    EraseInitStruct.Banks = FLASH_BANK_2;
        	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
        		Error_Handler();
        	}
        }
/*        if(WriteCRCAddr < 0x08040000){
        	//FLASH_PageErase((WriteCRCAddr-FLASH_APP_ADDR)/2048, FLASH_BANK_1); //crc
  		    EraseInitStruct.Page = (WriteCRCAddr-STM32_FLASH_BASE)/FLASH_PAGE_SIZE;
  		    EraseInitStruct.Banks = FLASH_BANK_1;
        	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
        		Error_Handler();
        	}
        }
        else{
        	//FLASH_PageErase((WriteCRCAddr-0x08040000)/2048, FLASH_BANK_2); //crc
  		    EraseInitStruct.Page = (WriteCRCAddr-0x08040000)/FLASH_PAGE_SIZE;//FLASH_PAGE_SIZE==2048
  		    EraseInitStruct.Banks = FLASH_BANK_2;
        	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
        		Error_Handler();
        	}
        }*/                                                                                                                 //close
    }
    //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
    uint64_t merge_pBuffer = 0;
    for(int i=0; i<NumtoWrite; i+=WriteToFlashSize){//(Address < FLASH_USER_END_ADDR) write 8 byte every time
//    	if(WriteToFlashSize==8){
    		merge_pBuffer = 0;
        	for(int j=0; j<WriteToFlashSize; j++){
        		merge_pBuffer |= (uint64_t)*(pBuffer + j);
        		if( j < (WriteToFlashSize-1) ){
        			merge_pBuffer <<= 8; //8 is bit not byte
        		}
        	}
        	merge_pBuffer = ((merge_pBuffer << 8) & (0xFF00FF00FF00FF00) ) | ((merge_pBuffer >> 8) & (0x00FF00FF00FF00FF) ); //must have add uint64_t because CPU do 32bit
        	merge_pBuffer = ((merge_pBuffer << 16) &  (0xFFFF0000FFFF0000) ) | ((merge_pBuffer >> 16) &  (0x0000FFFF0000FFFF) );
        	merge_pBuffer = (merge_pBuffer << 32) | (merge_pBuffer >> 32);
        	if(WriteAddr != LENGTH_ADDR){//保留位置寫入crc
            	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WriteAddr, merge_pBuffer) != HAL_OK)//write 8bytes *(pointer+0)=a[0]
            	{
            		Error_Handler();
            	}
        	}
        	else
        	{
        		APPLengthBuffer = merge_pBuffer;
        	}
//    	}
/*    	else{//LASH_TYPEPROGRAM_FAST用這個就不能用page erase 要用mass erase
    		uint32_t merge_pBuffer32 = 0;
        	for(int j=0; j<WriteToFlashSize; j++){
        		merge_pBuffer32 |= (uint32_t)*(pBuffer + j);
        		if( j < (WriteToFlashSize-1) ){
        			merge_pBuffer32 <<= 8; //8 is bit not byte
        		}
        	}
        	merge_pBuffer32 = ((merge_pBuffer32>>24)&0xff) | // move byte 3 to byte 0
                    ((merge_pBuffer32<<8)&0xff0000) | // move byte 1 to byte 2
                    ((merge_pBuffer32>>8)&0xff00) | // move byte 2 to byte 1
                    ((merge_pBuffer32<<24)&0xff000000); // byte 0 to byte 3
        	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, WriteAddr, merge_pBuffer32) != HAL_OK)//write 4bytes *(pointer+0)=a[0]
        	{
        		Error_Handler();
        	}
    	}*/
    	//printf("%lx \n",*(uint32_t*)WriteAddr );
/*    	merge_pBuffer = 0;
        for(int j=0; j<8; j++){
        	merge_pBuffer |= do_crc_table(pBuffer + j, 1);//sizeof(pBuffer)==array_length   send pointer
        	merge_pBuffer <<= 8;
        }
    	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WriteCRCAddr, merge_pBuffer) != HAL_OK)//write 8bytes *(pointer+0)=a[0]
    	{
    		//CRC_BUF = CRC_BUF + i;  重複使用crc_buf
    	}
*/                                                                                                                 //close
    	pBuffer = pBuffer + WriteToFlashSize;
    	WriteAddr = WriteAddr + WriteToFlashSize;
	}
	if(load_APP_flag == 1){
		WriteAPPLength = WriteAPPLength + NumtoWrite;
		//FLASH_PageErase((STM32_FLASH_TAIL-0x08040000)/2048, FLASH_BANK_2);
/*  	    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  	    EraseInitStruct.NbPages     = 1;
		EraseInitStruct.Page = 126; //page index
		EraseInitStruct.Banks = FLASH_BANK_2;
    	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
    		Error_Handler();
    	}*/
    	if(( WriteAPPLength % FLASH_PAGE_SIZE) == 0){
      	    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
      	    EraseInitStruct.NbPages     = 1;
            if(WriteAddr < 0x08040000){
            	//FLASH_PageErase( (WriteAddr-FLASH_APP_ADDR)/2048, FLASH_BANK_1); //FLASH_ErasePage法二 //error
      		    EraseInitStruct.Page = (WriteAddr - STM32_FLASH_BASE)/FLASH_PAGE_SIZE;
      		    EraseInitStruct.Banks = FLASH_BANK_1;
            	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
            		Error_Handler();
            	}
            }
            else{
      		    EraseInitStruct.Page = (WriteAddr - 0x08040000)/FLASH_PAGE_SIZE;
      		    EraseInitStruct.Banks = FLASH_BANK_2;
            	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
            		Error_Handler();
            	}
            }
    	}
    	APPLengthBuffer = APPLengthBuffer & (0x00000000FFFFFFFF);
    	APPLengthBuffer = APPLengthBuffer | ((uint64_t) WriteAPPLength) << 32;//要先移到高位才會存在08008024
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, LENGTH_ADDR, APPLengthBuffer) != HAL_OK){
			Error_Handler();//寫入的資料會自動強制轉型成uint64_t  0x08008020 LENGTH_ADDR STM32_FLASH_TAIL
		}
		//uint64_t crc_value = do_crc_table( ( uint8_t*)FLASH_APP_ADDR , WriteAPPLength);//先寫再算crc，才不會crc error
		uint64_t crc_value = HAL_CRC_Calculate(&hcrc, (uint32_t*)FLASH_APP_ADDR, WriteAPPLength)^0xFFFFFFFF;

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WriteAddr, crc_value) != HAL_OK){  //WriteAddr前面已經加了
			Error_Handler();
		}
	}
    HAL_FLASH_Lock();
}

//查表法算crc ModBus
/*unsigned short do_crc_table(uint8_t *ptr, uint32_t len)//len==array number
{
    unsigned short crc = 0xFFFF;

    while(len--)
    {
        crc = (crc >> 8) ^ crc_table[(crc ^ *ptr++) & 0xff];
    }

    return (crc);
}*/
void Standby(void){
	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_PWREx_EnablePullUpPullDownConfig();//standby mode專用，因為進standby modegpio都會設成類比
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_F, GPIO_PIN_15);
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_F, PWR_GPIO_BIT_12);
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_6);
//	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_C, GPIO_PIN_13);

	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2_LOW);
//	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
/*	  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
	  {
	    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);//Clear Standby flag
	  }*/

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);//我不用開rcc
	//HAL_PWR_EnableBkUpAccess();
	/* Clear all related wakeup flags */
//	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_LOW);
	//PWR_WakeUpPinCmd (ENABLE);
	HAL_PWR_EnterSTANDBYMode();
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
