/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cstdio"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TIMEOUT 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int checkBit(uint8_t num, int index){
	return (int) ((num >> index) & 1);
}

void serialPrint(uint8_t* msg, uint16_t size){
	HAL_UART_Transmit(&huart2, msg, size, TIMEOUT);
}

struct ArducamController{
	// The chip select signal should always be LOW during the SPI read or write bus cycle
	//https://www.uctronics.com/download/Amazon/ArduCAM_Mini_2MP_Camera_Shield_Hardware_Application_Note.pdf
	//I2C Settings Registers
	uint8_t I2C_ADDR_WRITE = 0x60;
	uint8_t I2C_ADDR_READ = 0x61;
	uint8_t TEST_REGISTER = 0x00;
	uint8_t CAPTURE_CONTROL_REG = 0x01; // Bits[2:0] = number of frames to be captured
	uint8_t SENSOR_INTERFACE_TIMING_REG = 0x03; // Bit[0]: Sensor H-sync Polarity, 0 = active high, 1 = active low,
												// Bit[1]: Sensor V-sync Polarity 0 = active high, 1 = active low
												// Bit[3]: Sensor data delay 0 = no delay, 1= delay 1 PCLK
												// Bit[4]: FIFO mode control, 0 = FIFO mode disable, 1 = enable FIFO mode
												// Bit[6]: low power mode control, 0 = normal mode, 1 = low power mode
	uint8_t FIFO_CONTROL_REG = 0x04; // Bit[0]: ‘1’ - clear FIFO write done flag, Bit[1]: write ‘1’ to start capture
									 // Bit[4]: write ‘1’ to reset FIFO write pointer Bit[5]: write ‘1’ to reset FIFO read pointer
	uint8_t FIFO_SINGLE_READ_REG = 0x3D; //RO
	uint8_t FIFO_STATUS_REG = 0x41; // Bit[0]: camera v-sync pin status, Bit[3]: camera write FIFO done flag
	uint8_t FIFO_BYTE0 = 0x42;
	uint8_t FIFO_BYTE1 = 0x43;
	uint8_t FIFO_BYTE2 = 0x44;

	uint8_t FIFO_FLAG_CLR = 0b00000001;
	uint8_t FIFO_PTR_CLR = 0b00110000;
	uint8_t SET_CAPTURE_FLAG = 0b00000010;

	//SPI Settings
	int SPI_CLK_HZ = 80000;
	I2C_HandleTypeDef* i2cHandle;
	SPI_HandleTypeDef* spiHandle;

public:
	ArducamController(I2C_HandleTypeDef* pHandleI2C, SPI_HandleTypeDef* pHandleSPI){
		i2cHandle = pHandleI2C;
		spiHandle = pHandleSPI;
		initSensor();
	}

	void initSensor(){
		printf("Initializing ArduCam");
		HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET); //CS LOW to configure
		uint8_t cmd = 0b00000001; //Capture 1 Frame per Capture

		resetFIFOPointers();
		clearFIFOFlag();
		i2cRegWrite(CAPTURE_CONTROL_REG, &cmd, 1);
		HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET); //CS HIGH when finished

	}

	/*To Write over i2c:
	* Request I2C_ADDR_WRITE -> Send Cam Register Address (left shift device addr's by 1) -> Send Data bytes
	*/
	HAL_StatusTypeDef i2cRegWrite(uint8_t reg, uint8_t *pData, uint16_t size){
		HAL_StatusTypeDef status;
		status = HAL_I2C_Master_Transmit(i2cHandle, I2C_ADDR_WRITE<<1, &reg, 1, TIMEOUT);
		status = HAL_I2C_Master_Transmit(i2cHandle, I2C_ADDR_WRITE<<1, pData, size, TIMEOUT);
		return status;
	}

	/*To Read over i2c:
		* To I2C_ADDR_WRITE: Write the Register You want to read from
		* To I2C_ADDR_READ: Read as much data as you want
	*/
	HAL_StatusTypeDef i2cRegRead(uint8_t reg, uint8_t *pBuffer, uint16_t size){
		HAL_StatusTypeDef status;
		status = HAL_I2C_Master_Transmit(i2cHandle, I2C_ADDR_WRITE<<1, &reg, 1, TIMEOUT);
		status = HAL_I2C_Master_Receive(i2cHandle, I2C_ADDR_READ<<1, pBuffer, size, TIMEOUT);
		return status;
	}

	HAL_StatusTypeDef spiRegWrite(uint8_t reg, uint8_t *pData, uint16_t size){
		HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET); // CS Pin Set LOW

		HAL_StatusTypeDef status;
		uint8_t cmdByte = 0x80 | reg; // a 1 followed by reg addr, to write to reg

		status = HAL_SPI_Transmit(spiHandle, &cmdByte, 1, TIMEOUT);
		status = HAL_SPI_Transmit(spiHandle, pData, size, TIMEOUT);
		return status;
	}

	HAL_StatusTypeDef spiRegRead(uint8_t reg, uint8_t *pBuffer, uint16_t size){
		HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET); // CS Pin Set LOW

		HAL_StatusTypeDef status;
		uint8_t cmdByte = 0x00 | reg; // a 0 followed by register to read

		status = HAL_SPI_Transmit(spiHandle, &cmdByte, 1, TIMEOUT);
		status = HAL_SPI_Receive(spiHandle, pBuffer, size, TIMEOUT);
		return status;

	}

	bool isFIFOBusy(){
		uint8_t data = 0x00;
		i2cRegRead(FIFO_STATUS_REG, &data, 1);
		return checkBit(data, 3);
	}

	void readFrameBuffer(uint8_t *buffer){

		if(isFIFOBusy()){
			printf("Can't read, FIFO is busy...");
		} else {
			i2cRegRead(FIFO_BYTE0, &buffer[0], 1);
			i2cRegRead(FIFO_BYTE1, &buffer[1], 1);
			i2cRegRead(FIFO_BYTE2, &buffer[2], 1);
		}

	}

	void singleCapture(){
		uint8_t cmd;
		if(isFIFOBusy()){
			printf("Can't Capture, FIFO is busy...");
		} else {
			cmd = SET_CAPTURE_FLAG;
			i2cRegWrite(FIFO_CONTROL_REG, &cmd, 1);
		}
	}

	void flushFIFO(){
		clearFIFOFlag();
		resetFIFOPointers();
	}

	void clearFIFOFlag(){
		uint8_t cmd = FIFO_FLAG_CLR;
		i2cRegWrite(FIFO_CONTROL_REG, &cmd, 1);
	}

	void resetFIFOPointers(){
		uint8_t cmd = FIFO_PTR_CLR;
		i2cRegWrite(FIFO_CONTROL_REG, &cmd, 1);
	}
};

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  ArducamController arducam(&hi2c1, &hspi1); //Initialize the Arducam
  arducam.singleCapture();
  uint8_t buffer[6] = "apple";
  arducam.readFrameBuffer(buffer);
  serialPrint(buffer, sizeof(buffer));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM_CS_Pin */
  GPIO_InitStruct.Pin = CAM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAM_CS_GPIO_Port, &GPIO_InitStruct);

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
