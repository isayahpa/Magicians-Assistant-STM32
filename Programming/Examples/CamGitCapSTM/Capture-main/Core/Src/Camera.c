/*
 * Camera.c
 *
 *  Created on: Oct 17, 2022
 *      Author: Adam Prochazka
 */

#include "Camera.h"
#include "main.h"

//Initialize camera by reverse engineering demo code for weaker chip on official github
void camInit(I2C_HandleTypeDef hi2c1, SPI_HandleTypeDef hspi1){
	resetCam(hi2c1);

	wCamRegs(hi2c1, OV5642_QVGA_Preview);
	wCamRegs(hi2c1, OV5642_JPEG_Capture_QSXGA);
	wCamRegs(hi2c1, OV5642_320x240);

    wCamRegs(hi2c1, OV5642_Init_Tail);

    // TODO: consult if this is the problematic register
	wCamRegSPI(hspi1, 0x03, 0x02); // SET VSYNC POLARITY TO ACTIVE LOW
}

//try to get any capture data back from camera module
void snapPic(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2, SPI_HandleTypeDef hspi1){

	resetFifoFlags(hspi1);
	resetCapDoneFlag(hspi1);

	setCaptureCount(hspi1, 1);

	startCapture(hspi1);

	//wait for capture done
	while(1){
		uint8_t regValue = rCamSPI(hspi1, CAPTURE_DONE_REG);
		uint8_t captureDoneMask = CAPTURE_DONE_MASK;
		if(regValue & captureDoneMask) break;
	}

	uint32_t fifoLen = getFifoLen(hspi1);
	uint32_t sendLen = (fifoLen>=4096) ? 4096 : fifoLen;

	int buffSize = 4096;
	uint8_t * picbuf = malloc(buffSize * sizeof(uint8_t));
	memset(picbuf, 0, buffSize);

	spiStart();

	uint8_t BURST_FIFO_READ = 0x3c;
	HAL_SPI_TransmitReceive(&hspi1, &BURST_FIFO_READ, picbuf, 1, HAL_MAX_DELAY);

	HAL_SPI_Receive(&hspi1, picbuf, sendLen, HAL_MAX_DELAY);

	//TODO: remove delay
	//while(hspi1.State != HAL_SPI_STATE_READY){;}
	HAL_Delay(1000); //delay to ensure full dma transmission

	spiEnd();

	HAL_UART_Transmit(&huart2, picbuf, sendLen, HAL_MAX_DELAY);
};

HAL_StatusTypeDef wCamReg(I2C_HandleTypeDef hi2c1, uint16_t regID, uint16_t data){
	HAL_StatusTypeDef ret;

	uint8_t messageBuff[3];
	uint8_t Addr = I2C_DEVICE_ADDRESS;

	messageBuff[0] = regID >> 8;	// first 8 bits of the address
	messageBuff[1] = regID;			// last 8 bits of the address
	messageBuff[2] = data;			// data to send

	ret = HAL_I2C_Master_Transmit(&hi2c1, Addr, messageBuff, 3, HAL_MAX_DELAY);

	return ret;
};

HAL_StatusTypeDef wCamRegs(I2C_HandleTypeDef hi2c1, const struct sensor_reg regList[])
{

	uint16_t regID;
	uint16_t regData;
	const struct sensor_reg *nextReg = regList;

	int writeStatus;

	while ((regID != END_OF_REG_LIST_ID) | (regData != END_OF_REG_LIST_DATA))
	{
		regID =nextReg->reg;
		regData = nextReg->val;

		writeStatus = wCamReg(hi2c1, regID, regData);

		if(writeStatus != HAL_OK) return writeStatus;

	    nextReg++;
	}

	return HAL_OK;
};

void resetCam(I2C_HandleTypeDef hi2c1){
	wCamReg(hi2c1, RESET_REG, RESET_VAL);
};

HAL_StatusTypeDef wCamRegSPI(SPI_HandleTypeDef hspi1, uint8_t addr, uint8_t data){

	HAL_StatusTypeDef ret;

	// set first bite of address to 1 (write mode)
	uint8_t addrFormatted = addr | SPI_WRITE_MASK;

	uint16_t bytesToSend = 1; // we want to send just 1 Byte on each transmit

	spiStart();

	// TODO: figure out if removing this delay is safe
	HAL_Delay(1);

	// send register id
	ret = HAL_SPI_Transmit(&hspi1, (uint8_t *)&addrFormatted, bytesToSend, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return ret;
	}

	// send new data to register
	ret = HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, bytesToSend, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return ret;
	}

	spiEnd();

	return HAL_OK;
};

//	read spi register
uint8_t rCamSPI(SPI_HandleTypeDef hspi1, uint8_t addr){

	uint8_t addrMasked = addr & SPI_READ_MASK;

	uint8_t emptyData = 0;
	uint8_t	dataFromReg = 0;
	uint16_t bytesToReceive = sizeof(dataFromReg); // 1 Byte

	spiStart();

	HAL_SPI_TransmitReceive(&hspi1, &addrMasked, &dataFromReg, bytesToReceive, HAL_MAX_DELAY);

	HAL_SPI_TransmitReceive(&hspi1, &emptyData, &dataFromReg, bytesToReceive, HAL_MAX_DELAY);

	spiEnd();

	return dataFromReg;
}

void spiStart(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
};

void spiEnd(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
};

void resetFifoFlags(SPI_HandleTypeDef hspi1){
	wCamRegSPI(hspi1, FIFO_REG, FIFO_POINTER_RESET);
};

void resetCapDoneFlag(SPI_HandleTypeDef hspi1){
	wCamRegSPI(hspi1, FIFO_REG, FIFO_WRITE_DONE_CLEAR);
};

void setCaptureCount(SPI_HandleTypeDef hspi1, uint8_t captureCount){
	wCamRegSPI(hspi1, CAPTURE_REG, captureCount);
};

void startCapture(SPI_HandleTypeDef hspi1){
	wCamRegSPI(hspi1, FIFO_REG, START_CAPTURE);
};

uint32_t getFifoLen(SPI_HandleTypeDef hspi1){
	uint32_t len1,len2,len3,len=0;

	len1 = rCamSPI(hspi1, FIFO_SIZE_7_0);
	len2 = rCamSPI(hspi1, FIFO_SIZE_15_8);
	len3 = rCamSPI(hspi1, FIFO_SIZE_22_16) & BIT_MASK_7;

	len = ((len3 << 16) | (len2 << 8) | len1) & BIT_MASK_23;

	return len;
};


