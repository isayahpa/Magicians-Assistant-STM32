/*
 * ArducamController.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Isayah Parent
 */

#ifndef SRC_ARDUCAMCONTROLLER_H_
#define SRC_ARDUCAMCONTROLLER_H_

#include "stm32l4xx_hal.h"
//#include <stdio.h>
#include "helpers.h"

// Register Addresses
//https://www.uctronics.com/download/Amazon/ArduCAM_Mini_2MP_Camera_Shield_Hardware_Application_Note.pdf
#define I2C_ADDR_WRITE 0x60
#define I2C_ADDR_READ 0x61
#define TEST_REGISTER 0x00
#define CAPTURE_CONTROL_REG 0x01
#define SENSOR_INTERFACE_TIMING_REG 0x03
#define FIFO_CONTROL_REG 0x04
#define FIFO_SINGLE_READ_REG 0x3D
#define FIFO_STATUS_REG 0x41
#define FIFO_BYTE0 0x42
#define FIFO_BYTE1 0x43
#define FIFO_BYTE2 0x44
#define FIFO_FLAG_CLR 0b00000001
#define FIFO_PTR_CLR 0b00110000
#define	SET_CAPTURE_FLAG 0b00000010

#define SPI_CLK_HZ 800000
#define TIMEOUT 100

typedef struct ArducamController ArducamController;

struct ArducamController{
	I2C_HandleTypeDef* pI2CHandle;
	SPI_HandleTypeDef* pSPIHandle;
	HAL_StatusTypeDef status;

	GPIO_TypeDef* pGPIOPort;
	uint16_t pinNo;
	//TODO: Include function pointers to the below functions
	//void (*init)(ArducamController*, I2C_HandleTypeDef*, SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t);
	void (*setCS)(int);

};


// Static makes functions "private"
//Initializer
void initArducam(ArducamController* pCtrl, I2C_HandleTypeDef* pHI2C, SPI_HandleTypeDef* pHSPI, GPIO_TypeDef* pGPIOPort, uint16_t pinNo);

//I2C Functions
void i2cRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size);
void i2cRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size);

//SPI Functions
void spiRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size);
void spiRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size);

//Camera Functions
int isFIFOBusy(ArducamController* pCtrl);
void readFrameBuffer(ArducamController* pCtrl, uint8_t *buffer);
void singleCapture(ArducamController* pCtrl);
void flushFIFO(ArducamController* pCtrl);
void clearFIFOFlag(ArducamController* pCtrl);
void resetFIFOPointers(ArducamController* pCtrl);

//Helpers
void printStatus(ArducamController* pCtrl);
void setCS(ArducamController* pCtrl, int level);


#endif /* SRC_ARDUCAMCONTROLLER_H_ */
