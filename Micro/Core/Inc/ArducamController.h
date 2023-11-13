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

};

// Static makes functions "private"
//Initializer
void initArducam(ArducamController*, I2C_HandleTypeDef*, SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t);

//I2C Functions
void i2cRegWrite(ArducamController*, uint8_t, uint8_t*, uint16_t);
void i2cRegRead(ArducamController*, uint8_t, uint8_t*, uint16_t);

//SPI Functions
void spiRegWrite(ArducamController*, uint8_t, uint8_t*, uint16_t);
void spiRegRead(ArducamController*, uint8_t, uint8_t*, uint16_t);

//Camera Functions
int isFIFOBusy(ArducamController*);
void readFrameBuffer(ArducamController*, uint8_t*);
void singleCapture(ArducamController*);
void flushFIFO(ArducamController*);
void clearFIFOFlag(ArducamController*);
void resetFIFOPointers(ArducamController*);

//Helpers
static void printStatus(ArducamController*);
void enable(ArducamController*);
void disable(ArducamController*);
void registerDump(ArducamController*);

#endif /* SRC_ARDUCAMCONTROLLER_H_ */
