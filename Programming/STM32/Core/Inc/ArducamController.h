/*
 * ArducamController.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Isayah Parent
 */

#ifndef SRC_ARDUCAMCONTROLLER_H_
#define SRC_ARDUCAMCONTROLLER_H_

#include "stm32l4xx_hal.h"
#include "helpers.h"
#include "cam_regs.h"

// Register Addresses
//https://www.uctronics.com/download/Amazon/ArduCAM_Mini_2MP_Camera_Shield_Hardware_Application_Note.pdf

//I2C Addresses
#define I2C_ADDR_WRITE (uint16_t) 0x60
#define I2C_ADDR_READ (uint16_t) 0x61

//Internal Register Addresses
#define TEST_REGISTER (uint8_t) 0x00
#define RESET_REG (uint8_t)
#define CAPTURE_CONTROL_REG (uint8_t) 0x01
#define SENSOR_INTERFACE_TIMING_REG (uint8_t) 0x03
#define FIFO_CONTROL_REG (uint8_t) 0x04
#define FIFO_SINGLE_READ_CMD (uint8_t) 0x3D
#define FIFO_STATUS_REG (uint8_t) 0x41
#define CHIP_VERSION_REG (uint8_t) 0x40
#define FIFO_BYTE0 (uint8_t) 0x42
#define FIFO_BYTE1 (uint8_t) 0x43
#define FIFO_BYTE2 (uint8_t) 0x44

//Commands
#define FIFO_FLAG_CLR (uint8_t)0x01
#define	SET_CAPTURE_FLAG (uint8_t)0x02
#define FIFO_PTR_CLR (uint8_t)0x30
#define CAPTURE_CMD (uint8_t) 0x84
#define FIFO_BURST_READ (uint8_t) 0x3C
#define MAX_FIFO_LENGTH 0x5FFFF

//Constants
#define CAM_TIMEOUT HAL_MAX_DELAY
#define CAPTURE_DELAY 1000
#define CS_DELAY 10
#define SPI_CLK_HZ 800000
#define SPI_WRITE_MASK 0x80
#define SPI_READ_MASK 0x7F

typedef struct ArducamController ArducamController;

struct ArducamController{
	I2C_HandleTypeDef* pI2CHandle;
	SPI_HandleTypeDef* pSPIHandle;
	HAL_StatusTypeDef status;

	GPIO_TypeDef* pGPIOPort;
	uint16_t pinNo;

};

// Static makes functions "private"
//Initializer
void initArducam(ArducamController*, I2C_HandleTypeDef*, SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t);

//I2C Functions
void i2cRegWrite(ArducamController*, uint8_t, uint8_t*, uint16_t);
void i2cRegRead(ArducamController*, uint8_t, uint8_t*, uint16_t);
void i2cWriteMultiple(ArducamController*, const struct SensorReg*);

//SPI Functions
void spiRegWrite(ArducamController*, uint8_t, uint8_t*, uint16_t);
void spiRegRead(ArducamController*, uint8_t, uint8_t*, uint16_t);

//Camera Functions
int isFIFOReady(ArducamController*);
void singleRead(ArducamController*, uint8_t *);
void singleCapture(ArducamController*, uint8_t**);
void flushFIFO(ArducamController*);
void clearFIFOFlag(ArducamController*);
void resetFIFOPointers(ArducamController*);
void setDefaultSettings(ArducamController*);
void setNCaptureFrames(ArducamController*, int);
void setCaptureFlag(ArducamController*);
void resetCPLD(ArducamController*);
void burstReadFIFO(ArducamController*, uint8_t*);

//Helpers
void printStatus(ArducamController*);
void enable(ArducamController*);
void disable(ArducamController*);
void registerDump(ArducamController*);
uint32_t getFIFOLength(ArducamController*);
int isSPIWorking(ArducamController*);

#endif /* SRC_ARDUCAMCONTROLLER_H_ */


