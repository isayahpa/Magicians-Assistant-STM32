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
#define TEST_REGISTER 0x00
#define CAPTURE_CONTROL_REG 0x01
#define SENSOR_INTERFACE_TIMING_REG 0x03
#define FIFO_CONTROL_REG 0x04
#define FIFO_SINGLE_READ_CMD 0x3D
#define FIFO_STATUS_REG 0x41
#define CHIP_VERSION_REG 0x40
#define FIFO_BYTE0 0x42
#define FIFO_BYTE1 0x43
#define FIFO_BYTE2 0x44

//Commands
#define FIFO_FLAG_CLR 0x01
#define	SET_CAPTURE_FLAG 0x02
#define FIFO_PTR_CLR 0x30
#define CAPTURE_CMD 0x84
#define FIFO_BURST_READ 0x3C
#define MAX_FIFO_LENGTH 0x5FFF

//Constants
#define CAM_TIMEOUT 2000
#define CAPTURE_DELAY 1000
#define SHUTTER_DELAY 1000
#define CS_DELAY 0

#define SPI_WRITE_MASK 0x80
#define SPI_READ_MASK 0x7F

#define MAX_PIC_BUF_SIZE 4096
#define MAX_BASE64_BUF_SIZE ((MAX_PIC_BUF_SIZE * 4)/3) + 1
typedef struct ArducamController ArducamController;

struct ArducamController{
	HAL_StatusTypeDef status;
	I2C_HandleTypeDef* pI2CHandle;
	SPI_HandleTypeDef* pSPIHandle;

	GPIO_TypeDef* pCSPort;
	uint16_t csPinNo;
	GPIO_TypeDef* pFlashPort;
	uint16_t flashPinNo;

	uint8_t pictureBuffer[MAX_PIC_BUF_SIZE];
	char base64Buffer[MAX_BASE64_BUF_SIZE];
	//uint8_t* pictureBuffer;
	//char* base64Buffer;
	uint16_t base64Size;
	uint16_t pictureBufferSize;
};

// Static makes functions "private"
//Initializer/Deconstructor
HAL_StatusTypeDef initArducam(ArducamController* pCtrl);
HAL_StatusTypeDef disconnectArducam(ArducamController* pCtrl);
void clearPicBuf(ArducamController* pCtrl);

//I2C Functions
void i2cRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t data);
uint8_t i2cRegRead(ArducamController* pCtrl, uint8_t reg);
void i2cWriteMultiple(ArducamController* pCtrl, const struct SensorReg*);

//SPI Functions
HAL_StatusTypeDef spiRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t data);
uint8_t spiRegRead(ArducamController* pCtrl, uint8_t reg);

//Camera Functions
int isFIFOReady(ArducamController* pCtrl);
HAL_StatusTypeDef singleCapture(ArducamController* pCtrl);
void clearFIFOFlag(ArducamController* pCtrl);
void resetFIFOPointers(ArducamController* pCtrl);
void setDefaultSettings(ArducamController* pCtrl);
void setNCaptureFrames(ArducamController* pCtrl, int nFrames);
void setCaptureFlag(ArducamController* pCtrl);
void resetCPLD(ArducamController* pCtrl);
uint16_t burstReadFIFO(ArducamController* pCtrl);
void shutter(ArducamController* pCtrl);
void flashOn(ArducamController* pCtrl);
void flashOff(ArducamController* pCtrl);

//Helpers
void registerDump(ArducamController* pCtrl);
void cam_enable(ArducamController* pCtrl);
void cam_disable(ArducamController* pCtrl);

void registerDump(ArducamController* pCtrl);
uint32_t getFIFOLength(ArducamController* pCtrl);
int isSPIWorking(ArducamController* pCtrl);
void picToBase64(ArducamController* pCtrl);

#endif /* SRC_ARDUCAMCONTROLLER_H_ */


