/*
 * ArducamController.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Isayah Parent
 */

#ifndef SRC_ARDUCAMCONTROLLER_H_
#define SRC_ARDUCAMCONTROLLER_H_

#include "stm32l4xx_hal.h"
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

typedef struct ArducamController ArducamController;

struct ArducamController{
	I2C_HandleTypeDef* pI2CHandle;
	SPI_HandleTypeDef* pSPIHandle;
	HAL_StatusTypeDef* status;
	//TODO: Include function pointers to the below functions
	void (*init)(const ArducamController*, I2C_HandleTypeDef*, SPI_HandleTypeDef*);

};
/*
//Initializer
void init(const struct ArducamController*);

//I2C Functions
void i2cRegWrite(uint8_t reg, uint8_t *pData, uint16_t size, const struct ArducamController*);
void i2cRegRead(uint8_t reg, uint8_t *pBuffer, uint16_t size);

//SPI Functions
void spiRegWrite(uint8_t reg, uint8_t *pData, uint16_t size);
void spiRegRead(uint8_t reg, uint8_t *pBuffer, uint16_t size);

//Camera Functions
int isFIFOBusy();
HAL_StatusTypeDef readFrameBuffer(uint8_t *buffer);
HAL_StatusTypeDef singleCapture();
HAL_StatusTypeDef flushFIFO();
HAL_StatusTypeDef clearFIFOFlag();
HAL_StatusTypeDef resetFIFOPointers();

//Helpers
void printStatus(ArducamController* camController);
*/
#endif /* SRC_ARDUCAMCONTROLLER_H_ */
