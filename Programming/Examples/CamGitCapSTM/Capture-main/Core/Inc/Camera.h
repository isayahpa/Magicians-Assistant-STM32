/*
 * Camera.h
 *
 *  Created on: Oct 17, 2022
 *      Author: semba
 */

#ifndef INC_CAMERA_H_
#define INC_CAMERA_H_

#include "main.h"
#include "sensorRegs.h"

#define SPI_WRITE_MASK 0b10000000	// 0x80
#define SPI_READ_MASK 0b01111111	// 0x7f

#define I2C_DEVICE_ADDRESS 0x78		// sccb (i2c) address of ov5642 shifted to 7bits (OV5642 doc page 119 and 113)

//! Function to initialize camera
/*! Initialize camera same way as in official ArduCam demo */
void camInit(I2C_HandleTypeDef hi2c1, SPI_HandleTypeDef hspi1);

//! Function to capture image
/*! Capture single image and output it to uart */
void snapPic(I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2, SPI_HandleTypeDef hspi1);

//! Write new value to i2c register
HAL_StatusTypeDef wCamReg(I2C_HandleTypeDef hi2c1, uint16_t regID, uint16_t data);
//! Loop function wCamReg on set of registers
HAL_StatusTypeDef wCamRegs(I2C_HandleTypeDef hi2c1, const struct sensor_reg regList[]);
//! Reset camera using internal register
void resetCam(I2C_HandleTypeDef hi2c1);

//! Write new value to spi register
HAL_StatusTypeDef wCamRegSPI(SPI_HandleTypeDef hspi1, uint8_t addr, uint8_t data);
//! Read value from spi register
uint8_t rCamSPI(SPI_HandleTypeDef hspi1, uint8_t addr);
//! Set CS signal low - start communication
void spiStart();
//! Set CS signal high - end communication
void spiEnd();

//! Reset FIFO write and read pointers using internal register
void resetFifoFlags(SPI_HandleTypeDef hspi1);
//! Reset Capture Done flag using internal register
void resetCapDoneFlag(SPI_HandleTypeDef hspi1);
//! Set number of images to capture
void setCaptureCount(SPI_HandleTypeDef hspi1, uint8_t captureCount);
//! Start capturing image
void startCapture(SPI_HandleTypeDef hspi1);
//!get length of FIFO buffer
uint32_t getFifoLen(SPI_HandleTypeDef hspi1);


#endif /* INC_CAMERA_H_ */
