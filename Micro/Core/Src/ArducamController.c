#include <stdio.h>
#include "ArducamController.h"


/* Notes:
 * The chip select signal should always be LOW during the SPI read or write bus cycle
 * Add printf() prototype from main into here if printing doesn't work
 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


void init(const ArducamController* pCtrl, I2C_HandleTypeDef* pHI2C, SPI_HandleTypeDef* pHSPI, HAL_StatusTypeDef* pStatus){
	printf("Initializing ArduCam");
	pCtrl->pI2CHandle = pHI2C;
	pCtrl->pSPIHandle = pHSPI;
	pCtrl->status = pStatus;
	pCtrl->init = &init;

	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET); //CS LOW to configure
	uint8_t cmd = 0b00000001; //Capture 1 Frame per Capture

	resetFIFOPointers();
	clearFIFOFlag();
	i2cRegWrite(CAPTURE_CONTROL_REG, &cmd, 1, pCtrl);
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_SET); //CS HIGH when finished
}

//TODO: Make it so that when Register Writes/Reads fail (status != 00), we print error data and throw an interrupt? or maybe just halt the function?
//
/*To Write over i2c:
* Request I2C_ADDR_WRITE -> Send Cam Register Address (left shift device addr's by 1) -> Send Data bytes
*/

HAL_StatusTypeDef i2cRegWrite(const ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(i2cHandle, I2C_ADDR_WRITE<<1, &reg, 1, TIMEOUT);
	status = HAL_I2C_Master_Transmit(i2cHandle, I2C_ADDR_WRITE<<1, pData, size, TIMEOUT);
	*(pCtrl->status) = status;
	return status;
}

/*To Read over i2c:
	* To I2C_ADDR_WRITE: Write the Register You want to read from
	* To I2C_ADDR_READ: Read as much data as you want
*/
HAL_StatusTypeDef i2cRegRead(const struct ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(i2cHandle, I2C_ADDR_WRITE<<1, &reg, 1, TIMEOUT);
	status = HAL_I2C_Master_Receive(i2cHandle, I2C_ADDR_READ<<1, pBuffer, size, TIMEOUT);
	*(pCtrl->status) = status;
	return status;
}

HAL_StatusTypeDef spiRegWrite(const struct ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET); // CS Pin Set LOW

	uint8_t cmdByte = 0x80 | reg; // a 1 followed by Reg addr, to write to reg

	status = HAL_SPI_Transmit(spiHandle, &cmdByte, 1, TIMEOUT);
	status = HAL_SPI_Transmit(spiHandle, pData, size, TIMEOUT);
	*(pCtrl->status) = status;
	return status;
}

HAL_StatusTypeDef spiRegRead(const struct ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(CAM_CS_GPIO_Port, CAM_CS_Pin, GPIO_PIN_RESET); // CS Pin Set LOW

	uint8_t cmdByte = 0x00 | reg; // a 0 followed by register to read

	status = HAL_SPI_Transmit(spiHandle, &cmdByte, 1, TIMEOUT);
	status = HAL_SPI_Receive(spiHandle, pBuffer, size, TIMEOUT);
	*(pCtrl->status) = status;
	return status;
}

int isFIFOBusy(const struct ArducamController* pCtrl){
	uint8_t data = 0x00;
	i2cRegRead(FIFO_STATUS_REG, &data, 1, pCtrl);
	return checkBit(data, 3);
}

void readFrameBuffer(const struct ArducamController* pCtrl, uint8_t *buffer){

	if(isFIFOBusy()){
		printf("Can't read, FIFO is busy...");
	} else {
		i2cRegRead(FIFO_BYTE0, &buffer[0], 1, pCtrl);
		i2cRegRead(FIFO_BYTE1, &buffer[1], 1, pCtrl);
		i2cRegRead(FIFO_BYTE2, &buffer[2], 1, pCtrl);
	}

}

void singleCapture(const struct ArducamController* pCtrl){

	if(isFIFOBusy()){
		printf("Can't Capture, FIFO is busy...");
	} else {
		uint8_t cmd = SET_CAPTURE_FLAG;
		i2cRegWrite(FIFO_CONTROL_REG, &cmd, 1, pCtrl);
	}
}

void flushFIFO(const struct ArducamController* pCtrl){
	clearFIFOFlag(pCtrl);
	resetFIFOPointers(pCtrl);
}

void clearFIFOFlag(const struct ArducamController* pCtrl){
	uint8_t cmd = FIFO_FLAG_CLR;
	i2cRegWrite(FIFO_CONTROL_REG, &cmd, 1, pCtrl);
}

void resetFIFOPointers(const struct ArducamController* pCtrl){
	uint8_t cmd = FIFO_PTR_CLR;
	i2cRegWrite(FIFO_CONTROL_REG, &cmd, 1, pCtrl);
}

