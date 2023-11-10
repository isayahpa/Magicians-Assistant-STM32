
#include "ArducamController.h"

/* Notes:
 * The chip select signal should always be LOW during the SPI read or write bus cycle
 * Add printf() prototype from main into here if printing doesn't work
 */

void setCS(ArducamController* pCtrl, int level){
	HAL_GPIO_WritePin(pCtrl->pGPIOPort, pCtrl->pinNo, level);
}

void initArducam(ArducamController* pCtrl, I2C_HandleTypeDef* pHI2C, SPI_HandleTypeDef* pHSPI, GPIO_TypeDef* pGPIOPort, uint16_t pinNo){
	printf("Initializing ArduCam\n");
	// Might need to Malloc/Calloc some memory here?
	pCtrl->pI2CHandle = pHI2C;
	pCtrl->pSPIHandle = pHSPI;
	pCtrl->status = HAL_OK;
	//pCtrl->init = &init;
	pCtrl -> pGPIOPort = pGPIOPort;
	pCtrl -> pinNo = pinNo;

	setCS(pCtrl, GPIO_PIN_RESET); //CS LOW to to enable

	uint8_t cmd = 0b00000001; //Capture 1 Frame per Capture
	flushFIFO(pCtrl);
	i2cRegWrite(pCtrl, CAPTURE_CONTROL_REG, &cmd, 1);

	setCS(pCtrl, GPIO_PIN_SET); //CS HIGH when finished
	printStatus(pCtrl);
}

//TODO: Make it so that when Register Writes/Reads fail (status != 00), we print error data and throw an interrupt? or maybe just halt the function?
//
/*To Write over i2c:
* Request I2C_ADDR_WRITE -> Send Cam Register Address (left shift device addr's by 1) -> Send Data bytes
*/

void i2cRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){
	pCtrl->status = HAL_I2C_Master_Transmit(pCtrl->pI2CHandle, I2C_ADDR_WRITE<<1, &reg, 1, TIMEOUT);
	pCtrl->status = HAL_I2C_Master_Transmit(pCtrl->pI2CHandle, I2C_ADDR_WRITE<<1, pData, size, TIMEOUT);
}

/*To Read over i2c:
	* To I2C_ADDR_WRITE: Write the Register You want to read from
	* To I2C_ADDR_READ: Read as much data as you want
*/
void i2cRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	pCtrl->status = HAL_I2C_Master_Transmit(pCtrl->pI2CHandle, I2C_ADDR_WRITE<<1, &reg, 1, TIMEOUT);
	pCtrl->status = HAL_I2C_Master_Receive(pCtrl->pI2CHandle, I2C_ADDR_READ<<1, pBuffer, size, TIMEOUT);
}

void spiRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){
	setCS(pCtrl, GPIO_PIN_RESET); // CS Pin Set LOW

	uint8_t cmdByte = 0x80 | reg; // a 1 followed by Reg addr, to write to reg

	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, &cmdByte, 1, TIMEOUT);
	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, pData, size, TIMEOUT);
}

void spiRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	setCS(pCtrl, GPIO_PIN_RESET); // CS Pin Set LOW

	uint8_t cmdByte = 0x00 | reg; // a 0 followed by register to read

	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, &cmdByte, 1, TIMEOUT);
	pCtrl->status = HAL_SPI_Receive(pCtrl->pSPIHandle, pBuffer, size, TIMEOUT);
}

int isFIFOBusy(ArducamController* pCtrl){
	uint8_t data = 0x00;
	i2cRegRead(pCtrl, FIFO_STATUS_REG, &data, 1);
	return checkBit(data, 3);
}

void readFrameBuffer(ArducamController* pCtrl, uint8_t *buffer){

	if(isFIFOBusy(pCtrl)){
		printf("Can't read, FIFO is busy...");
	} else {
		i2cRegRead(pCtrl, FIFO_BYTE0, &buffer[0], 1);
		i2cRegRead(pCtrl, FIFO_BYTE1, &buffer[1], 1);
		i2cRegRead(pCtrl, FIFO_BYTE2, &buffer[2], 1);
	}

}

void singleCapture(ArducamController* pCtrl){

	if(isFIFOBusy(pCtrl)){
		printf("Can't Capture, FIFO is busy...");
	} else {
		uint8_t cmd = SET_CAPTURE_FLAG;
		i2cRegWrite(pCtrl, FIFO_CONTROL_REG, &cmd, 1);
	}
}

void flushFIFO(ArducamController* pCtrl){
	clearFIFOFlag(pCtrl);
	resetFIFOPointers(pCtrl);
}

void clearFIFOFlag(ArducamController* pCtrl){
	uint8_t cmd = FIFO_FLAG_CLR;
	i2cRegWrite(pCtrl, FIFO_CONTROL_REG, &cmd, 1);
}

void resetFIFOPointers(ArducamController* pCtrl){
	uint8_t cmd = FIFO_PTR_CLR;
	i2cRegWrite(pCtrl, FIFO_CONTROL_REG, &cmd, 1);
}

void printStatus(ArducamController* pCtrl){
	printf("Arducam Status: %x\n", pCtrl->status);
}

