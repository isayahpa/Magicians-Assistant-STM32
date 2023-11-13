
#include "ArducamController.h"

/* Notes:
 * The chip select signal should always be LOW during the SPI read or write bus cycle
 * Add printf() prototype from main into here if printing doesn't work
 */

void initArducam(ArducamController* pCtrl, I2C_HandleTypeDef* pHI2C, SPI_HandleTypeDef* pHSPI, GPIO_TypeDef* pGPIOPort, uint16_t pinNo){
	printf("Initializing ArduCam\n");
	pCtrl->pI2CHandle = pHI2C;
	pCtrl->pSPIHandle = pHSPI;
	pCtrl->status = HAL_OK;
	pCtrl -> pGPIOPort = pGPIOPort;
	pCtrl -> pinNo = pinNo;

	enable(pCtrl);

	printf("Flushing FIFO\n");
	flushFIFO(pCtrl);

	uint8_t cmd = 0b00000001; //Capture 1 Frame per Capture
	i2cRegWrite(pCtrl, CAPTURE_CONTROL_REG, &cmd, 1);


	disable(pCtrl);

	printStatus(pCtrl);
}

//TODO: Make it so that when Register Writes/Reads fail (status != 00), we print error data and throw an interrupt? or maybe just halt the function?
// TODO: Find out why the status == 1 when we try I2C transmit
//
/*To Write over i2c:
* Request I2C_ADDR_WRITE -> Send Cam Register Address (left shift device addr's by 1) -> Send Data bytes
*/

void i2cRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){
	printf("Writing %x to Reg %x\n", *pData, reg);
	printf("Before T1: %x\n", pCtrl->status);
	pCtrl->status = HAL_I2C_Master_Transmit(pCtrl->pI2CHandle, I2C_ADDR_WRITE<<1, &reg, 1, TIMEOUT);
	printf("After T1: %x\n", pCtrl->status);
	pCtrl->status = HAL_I2C_Master_Transmit(pCtrl->pI2CHandle, I2C_ADDR_WRITE<<1, pData, size, TIMEOUT);
	printf("After T2: %x\n", pCtrl->status);
}

/*To Read over i2c:
	* To I2C_ADDR_WRITE: Write the Register You want to read from
	* To I2C_ADDR_READ: Read as much data as you want
*/
void i2cRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	printf("Reading from Reg %x\n", reg);
	pCtrl->status = HAL_I2C_Master_Transmit(pCtrl->pI2CHandle, I2C_ADDR_WRITE<<1, &reg, 1, TIMEOUT);
	printf("After T1: %x\n", pCtrl->status);
	pCtrl->status = HAL_I2C_Master_Receive(pCtrl->pI2CHandle, I2C_ADDR_READ<<1, pBuffer, size, TIMEOUT);
	printf("After R1: %x\n", pCtrl->status);
}

void spiRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){
	enable(pCtrl); // CS Pin Set LOW

	uint8_t cmdByte = 0x80 | reg; // a 1 followed by Reg addr, to write to reg

	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, &cmdByte, 1, TIMEOUT);
	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, pData, size, TIMEOUT);

	disable(pCtrl);
}

void spiRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	enable(pCtrl);

	uint8_t cmdByte = 0x00 | reg; // a 0 followed by register to read

	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, &cmdByte, 1, TIMEOUT);
	pCtrl->status = HAL_SPI_Receive(pCtrl->pSPIHandle, pBuffer, size, TIMEOUT);

	disable(pCtrl);
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


void enable(ArducamController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pGPIOPort, pCtrl->pinNo, GPIO_PIN_RESET);
}

void disable(ArducamController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pGPIOPort, pCtrl->pinNo, GPIO_PIN_SET);
}

//Prints all of the relevant registers in the Arducam
void registerDump(ArducamController* pCtrl){
	uint8_t data;
	printf("*******Register Table:*******\n");
	printf("(I2C Read)\n");
	i2cRegRead(pCtrl, TEST_REGISTER, &data, 1);
	printf("Test Register: %x\n", data);
	i2cRegRead(pCtrl, CAPTURE_CONTROL_REG, &data, 1);
	printf("Capture Control Register: %x\n", data);
	i2cRegRead(pCtrl, FIFO_CONTROL_REG, &data, 1);
	printf("FIFO Control Register: %x\n", data);
	i2cRegRead(pCtrl, FIFO_SINGLE_READ_REG, &data, 1);
	printf("FIFO Single Read Register: %x\n", data);
	i2cRegRead(pCtrl, FIFO_STATUS_REG, &data, 1);
	printf("FIFO Status Register: %x\n", data);

	printf("(SPI Read)\n");
	spiRegRead(pCtrl, TEST_REGISTER, &data, 1);
	printf("Test Register: %x\n", data);
	spiRegRead(pCtrl, CAPTURE_CONTROL_REG, &data, 1);
	printf("Capture Control Register: %x\n", data);
	spiRegRead(pCtrl, FIFO_CONTROL_REG, &data, 1);
	printf("FIFO Control Register: %x\n", data);
	spiRegRead(pCtrl, FIFO_SINGLE_READ_REG, &data, 1);
	printf("FIFO Single Read Register: %x\n", data);
	spiRegRead(pCtrl, FIFO_STATUS_REG, &data, 1);
	printf("FIFO Status Register: %x\n", data);

}
static void printStatus(ArducamController* pCtrl){
	printf("**********Status Report**********\n");
	printf("Arducam Status: %x\n", pCtrl->status);
	registerDump(pCtrl);
	printf("*******************************\n");

}

