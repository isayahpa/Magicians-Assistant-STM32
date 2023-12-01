
#include "ArducamController.h"
#include "helpers.h"
#include <stdlib.h>

/* Notes:
 * The chip select signal should always be LOW during the SPI read or write bus cycle
 * I2C interfaces directly with the OV2640 sensor (the camera itself)
 * SPI interfaces with the Chip as a whole, to indirectly control the camera
 */
void initArducam(ArducamController* pCtrl, I2C_HandleTypeDef* pHI2C, SPI_HandleTypeDef* pHSPI, GPIO_TypeDef* pCSPort, uint16_t csPinNo, GPIO_TypeDef* pFlashPort, uint16_t flashPinNo){
	printf("Initializing ArduCam\n");
	pCtrl->pI2CHandle = pHI2C;
	pCtrl->pSPIHandle = pHSPI;
	pCtrl->status = HAL_OK;
	pCtrl -> pCSPort = pCSPort;
	pCtrl -> csPinNo = csPinNo;
	pCtrl -> pFlashPort = pFlashPort;
	pCtrl -> flashPinNo = flashPinNo;

	resetCPLD(pCtrl);
	if((pCtrl->status = HAL_I2C_IsDeviceReady(pHI2C, I2C_ADDR_WRITE, 1, HAL_MAX_DELAY)) != HAL_OK ||
			(pCtrl->status = HAL_I2C_IsDeviceReady(pHI2C, I2C_ADDR_READ, 1, HAL_MAX_DELAY != HAL_OK))){
		printf("Arducam I2C Error.\n");
	} else if(!isSPIWorking(pCtrl)){
		printf("Arducam SPI Error.\n");
	} else {
		printf("I2C Check Passed |");
		printf("SPI Check Passed\n");
		setDefaultSettings(pCtrl);
		HAL_Delay(1000);
	}

	if(pCtrl->status != HAL_OK){
		printf("Arducam Init Fail\n");
	} else {
		printf("Arducam Init Success!\n");
	}

	//printStatus(pCtrl);
}

//Fills *ppBuffer with the Picture Data
//Returns the # of bytes read from the FIFO
uint16_t singleCapture(ArducamController* pCtrl, uint8_t **ppBuffer){
		printf("Starting Capture\n");

		flashOn(pCtrl);
		clearFIFOFlag(pCtrl);
		resetFIFOPointers(pCtrl);
		setNCaptureFrames(pCtrl, 1);
		setCaptureFlag(pCtrl);
		while(!isFIFOReady(pCtrl)){
			HAL_Delay(CAPTURE_DELAY);//    Wait 'til Finished Flag is set
		}
		printf("FIFO Write Finished!\n");
		flashOff(pCtrl);

		uint16_t bufferSize = burstReadFIFO(pCtrl, ppBuffer);

		printf("Capture Complete!\n");
		/*printf("Picture Buffer: \n");
		for(int i = 0; i < bufferSize; i++){
			printf("%x", (*ppBuffer)[i]);
			//if(!(i % 100)){printf("\n");}
		}
		printf("\n");
		*/
		return bufferSize;
}

//Returns the amount of data (in bytes) read from FIFO
uint16_t burstReadFIFO(ArducamController *pCtrl, uint8_t **ppBuffer){
	uint8_t cmd = FIFO_BURST_READ;
	uint32_t fifoLength = getFIFOLength(pCtrl);
	uint32_t transmissionSize = fifoLength;
	if(fifoLength >= 0xFFFF){
		printf("Had to Truncate FIFO Transfer\n");
		transmissionSize = 0xFFFF;
	}

	// Allocate some space for the buffer
	*ppBuffer = calloc(transmissionSize, sizeof(uint8_t));

	printf("Reading %lu bytes from Arducam\n", transmissionSize);
	cam_enable(pCtrl);
	pCtrl->status = HAL_SPI_TransmitReceive(pCtrl->pSPIHandle, &cmd, *ppBuffer, 1, HAL_MAX_DELAY);
	pCtrl -> status = HAL_SPI_Receive(pCtrl->pSPIHandle, *ppBuffer, transmissionSize, HAL_MAX_DELAY);
	HAL_Delay(1000); // Just making sure all the data makes it through
	cam_disable(pCtrl);

	return transmissionSize;
}

void setDefaultSettings(ArducamController* pCtrl){
	printf("Configuring Default Settings\n");
	uint8_t data = 0x01;
	i2cRegWrite(pCtrl, 0xFF, &data, 1);
	data = 0x80;
	i2cRegWrite(pCtrl, 0x12, &data, 1);
	HAL_Delay(100);

	i2cWriteMultiple(pCtrl, OV2640_JPEG_INIT);
	i2cWriteMultiple(pCtrl, OV2640_YUV422);
	i2cWriteMultiple(pCtrl, OV2640_JPEG);
	data = 0x01;
	i2cRegWrite(pCtrl, 0xFF, &data, 1);
	data = 0x00;
	i2cRegWrite(pCtrl, 0x15, &data, 1);
	i2cWriteMultiple(pCtrl, OV2640_320x240_JPEG);

	// Sets VSync Polarity Low
	data = 0x02;
	spiRegWrite(pCtrl, (uint8_t)0x03, &data, 1);

}


int isSPIWorking(ArducamController *pCtrl){
	uint8_t testVal = 0xAB;
	uint8_t readVal = 0x00;
	spiRegWrite(pCtrl, 0x00, &testVal, 1);
	spiRegRead(pCtrl, 0x00, &readVal, 1);

	return (readVal == testVal);
}

//TODO: Make it so that when Register Writes/Reads fail (status != 00), we print error data and throw an interrupt? or maybe just halt the function?
//
/*To Write over i2c:
* Request I2C_ADDR_WRITE -> Send Cam Register Address (left shift device addr's by 1) -> Send Data bytes
*/
void i2cRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){
	printf("(I2C) Writing 0x%04X to Reg 0x%02X\n", *pData, reg);
	//printf("Before T1: %x\n", pCtrl->status);
	pCtrl->status = HAL_I2C_Master_Transmit(pCtrl->pI2CHandle, I2C_ADDR_WRITE, &reg, (uint16_t) 1, CAM_TIMEOUT);
	//printf("After T1: %x\n", pCtrl->status);
	pCtrl->status = HAL_I2C_Master_Transmit(pCtrl->pI2CHandle, I2C_ADDR_WRITE, pData, (uint16_t) size, CAM_TIMEOUT);
	//printf("After T2: %x\n", pCtrl->status);
}

/*To Read over i2c:
	* To I2C_ADDR_WRITE: Write the Register You want to read from
	* To I2C_ADDR_READ: Read as much data as you want
*/
void i2cRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	pCtrl->status = HAL_I2C_Master_Transmit(pCtrl->pI2CHandle, I2C_ADDR_WRITE, &reg, 1, CAM_TIMEOUT);
	//printf("After T1: %x\n", pCtrl->status);
	pCtrl->status = HAL_I2C_Master_Receive(pCtrl->pI2CHandle, I2C_ADDR_READ, pBuffer, size, CAM_TIMEOUT);
	//printf("After R1: %x\n", pCtrl->status);
	printf("(I2C) Read 0x%02X from 0x%02X\n", *pBuffer, reg);
}

void i2cWriteMultiple(ArducamController* pCtrl, const struct SensorReg *regList){
	struct SensorReg *current = (struct SensorReg *) regList;

	while(current->addr != 0xFF || current->val != 0xFF){
		i2cRegWrite(pCtrl, current->addr, &(current->val), 1);
		current++;
	}

}

void spiRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){

	cam_enable(pCtrl); // CS Pin Set LOW
	HAL_Delay(CS_DELAY);
	uint8_t maskedAddr = reg | SPI_WRITE_MASK; // a 1 followed by Reg addr, to write to reg

	printf("(SPI) Writing 0x%02X to 0x%02X | cmd = 0x%02X\n", *pData, reg, maskedAddr);
	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, &maskedAddr, (uint16_t)1, CAM_TIMEOUT);
	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, pData, size, CAM_TIMEOUT);

	cam_disable(pCtrl);
}

void spiRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	cam_enable(pCtrl);
	HAL_Delay(CS_DELAY);
	uint8_t maskedAddr = reg & SPI_READ_MASK; // a 0 followed by register to read
	uint8_t dummyByte = 0x00;
	pCtrl->status = HAL_SPI_TransmitReceive(pCtrl->pSPIHandle, &maskedAddr, pBuffer, (uint16_t)1, CAM_TIMEOUT);
	pCtrl->status = HAL_SPI_TransmitReceive(pCtrl->pSPIHandle, &dummyByte, pBuffer, size, CAM_TIMEOUT);
	cam_disable(pCtrl);
	printf("(SPI) Read 0x%02X from 0x%02X\n", *pBuffer, reg);
}

//Returns FIFO 'finished' flag. 0 -> FIFO is busy, 1 -> capture is finished

int isFIFOReady(ArducamController* pCtrl){
	uint8_t registerData = 0;
	spiRegRead(pCtrl, FIFO_STATUS_REG, &registerData, 1);
	int isFinished = checkBit(registerData, 3);
	printf("FIFO Ready Flag : %d\n", isFinished);
	return isFinished;
}

//Might remove this
void singleRead(ArducamController* pCtrl, uint8_t *buffer){

	printf("Trying to read Frame Buffer...\n");
	if(!isFIFOReady(pCtrl)){
		printf("Can't read, FIFO is busy...\n");
	} else {
		spiRegRead(pCtrl, FIFO_BYTE0, &buffer[2], 1);
		spiRegRead(pCtrl, FIFO_BYTE1, &buffer[1], 1);
		spiRegRead(pCtrl, FIFO_BYTE2, &buffer[0], 1);
	}

}

//Resets the CPLD
void resetCPLD(ArducamController* pCtrl){
	uint8_t cmd = 0x80;
	spiRegWrite(pCtrl, 0x07, &cmd, 1);
	HAL_Delay(100);
	cmd = 0x00;
	spiRegWrite(pCtrl, 0x07, &cmd, 1);
	HAL_Delay(100);
}


void flushFIFO(ArducamController* pCtrl){
	printf("Flushing FIFO\n");
	clearFIFOFlag(pCtrl);
	//printf("Reseting FIFO Pointers\n");
	//resetFIFOPointers(pCtrl);
}

void clearFIFOFlag(ArducamController* pCtrl){
	uint8_t cmd = FIFO_FLAG_CLR;
	spiRegWrite(pCtrl, FIFO_CONTROL_REG, &cmd, 1);
}

void resetFIFOPointers(ArducamController* pCtrl){
	uint8_t cmd = FIFO_PTR_CLR;
	spiRegWrite(pCtrl, FIFO_CONTROL_REG, &cmd, 1);
}

void setCaptureFlag(ArducamController* pCtrl){
	uint8_t cmd = SET_CAPTURE_FLAG;
	spiRegWrite(pCtrl, FIFO_CONTROL_REG, &cmd, 1);
}

void setNCaptureFrames(ArducamController* pCtrl, int n){
	uint8_t cmd = 0b00000001;
	if((0 < n) && (n <= 7)){ cmd = (uint8_t) n; }

	spiRegWrite(pCtrl, CAPTURE_CONTROL_REG, &cmd, 1);
}

//How we find the number of bytes the FIFO is holding (for burst reading)
uint32_t getFIFOLength(ArducamController *pCtrl){
	uint32_t reg0, reg1, reg2 = 0;
	uint32_t fifoLength = 0;

	spiRegRead(pCtrl, FIFO_BYTE0, (uint8_t*) &reg0, 1);
	spiRegRead(pCtrl, FIFO_BYTE1, (uint8_t*) &reg1, 1);
	spiRegRead(pCtrl, FIFO_BYTE2, (uint8_t*) &reg2, 1);
	reg2 = reg2 & 0x7F;

	fifoLength = ((reg2 << 16) | (reg1 << 8) | reg0) & 0x007FFFFF;
	printf("FIFO Length : %lu\n", fifoLength);
	return fifoLength;
}

void cam_enable(ArducamController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pCSPort, pCtrl->csPinNo, GPIO_PIN_RESET);
}

void cam_disable(ArducamController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pCSPort, pCtrl->csPinNo, GPIO_PIN_SET);
}

void flashOn(ArducamController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pFlashPort, pCtrl->flashPinNo, GPIO_PIN_SET);
}

void flashOff(ArducamController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pFlashPort, pCtrl->flashPinNo, GPIO_PIN_RESET);
}

//Prints all of the relevant registers in the Arducam
void registerDump(ArducamController* pCtrl){
	uint8_t data;
	printf("Register Table:\n");
	/*printf("(I2C Read)\n");
	i2cRegRead(pCtrl, TEST_REGISTER, &data, 1);
	printf("Test Register: 0x%02X\n", data);
	i2cRegRead(pCtrl, CAPTURE_CONTROL_REG, &data, 1);
	printf("Capture Control Register: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_CONTROL_REG, &data, 1);
	printf("FIFO Control Register: 0x%02X\n", data);
	i2cRegRead(pCtrl, CHIP_VERSION_REG, &data, 1);
	printf("Chip Version: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_STATUS_REG, &data, 1);
	printf("FIFO Status Register: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_BYTE0, &data, 1);
	printf("FIFO Byte 0: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_BYTE1, &data, 1);
	printf("FIFO Byte 1: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_BYTE2, &data, 1);
	printf("FIFO Byte 2: 0x%02X\n", data);*/

	printf("(SPI Read)\n");
	spiRegRead(pCtrl, TEST_REGISTER, &data, 1);
	printf("Test Register: 0x%02X\n", data);
	spiRegRead(pCtrl, CAPTURE_CONTROL_REG, &data, 1);
	printf("Capture Control Register: 0x%02X\n", data);
	spiRegRead(pCtrl, FIFO_CONTROL_REG, &data, 1);
	printf("FIFO Control Register: 0x%02X\n", data);
	spiRegRead(pCtrl, CHIP_VERSION_REG, &data, 1);
	printf("Chip Version: 0x%02X\n", data);
	spiRegRead(pCtrl, FIFO_STATUS_REG, &data, 1);
	printf("FIFO Status Register: 0x%02X\n", data);
	spiRegRead(pCtrl, FIFO_BYTE0, &data, 1);
	printf("FIFO Byte 0: 0x%02X\n", data);
	spiRegRead(pCtrl, FIFO_BYTE1, &data, 1);
	printf("FIFO Byte 1: 0x%02X\n", data);
	spiRegRead(pCtrl, FIFO_BYTE2, &data, 1);
	printf("FIFO Byte 2: 0x%02X\n", data);

}

void printStatus(ArducamController* pCtrl){
	printf("**********Status Report**********\n");
	registerDump(pCtrl);
	printf("Arducam Status: %x\n", pCtrl->status);
	printf("*******************************\n");

}

