
#include "ArducamController.h"
#include "helpers.h"
#include <stdlib.h>

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

	if(HAL_I2C_IsDeviceReady(pHI2C, I2C_ADDR_WRITE, 1, HAL_MAX_DELAY) == HAL_OK){
		resetCam(pCtrl);

		setDefaultSettings(pCtrl);
		flushFIFO(pCtrl);

		printStatus(pCtrl);
	} else {
		printf("Arducam I2C Wasn't Ready Yet\n");
	}

	if(pCtrl->status != HAL_OK){
		printf("Arducam Init Fail | Status = 0x%02X\n", pCtrl->status);
	} else {
		printf("Arducam Init Success!\n");
	}

}

//TODO: Make it so that when Register Writes/Reads fail (status != 00), we print error data and throw an interrupt? or maybe just halt the function?
// TODO: Find out why the status == 1 when we try I2C transmit
//
/*To Write over i2c:
* Request I2C_ADDR_WRITE -> Send Cam Register Address (left shift device addr's by 1) -> Send Data bytes
*/

void i2cRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){
	printf("Writing 0x%04X to Reg 0x%02X\n", *pData, reg);
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
	printf("Read 0x%02X from 0x%02X\n", *pBuffer, reg);
}

void i2cWriteMultiple(ArducamController* pCtrl, const struct SensorReg *regList){
	struct SensorReg *current = (struct SensorReg *) regList;

	while(current->addr != 0xFF || current->val != 0xFF){
		i2cRegWrite(pCtrl, current->addr, &(current->val), 1);
		current++;
	}

}

void spiRegWrite(ArducamController* pCtrl, uint8_t reg, uint8_t *pData, uint16_t size){
	printf("Writing 0x%02X to %02X\n", *pData, reg);
	enable(pCtrl); // CS Pin Set LOW
	//HAL_Delay(CAM_TIMEOUT);
	uint8_t cmdByte = reg | SPI_WRITE_MASK; // a 1 followed by Reg addr, to write to reg

	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, &cmdByte, 1, CAM_TIMEOUT);
	pCtrl->status = HAL_SPI_Transmit(pCtrl->pSPIHandle, pData, size, CAM_TIMEOUT);

	disable(pCtrl);
}

void spiRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	enable(pCtrl);
	//HAL_Delay(CAM_TIMEOUT);
	uint8_t cmdByte = reg & SPI_READ_MASK; // a 0 followed by register to read
	uint8_t dummyByte = 0;
	pCtrl->status = HAL_SPI_TransmitReceive(pCtrl->pSPIHandle, &cmdByte, pBuffer, 1, CAM_TIMEOUT);
	pCtrl->status = HAL_SPI_TransmitReceive(pCtrl->pSPIHandle, &dummyByte, pBuffer, size, CAM_TIMEOUT);
	//pCtrl->status = HAL_SPI_Receive(pCtrl->pSPIHandle, pBuffer, size, CAM_TIMEOUT);
	disable(pCtrl);
	printf("Read 0x%02X from %02X\n", *pBuffer, reg);
}

//Returns FIFO 'finished' flag. 0 -> FIFO is busy, 1 -> capture is finished

int isFIFOReady(ArducamController* pCtrl){
	uint8_t registerData = 0;
	i2cRegRead(pCtrl, FIFO_STATUS_REG, &registerData, 1);
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
/*It is a basic capture function of the ArduChip. The capture command code is 0x84, and write
‘1’ to bit[1] to start a capture sequence. And then polling bit[3] which is the capture done flag by
sending command code 0x41. After capture is done, user have to clear the capture done flag by
sending command code 0x41 and write ‘1’ into bit[0] before next capture command.*/

//Returns a pointer to the picture data
void singleCapture(ArducamController* pCtrl, uint8_t **ppBuffer, FILE *pPicFile){
		printf("Starting Capture\n");
		setCaptureFlag(pCtrl);
		while(!isFIFOReady(pCtrl)); // Wait 'til Finished Flag is set
		printf("FIFO Write Finished!\n");

		//uint32_t fifoLength = getFIFOLength(pCtrl);
		//uint32_t transmissionSize = 0;
		//if(fifoLength >= 4096){ transmissionSize = 4096; } else { transmissionSize = fifoLength; } //4096 looks like the max amount of data

		int bufferSize = 4096;
		*ppBuffer = calloc(bufferSize, sizeof(uint8_t));
		burstReadFIFO(pCtrl, *ppBuffer);

		printf("Capture Complete!\n");
		printf("Picture Buffer: \n");

		for(int i = 0; i < bufferSize; i++){
			printf("%0x02X", *ppBuffer[i]);
			if(!(i % 10)){printf("\n");}
		}

		if(pPicFile != NULL){
			fwrite((const void *)*ppBuffer, sizeof(uint8_t), bufferSize, pPicFile);
		}
}


void setDefaultSettings(ArducamController* pCtrl){
	printf("Configuring Default Settings\n");
	i2cWriteMultiple(pCtrl, OV2640_QVGA);
	i2cWriteMultiple(pCtrl, OV2640_JPEG_INIT);
	i2cWriteMultiple(pCtrl, OV2640_640x480_JPEG);
	flushFIFO(pCtrl);
	setNCaptureFrames(pCtrl, 1);
}

//Resets the CPLD
void resetCam(ArducamController* pCtrl){
	uint8_t cmd = 0x80;
	i2cRegWrite(pCtrl, 0x07, &cmd, 1);
	HAL_Delay(CAM_TIMEOUT);
	cmd = 0x00;
	i2cRegWrite(pCtrl, 0x07, &cmd, 1);
	HAL_Delay(CAM_TIMEOUT);
}


//I think i'll need to ignore the first dummy byte
void burstReadFIFO(ArducamController *pCtrl, uint8_t *pBuffer){
	enable(pCtrl);
	uint8_t cmd = FIFO_BURST_READ;
	HAL_SPI_TransmitReceive(pCtrl->pSPIHandle, &cmd, pBuffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(pCtrl->pSPIHandle, pBuffer, getFIFOLength(pCtrl), HAL_MAX_DELAY);
	HAL_Delay(1000); // Just making sure all the data makes it through
	disable(pCtrl);
}

void flushFIFO(ArducamController* pCtrl){
	printf("Flushing FIFO\n");
	clearFIFOFlag(pCtrl);
	printf("Reseting FIFO Pointers\n");
	resetFIFOPointers(pCtrl);
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
	return fifoLength;
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
	printf("Test Register: 0x%02X\n", data);
	i2cRegRead(pCtrl, CAPTURE_CONTROL_REG, &data, 1);
	printf("Capture Control Register: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_CONTROL_REG, &data, 1);
	printf("FIFO Control Register: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_SINGLE_READ_CMD, &data, 1);
	printf("FIFO Single Read Register: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_STATUS_REG, &data, 1);
	printf("FIFO Status Register: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_BYTE0, &data, 1);
	printf("FIFO Byte 0: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_BYTE1, &data, 1);
	printf("FIFO Byte 1: 0x%02X\n", data);
	i2cRegRead(pCtrl, FIFO_BYTE2, &data, 1);
	printf("FIFO Byte 2: 0x%02X\n", data);

	printf("(SPI Read)\n");
	spiRegRead(pCtrl, TEST_REGISTER, &data, 1);
	printf("Test Register: 0x%02X\n", data);
	spiRegRead(pCtrl, CAPTURE_CONTROL_REG, &data, 1);
	printf("Capture Control Register: 0x%02X\n", data);
	spiRegRead(pCtrl, FIFO_CONTROL_REG, &data, 1);
	printf("FIFO Control Register: 0x%02X\n", data);
	//spiRegRead(pCtrl, FIFO_SINGLE_READ_REG, &data, 1);
	//printf("FIFO Single Read Register: %x\n", data);
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
	printf("Arducam Status: %x\n", pCtrl->status);
	registerDump(pCtrl);
	printf("*******************************\n");

}

