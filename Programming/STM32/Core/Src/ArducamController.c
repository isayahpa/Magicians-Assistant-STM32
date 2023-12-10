
#include "ArducamController.h"
#include "helpers.h"
#include <stdlib.h>
#include <string.h>

/* Notes:
 * The chip select signal should always be LOW during the SPI read or write bus cycle
 * I2C interfaces directly with the OV2640 sensor (the camera itself)
 * SPI interfaces with the Chip as a whole, to indirectly control the camera
 */
//void initArducam(ArducamController* pCtrl, I2C_HandleTypeDef* pHI2C, SPI_HandleTypeDef* pHSPI, GPIO_TypeDef* pCSPort, uint16_t csPinNo, GPIO_TypeDef* pFlashPort, uint16_t flashPinNo){
HAL_StatusTypeDef initArducam(ArducamController* pCtrl){
	printf("Initializing ArduCam\n");

	resetCPLD(pCtrl);
	if((pCtrl->status = HAL_I2C_IsDeviceReady(pCtrl->pI2CHandle, I2C_ADDR_WRITE, 1, HAL_MAX_DELAY)) != HAL_OK ||
			(pCtrl->status = HAL_I2C_IsDeviceReady(pCtrl->pI2CHandle, I2C_ADDR_READ, 1, HAL_MAX_DELAY != HAL_OK))){
		printf("Arducam I2C Error.\n");
	} else if(!isSPIWorking(pCtrl)){
		printf("Arducam SPI Error.\n");
	} else {
		printf("Arducam I2C Check Passed | SPI Check Passed\n");

		pCtrl->pictureBufferSize = 0;
		pCtrl->base64Size = 0;
		memset(pCtrl->pictureBuffer, 0, MAX_PIC_BUF_SIZE);
		memset(pCtrl->base64Buffer, 0, MAX_BASE64_BUF_SIZE);

		setDefaultSettings(pCtrl);
		HAL_Delay(1000);
		shutter(pCtrl);
	}



	if(pCtrl->status != HAL_OK){
		printf("FAILED Arducam Init\n");
	} else {
		printf("SUCCESS Arducam Init\n");
	}

	return pCtrl->status;

	//printStatus(pCtrl);
}

//Deconstructor
HAL_StatusTypeDef disconnectArducam(ArducamController* pCtrl){
	printf("Disconnecting Arducam Controller...\n");
	return pCtrl->status;
}

//Fills pictureBuffer with the Picture Data
HAL_StatusTypeDef singleCapture(ArducamController* pCtrl){
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

		if(burstReadFIFO(pCtrl) == 0){
			pCtrl->status = HAL_ERROR;
		} else {
			picToBase64(pCtrl);
		}

		if(pCtrl->status != HAL_OK){
			printf("FAILED Single Snap\n");

		} else {
			printf("SUCCESS Single Snap\n");
		}

		return pCtrl->status;
}

//Returns the amount of data (in bytes) read from FIFO
uint16_t burstReadFIFO(ArducamController *pCtrl){

	uint32_t fifoLength = getFIFOLength(pCtrl);
	uint32_t transmissionSize = fifoLength;
	if(fifoLength >= 0x5FFF){
		printf("Had to Truncate FIFO Transfer\n");
		transmissionSize = 0x5FFF;
	} else if(fifoLength == 0){
		printf("FAILED burst read FIFO, FIFO has no data\n");
		pCtrl->pictureBufferSize = 0;
		return 0;
	}

	//clearPicBuf(pCtrl);
	pCtrl->pictureBufferSize = transmissionSize; //TODO: Need to figure out how much data to buffer

	printf("Reading %u bytes from Arducam\n", pCtrl->pictureBufferSize);
	cam_enable(pCtrl);

	uint8_t cmd = FIFO_BURST_READ;
	pCtrl->status = HAL_SPI_TransmitReceive(pCtrl->pSPIHandle, &cmd, pCtrl->pictureBuffer, 1, HAL_MAX_DELAY);
	if(pCtrl->status != HAL_OK){
		printf("FAILED sending FIFO_BURST_READ byte | Error: %s\n", stat2Str(pCtrl->status));
	} else {
		pCtrl -> status = HAL_SPI_Receive(pCtrl->pSPIHandle, pCtrl->pictureBuffer, pCtrl->pictureBufferSize, HAL_MAX_DELAY); //Read bytes into pictureBuffer
		if(pCtrl->status != HAL_OK){
			printf("FAILED receiving picture data from SPI bus | Error: %s\n", stat2Str(pCtrl->status));
		}
	}

	cam_disable(pCtrl);
	return pCtrl->pictureBufferSize;
}

//How we find the number of bytes the FIFO is holding (for burst reading)
uint32_t getFIFOLength(ArducamController *pCtrl){
	uint8_t buf = 0x02;
	uint32_t reg0;
	uint32_t reg1;
	uint32_t reg2;
	uint32_t fifoLength = 0;

	reg0 = spiRegRead(pCtrl, FIFO_BYTE0, &buf, sizeof(uint8_t));
	reg1 = spiRegRead(pCtrl, FIFO_BYTE1, &buf, sizeof(uint8_t));
	reg2 = spiRegRead(pCtrl, FIFO_BYTE2, &buf, sizeof(uint8_t));
	reg2 = reg2 & 0x7F;
	uint32_t longReg0 = (uint32_t) reg0;
	uint32_t longReg1 = (uint32_t) reg1;
	uint32_t longReg2 = (uint32_t) reg2;

	fifoLength = ((longReg2 << 16) | (longReg1 << 8) | longReg0) & 0x007FFFFF;
	//fifoLength = ((reg2 << 16) | (reg1 << 8) | reg0) & 0x007FFFFF;
	printf("FIFO Length : %luKB\n", fifoLength/1024);
	return fifoLength;
}

//Clears the picture buffer
//Sets all bytes to 0xFF
void clearPicBuf(ArducamController* pCtrl){
	for(int i = 0; i < MAX_PIC_BUF_SIZE; i++){
		pCtrl->pictureBuffer[i] = 0xFF;
	}
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

uint8_t spiRegRead(ArducamController* pCtrl, uint8_t reg, uint8_t *pBuffer, uint16_t size){
	cam_enable(pCtrl);
	HAL_Delay(CS_DELAY);
	uint8_t maskedAddr = reg & SPI_READ_MASK; // a 0 followed by register to read
	uint8_t dummyByte = 0x00;
	pCtrl->status = HAL_SPI_TransmitReceive(pCtrl->pSPIHandle, &maskedAddr, pBuffer, (uint16_t)1, CAM_TIMEOUT);
	pCtrl->status = HAL_SPI_TransmitReceive(pCtrl->pSPIHandle, &dummyByte, pBuffer, size, CAM_TIMEOUT);
	cam_disable(pCtrl);
	printf("(SPI) Read 0x%02X from 0x%02X\n", *pBuffer, reg);
	return *pBuffer;
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
	clearFIFOFlag(pCtrl);
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

void cam_enable(ArducamController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pCSPort, pCtrl->csPinNo, GPIO_PIN_RESET);
}

void cam_disable(ArducamController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pCSPort, pCtrl->csPinNo, GPIO_PIN_SET);
}

void shutter(ArducamController* pCtrl){
	flashOn(pCtrl);
	HAL_Delay(SHUTTER_DELAY);
	flashOff(pCtrl);
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

const char BASE64LOOKUPTABLE[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

//Fills the base64 buffer
void picToBase64(ArducamController* pCtrl){
  printf("Converting data to Base64...\n");

  uint8_t *inputBytes = pCtrl->pictureBuffer;
  int nBytes = pCtrl->pictureBufferSize;

  char *output = pCtrl->base64Buffer;

  uint32_t byte0, byte1, byte2;
  uint8_t base64Digit0, base64Digit1, base64Digit2, base64Digit3;
  uint8_t tableIdx0, tableIdx1, tableIdx2, tableIdx3;
  uint32_t threeByteCombo;

  int inputIdx = 0;
  int outputIdx = 0;
  while(inputIdx < nBytes-2){
    byte0 = inputBytes[inputIdx];
    byte1 = inputBytes[inputIdx+1];
    byte2 = inputBytes[inputIdx+2];
    threeByteCombo = ((byte0 << 16) | (byte1 << 8) | byte2); // 24 bits

    // Turn every 6 bits into a Base64 digit
    tableIdx0 = (threeByteCombo >> 18);
    tableIdx1 = (threeByteCombo >> 12) & 0x3F;
    tableIdx2 = (threeByteCombo >> 6) & 0x3F;
    tableIdx3 = threeByteCombo & 0x3F;

    base64Digit0 = BASE64LOOKUPTABLE[tableIdx0];
    base64Digit1 = BASE64LOOKUPTABLE[tableIdx1];
    base64Digit2 = BASE64LOOKUPTABLE[tableIdx2];
    base64Digit3 = BASE64LOOKUPTABLE[tableIdx3];

    output[outputIdx] = (char) base64Digit0;
    output[outputIdx + 1] = (char) base64Digit1;
    output[outputIdx + 2] = (char) base64Digit2;
    output[outputIdx + 3] = (char) base64Digit3;

    inputIdx += 3;
    outputIdx += 4;
  }

  // Padding if necessary
  if((nBytes-1)%3 == 2){

    byte0 = inputBytes[nBytes-2];
    byte1 = inputBytes[nBytes-1];
    byte2 = 0; //Will need to pad here
    threeByteCombo = ((byte0 << 16) | (byte1 << 8) | byte2); // 24 bits

    // Turn every 6 bits into a Base64 digit
    tableIdx0 = (threeByteCombo >> 18);
    tableIdx1 = (threeByteCombo >> 12) & 0x3F;
    tableIdx2 = (threeByteCombo >> 6) & 0x3F;
    tableIdx3 = threeByteCombo & 0x3F;

    base64Digit0 = BASE64LOOKUPTABLE[tableIdx0];
    base64Digit1 = BASE64LOOKUPTABLE[tableIdx1];
    base64Digit2 = BASE64LOOKUPTABLE[tableIdx2];
    base64Digit3 = '=';

    output[outputIdx] = (char) base64Digit0;
	output[outputIdx + 1] = (char) base64Digit1;
	output[outputIdx + 2] = (char) base64Digit2;
	output[outputIdx + 3] = (char) base64Digit3;
  } else if ((nBytes-1)%3 == 1){
    byte0 = inputBytes[nBytes-1];
    byte1 = 0; //Will need to pad here
    byte2 = 0;
    threeByteCombo = ((byte0 << 16) | (byte1 << 8) | byte2); // 24 bits

    // Turn every 6 bits into a Base64 digit
    tableIdx0 = (threeByteCombo >> 18);
    tableIdx1 = (threeByteCombo >> 12) & 0x3F;
    tableIdx2 = (threeByteCombo >> 6) & 0x3F;
    tableIdx3 = threeByteCombo & 0x3F;

    base64Digit0 = BASE64LOOKUPTABLE[tableIdx0];
    base64Digit1 = BASE64LOOKUPTABLE[tableIdx1];
    base64Digit2 = '=';
    base64Digit3 = '=';

    output[outputIdx] = (char) base64Digit0;
	output[outputIdx + 1] = (char) base64Digit1;
	output[outputIdx + 2] = (char) base64Digit2;
	output[outputIdx + 3] = (char) base64Digit3;
  }

  printf("Base 64 Data = %s\n", output);
}

