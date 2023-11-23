
// Just some useful functions :)

#include "helpers.h"

UART_HandleTypeDef *pHUART;
I2C_HandleTypeDef *pHI2C;

void initHelpers(UART_HandleTypeDef* pUHandle, I2C_HandleTypeDef *pIHandle){
	pHUART = pUHandle;
	pHI2C = pIHandle;
}

//Returns the value of the bit at index in num
//Ex: checkBit(0x05, 2) == 2
int checkBit(uint8_t num, int index){
	return (int) ((num >> index) & 0x01);
}

// Allows for printf() use
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(pHUART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void i2cScan(){
	HAL_StatusTypeDef ret;
	// I2C Scan
	for(int i = 0; i < 128; i++){
		ret = HAL_I2C_IsDeviceReady(pHI2C, (uint16_t)i, 3, 10000);
		if(ret == HAL_OK){
			printf("Device found at Addr 0x%02X", i);
		}
	}
}


