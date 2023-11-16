
// Just some useful functions :)

#include "helpers.h"


//Returns the value of the bit at index in num
//Ex: checkBit(0x05, 2) == 2
int checkBit(unsigned char num, int index){
	return (int) ((num >> index) & 0x01);
}

// Allows for printf() use

UART_HandleTypeDef *pHuart;

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(pHuart, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void initHelpers(UART_HandleTypeDef* pUHandle){
	pHuart = pUHandle;
}

