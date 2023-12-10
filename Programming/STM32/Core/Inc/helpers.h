
#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include "stm32l4xx_hal.h"
#include <stdio.h>

int checkBit(uint32_t num, int bitNo);
void initHelpers(UART_HandleTypeDef* pHUART);
void i2cScan(I2C_HandleTypeDef *pHI2C);
char* stat2Str(HAL_StatusTypeDef status);
void printStatus(void* pCtrl);
#endif /* INC_HELPERS_H_ */
