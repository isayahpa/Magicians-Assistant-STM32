
#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include "stm32l4xx_hal.h"
#include <stdio.h>

int checkBit(uint8_t, int);
void initHelpers(UART_HandleTypeDef*, I2C_HandleTypeDef*);
char* statusToString(HAL_StatusTypeDef status);

#endif /* INC_HELPERS_H_ */
