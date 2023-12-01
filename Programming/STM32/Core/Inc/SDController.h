#ifndef INC_SDCONTROLLER_H_
#define INC_SDCONTROLLER_H_

#include "stm32l4xx_hal.h"

typedef struct SDController SDController;
struct SDController {
	HAL_StatusTypeDef status;
	SPI_HandleTypeDef* pSPIHandle;
	uint16_t csPin;
	GPIO_TypeDef* pCSPort;
};


void sd_enable(SDController* pCtrl);
void sd_disable(SDController* pCtrl);
#endif /* INC_SDCONTROLLER_H_ */
