#ifndef INC_SDCONTROLLER_H_
#define INC_SDCONTROLLER_H_

#include "stm32l4xx_hal.h"
#include "fatfs.h"

typedef struct SDController SDController;
struct SDController {
	HAL_StatusTypeDef status;
	SPI_HandleTypeDef* pSPIHandle;
	GPIO_TypeDef* pCSPort;
	uint16_t csPin;
};

HAL_StatusTypeDef initSD(SDController* pCtrl);
HAL_StatusTypeDef disconnectSD(SDController* pCtrl);
void getSDStats();

HAL_StatusTypeDef mountFS(SDController* pCtrl);
HAL_StatusTypeDef dismountFS(SDController* pCtrl);

HAL_StatusTypeDef openFile(SDController* pCtrl, char* filename, BYTE perms);
HAL_StatusTypeDef readFile(SDController* pCtrl, char* filename, char *dataBuf, UINT length);
HAL_StatusTypeDef writeFile(SDController *pCtrl, char *filename, void *dataToWrite, size_t length);
HAL_StatusTypeDef closeFile(SDController* pCtrl);

void sd_enable(SDController* pCtrl);
void sd_disable(SDController* pCtrl);
#endif /* INC_SDCONTROLLER_H_ */
