#ifndef INC_WIFICONTROLLER_H_
#define INC_WIFICONTROLLER_H_

#include "stm32l4xx_hal.h"

typedef struct WiFiController WiFiController;

extern const char LIGHTS_ON[];
extern const char LIGHTS_OFF[];
extern const char SHUFFLE[];
extern const char SNAP[];
extern const char ARCHIDEKT[];
extern const char SD_READ[];
extern const char SHUTDOWN[];

extern const char* STATUS_LIGHTS_ON;
extern const char* STATUS_LIGHTS_OFF;
extern const char* STATUS_SHUFFLE;
extern const char* STATUS_SNAP;
extern const char* STATUS_ARCHIDEKT;
extern const char* STATUS_SD_READ;
extern const char* STATUS_SHUTDOWN;
extern const char* STATUS_UNKNOWN;

#define ESP_READY_DELAY 100
#define ESP_CMD_TIMEOUT 60000

struct WiFiController{
	HAL_StatusTypeDef status;
	UART_HandleTypeDef* pUARTHandle;
	GPIO_TypeDef* pGPIOPort;
	uint16_t readyFlagPin;
};

HAL_StatusTypeDef initESP(WiFiController* pCtrl);
HAL_StatusTypeDef disconnectESP(WiFiController* pCtrl);
void signalReady(WiFiController* pCtrl);
void signalBusy(WiFiController* pCtrl);
HAL_StatusTypeDef getNextCMD(WiFiController* pCtrl, char* pCMDBuffer);
void clearCMDBuffer(WiFiController* pCtrl);
HAL_StatusTypeDef sendData(WiFiController* pCtrl, void* pDataBuffer, uint16_t size);

#endif /* INC_WIFICONTROLLER_H_ */
