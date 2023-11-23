#ifndef INC_WIFICONTROLLER_H_
#define INC_WIFICONTROLLER_H_

#include "stm32l4xx_hal.h"

typedef struct WiFiController WiFiController;

extern const char* LIGHTS_ON;
extern const char* LIGHTS_OFF;
extern const char* SHUFFLE;
extern const char* SNAP;
extern const char* ARCHIDEKT;
extern const char* SHUTDOWN;

extern const char* STATUS_LIGHTS_ON;
extern const char* STATUS_LIGHTS_OFF;
extern const char* STATUS_SHUFFLE;
extern const char* STATUS_SNAP;
extern const char* STATUS_ARCHIDEKT;
extern const char* STATUS_SHUTDOWN;
extern const char* STATUS_UNKNOWN;

struct WiFiController{

	UART_HandleTypeDef* pUARTHandle;
	GPIO_TypeDef* pGPIOPort;
	uint16_t readyFlagPin;

	HAL_StatusTypeDef status;
};

void initESP(WiFiController* pCtrl, UART_HandleTypeDef* pUARTHandle, GPIO_TypeDef* pGPIOPort, uint16_t readyFlagPin);
void signalReady(WiFiController* pCtrl);
void signalBusy(WiFiController* pCtrl);
HAL_StatusTypeDef getNextCMD(WiFiController* pCtrl, char* pCMDBuffer);


#endif /* INC_WIFICONTROLLER_H_ */
