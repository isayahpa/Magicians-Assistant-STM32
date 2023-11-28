#include "WifiController.h"
#include "helpers.h"

//Commands - Each 8 bytes long
const char* LIGHTS_ON = "light_on";
const char* LIGHTS_OFF = "lightoff";
const char* SHUFFLE = "shuffle_";
const char* SNAP = "picture_";
const char* ARCHIDEKT = "archidekt";
const char* SHUTDOWN = "shutdown";

const char* STATUS_LIGHTS_ON = "Turning Lights On";
const char* STATUS_LIGHTS_OFF = "Turning Lights Off";
const char* STATUS_SHUFFLE = "Starting Shuffle";
const char* STATUS_SNAP = "Taking a Picture";
const char* STATUS_ARCHIDEKT = "Sending Deck to Archidekt";
const char* STATUS_SHUTDOWN = "Shutting Down";
const char* STATUS_UNKNOWN = "Unknown Action";

void initESP(WiFiController* pCtrl, UART_HandleTypeDef* pUARTHandle, GPIO_TypeDef* pGPIOPort, uint16_t readyFlagPin){
	pCtrl->pUARTHandle = pUARTHandle;
	pCtrl->pGPIOPort = pGPIOPort;
	pCtrl->readyFlagPin = readyFlagPin;
	pCtrl->status = HAL_OK;
}

void signalReady(WiFiController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pGPIOPort, pCtrl->readyFlagPin, GPIO_PIN_SET);
}

void signalBusy(WiFiController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pGPIOPort, pCtrl->readyFlagPin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef getNextCMD(WiFiController* pCtrl, char *pCMDBuffer){
	printf("Waiting for next CMD...\n");
	signalReady(pCtrl);
	HAL_Delay(100);
	pCtrl->status = HAL_UART_Receive(pCtrl->pUARTHandle, (uint8_t*)  pCMDBuffer, 8, HAL_MAX_DELAY); // might need to figure out how to decide how much data to read in
	printf("CMD FROM ESP: %s\n", pCMDBuffer);
	signalBusy(pCtrl);
	return pCtrl->status;
}

HAL_StatusTypeDef sendData(WiFiController* pCtrl, uint8_t* pDataBuffer, uint16_t size){
	printf("Sending %u bytes to ESP\n", size);
	pCtrl->status = HAL_UART_Transmit(pCtrl->pUARTHandle, pDataBuffer, size, ESP_TIMEOUT);
	return pCtrl -> status;
}
