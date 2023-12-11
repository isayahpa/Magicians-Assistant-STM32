#include "WifiController.h"
#include "helpers.h"

//Commands - Each 8 bytes long
const char LIGHTS_ON[] = "light_on";
const char LIGHTS_OFF[] = "lightoff";
const char SHUFFLE[] = "shuffle_";
const char SNAP[] = "picture_";
const char ARCHIDEKT[] = "archidekt";
const char SD_READ[] = "read_sd_";
const char SHUTDOWN[] = "shutdown";

const char* STATUS_LIGHTS_ON = "Turning Lights On\n";
const char* STATUS_LIGHTS_OFF = "Turning Lights Off\n";
const char* STATUS_SHUFFLE = "Starting Shuffle\n";
const char* STATUS_SNAP = "Taking a Picture\n";
const char* STATUS_ARCHIDEKT = "Sending Deck to Archidekt\n";
const char* STATUS_SD_READ = "ESP Requesting SD Data\n";
const char* STATUS_SHUTDOWN = "Shutting Down\n";
const char* STATUS_UNKNOWN = "Unknown Action";

HAL_StatusTypeDef initESP(WiFiController* pCtrl){
	printf("Initialzing WiFi Controller...\n");
	signalBusy(pCtrl);
	return pCtrl->status;
}

HAL_StatusTypeDef disconnectESP(WiFiController* pCtrl){
	printf("Disconnecting WiFi Controller...\n");
	signalBusy(pCtrl);
	clearCMDBuffer(pCtrl);
	return pCtrl->status;
}

void signalReady(WiFiController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pGPIOPort, pCtrl->readyFlagPin, GPIO_PIN_SET);
}

void signalBusy(WiFiController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pGPIOPort, pCtrl->readyFlagPin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef getNextCMD(WiFiController* pCtrl, char *pCMDBuffer){
	HAL_StatusTypeDef cmdStatus = HAL_OK;
	printf("Waiting for next CMD...\n");
	signalReady(pCtrl);
	HAL_Delay(ESP_READY_DELAY); // Wait for ESP to send the data
	cmdStatus = HAL_UART_Receive(pCtrl->pUARTHandle, (uint8_t*) pCMDBuffer, 8, ESP_CMD_TIMEOUT); // Waits 60s for a CMD, then yields control
	switch(cmdStatus){
		case HAL_OK:
			printf("CMD: %s\n", pCMDBuffer);
			break;
		case HAL_BUSY:
			printf("CMD Buffer (RX) Busy\n");
			break;
		case HAL_ERROR:
			printf("Error reading CMD from ESP\n");
			pCtrl->status = HAL_ERROR;
			break;
		case HAL_TIMEOUT:
			printf("Timed out waiting for CMD\n");
			break;
	}

	signalBusy(pCtrl);
	return cmdStatus;
}

// Cleans out the ESP_RX buffer
// Note: Using this in the loop removes the ability to make a "Command Queue", so the buffer can only ever have one CMD at a time
void clearCMDBuffer(WiFiController* pCtrl){
	uint8_t byteRead = 0;
	HAL_StatusTypeDef status = HAL_OK;
	while(status == HAL_OK){
		status = HAL_UART_Receive(pCtrl->pUARTHandle, &byteRead, 1, 0); //Should become HAL_TIMEOUT when there is no data left
	}
}

HAL_StatusTypeDef sendData(WiFiController* pCtrl, void* pDataBuffer, uint16_t size){
	printf("Sending %u bytes to ESP\n", size);
	pCtrl->status = HAL_UART_Transmit(pCtrl->pUARTHandle, (const uint8_t*) pDataBuffer, size, HAL_MAX_DELAY);
	return pCtrl -> status;
}
