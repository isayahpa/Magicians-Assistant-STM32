#include "SDController.h"
#include "helpers.h"

// Credit to : https://01001000.xyz/2020-08-09-Tutorial-STM32CubeIDE-SD-card/ for this tutorial

/* This Controller works differently than the rest b/c
*It's important that the SD FS code is singleton
*(We can only have one open at a time)
*/

FATFS fatFS; 	//Fatfs handle
FIL file; 		//File handle
FRESULT fres; //Result after operations

HAL_StatusTypeDef initSD(SDController* pCtrl){
	printf("Initializing SD Controller...\n");

	// Mount the File System
	if(mountFS(pCtrl) != HAL_OK){
		printf("FAILED SD Init\n | Status : %s\n", stat2Str(pCtrl->status));
	} else {
		printf("SUCCESS SD Initialized\n");
		getSDStats();
	}

	return pCtrl->status;
}

HAL_StatusTypeDef disconnectSD(SDController* pCtrl){
	printf("Disconnecting SD Card...\n");
	if(dismountFS(pCtrl) != HAL_OK){	//Took out closeFile() from here since it should be closed if(closeFile(pCtrl) && ...)
		printf("FAILED Disconnecting SD\n");
	} else {
		printf("SUCCESS SD Disconnected\n");
	}

	return pCtrl->status;
}

HAL_StatusTypeDef mountFS(SDController *pCtrl){
	fres = f_mount(&fatFS, "", 1);
	if(fres != FR_OK){
		printf("FAILED mounting FS | FRESULT : %i\n", fres);
		pCtrl->status = HAL_ERROR;
	} else {
		printf("SUCCESS File System mounted\n");
	}
	return pCtrl->status;
}

HAL_StatusTypeDef dismountFS(SDController* pCtrl){
	fres = f_mount(NULL, "", 0);
	if(fres != FR_OK){
		printf("FAILED dismounting FS | FRESULT : %i\n", fres);
		pCtrl->status = HAL_ERROR;
	} else {
		printf("SUCCESS File System mounted\n");
	}
	return pCtrl->status;
}

//Prints SD FS Stats
void getSDStats(){
	DWORD freeClusters, freeSectors, totalSectors;
	FATFS* pFreeFS;

	fres = f_getfree("", &freeClusters, &pFreeFS);
	if(fres != FR_OK){
		printf("FAILED to get SD stats | FRESULT : %i\n", fres);
		return;
	}

	totalSectors = (pFreeFS->n_fatent - 2) * pFreeFS->csize;
	freeSectors = freeClusters * pFreeFS->csize;

	printf("SD Stats:\n%10lu KB Total Capacity\n%10lu KB Free Space.\r\n", totalSectors / 2, freeSectors / 2);
}

HAL_StatusTypeDef openFile(SDController* pCtrl, char* filename, BYTE perms){
	fres = f_open(&file, (const TCHAR*) filename, perms); // See if this mask works or if i have to split this over reads/writes
	if (fres != FR_OK) {
		printf("FAILED to open '%s' | FRESULT : %i\n", filename, fres);
		pCtrl -> status = HAL_ERROR;
	} else {
		printf("SUCCESS opening '%s'\n", filename);
	}

	return pCtrl->status;
}

// Remember that there is only ever one file open at once
// So you are reading from whichever file that is
HAL_StatusTypeDef readFile(SDController* pCtrl, char* filename, char *dataBuf, UINT length){
	openFile(pCtrl, filename, FA_READ);

	char* pResult = f_gets((TCHAR*)dataBuf, length, &file);
	if(pResult == 0){
		printf("FAILED to read %d bytes from active file\n", length);
		pCtrl->status = HAL_ERROR;
	} else {
		printf("SUCCESS read %d bytes from active file\n", length);
	}

	closeFile(pCtrl);
	return pCtrl->status;
}

HAL_StatusTypeDef writeFile(SDController *pCtrl, char *filename, void *dataToWrite, UINT length){
	UINT bytesWrote;

	openFile(pCtrl, filename, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	fres = f_write(&file, (const void*) dataToWrite, length, &bytesWrote);
	if(fres != FR_OK) {
		printf("FAILED writing %d bytes to '%s' | \n", length, filename);
		pCtrl->status = HAL_ERROR;
	} else {
		printf("SUCCESS writing %d bytes to '%s'\n", bytesWrote, filename);
	}

	closeFile(pCtrl);
	return pCtrl->status;
}

HAL_StatusTypeDef closeFile(SDController* pCtrl){
	fres = f_close(&file);
	if(fres != FR_OK){
		printf("FAILED to close file | FRESULT : %i\n", fres);
		pCtrl->status = HAL_ERROR;
	} else {
		printf("SUCCESS closing file\n");
	}

	return pCtrl->status;
}

void sd_enable(SDController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pCSPort, pCtrl->csPin, GPIO_PIN_RESET);
}

void sd_disable(SDController* pCtrl){
	HAL_GPIO_WritePin(pCtrl->pCSPort, pCtrl->csPin, GPIO_PIN_SET);
}
