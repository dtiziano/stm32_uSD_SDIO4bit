/*
 * SD.c
 *
 *  Created on: 16.02.2023
 *      Author: dipietro
 */


#include "SD.h"

extern volatile int writeCounter;
extern SD_HandleTypeDef hsd;

bool isSDCardInserted()
{
	GPIO_PinState cardInserted =  HAL_GPIO_ReadPin(uSD_Detect_GPIO_Port, uSD_Detect_Pin);
	if(cardInserted == GPIO_PIN_RESET)
	{
		return true;
	}else{
		return false;
	}
}

void mountSDCard(FRESULT *fr, FATFS *SDFatFs) {
	if(isSDCardInserted())
	{
		if (HAL_SD_Init(&hsd) != HAL_OK)
		{
			Error_Handler();
		}

		if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
		{
			Error_Handler();
		}
		/*
		 * reinitialize to make sure the sd can be mounted several times
		 */
		disk_reinitialize(SDFatFs->drv);

		/*
		 *  check if mount was successful
		 */
		if ((*fr = f_mount(SDFatFs, (TCHAR const * ) SDPath, 0)) != FR_OK)
		{
			MX_FATFS_DeInit();
			*fr = FR_DISK_ERR;
		}
	}else
	{
		*fr = FR_NOT_READY;
	}
}
void unMountSDCard(FRESULT *fr, FATFS *SDFatFs) {
    /*
     *  check if mount was successful
     */
    if ((*fr = f_mount(NULL, (TCHAR
            const * ) SDPath, 1)) != FR_OK) {
        //TODO: Error Handling
        //Error_Handler();
    }
}
void openWriteModeSDCardFile(FRESULT *fr, char * fileName) {
    /*
     * check if opening was successful
     */
    if ((*fr = f_open( & SDFile, fileName, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK) {
        //TODO: Error Handling
        //Error_Handler();
    }
}
void formatSDCard(FRESULT *fr, uint8_t *buffer, UINT len ) {
    /*
     * check if formatting was succesful
     */
    if ((*fr = f_mkfs((TCHAR
            const * ) SDPath, FM_ANY, 0, buffer, len)) != FR_OK) {
        //TODO: Error Handling
        //Error_Handler();
    }
}
/*
 * write a buffer to the open file on SD card
 * dataBuffer: pointer to the data buffer
 * dataBufferLength: is the number of bytes to be written
 */
void writeSDCardFile(FRESULT *fr, void * dataBuffer, UINT dataBufferLength, UINT *byteswritten)
{
	writeCounter++;
    *fr = f_write( &SDFile, dataBuffer, dataBufferLength, byteswritten);
    if ((byteswritten == 0) || (*fr != FR_OK)) {
        //TODO: Error Handling
        //Error_Handler();
    	__NOP();
    }
}
void openReadModeSDCardFile(char * fileName)
{
    if (f_open( &SDFile, fileName, FA_READ) != FR_OK)
    {
        /* 'STM32.TXT' file Open for read Error */
        //Error_Handler();
    }
}
void readSDCardFile(FRESULT *fr, void * dataBuffer, UINT dataBufferLength, UINT * bytesread)
{
    *fr = f_read( &SDFile, dataBuffer, dataBufferLength, bytesread);

    if ((bytesread == 0) || (fr != FR_OK)) {
        /* 'STM32.TXT' file Read or EOF Error */
        //Error_Handler();
    }
}
void closeSDCardFile(FRESULT *fr)
{
    /*
     * check if closing was successful
     */
    if ((*fr = f_close( & SDFile)) != FR_OK)
    {
        //TODO: Error Handling
        //Error_Handler();
    }
}
uint16_t getSDFileNumber(void) {
    uint16_t fileCounter = 0;
    FRESULT fr; /* Return value */
    DIR dj; /* Directory object */
    FILINFO fno; /* File information */


    fr = f_findfirst(&dj, &fno, "", "*.bin"); /* Start to search for SSP files */

    while (fr == FR_OK && fno.fname[0]) {
        /* Repeat while an item is found */
        //printf("%s\n", fno.fname);                /* Print the object name */
        fr = f_findnext(&dj, &fno); /* Search for next item */
        fileCounter++;
    }

    f_closedir(&dj);
    return fileCounter;
}
void setFileName(uint16_t *fileCounter,char *SDFileNamePtr)
{
    /*
     * check how many files are present on the SD card
     */
    *fileCounter = getSDFileNumber() + 1;

    sprintf(SDFileNamePtr, "FIL%05i.bin", *fileCounter);

}


















