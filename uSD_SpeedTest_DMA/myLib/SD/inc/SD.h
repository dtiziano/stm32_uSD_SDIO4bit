/*
 * SD.h
 *
 *  Created on: 16.02.2023
 *      Author: dipietro
 */

#ifndef APPLICATION_USER_MYLIB_SD_INC_SD_H_
#define APPLICATION_USER_MYLIB_SD_INC_SD_H_

#include "ff.h"
#include "fatfs.h"
#include "ffconf.h"
#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "myDiskio.h"
#include "main.h"

extern char SDFileName[13];
extern volatile int writeCounter;
extern int measurementInclination;

extern uint32_t idPart1;
extern uint32_t idPart2;
extern uint32_t idPart3;

bool isSDCardInserted();
void mountSDCard(FRESULT *fr, FATFS *SDFatFs);
void unMountSDCard(FRESULT *fr, FATFS *SDFatFs);
void openWriteModeSDCardFile(FRESULT *fr, char * fileName);
void formatSDCard(FRESULT *fr, uint8_t *buffer, UINT len );
void writeSDCardFile(FRESULT *fr, void * dataBuffer, UINT dataBufferLength, UINT *byteswritten);
void openReadModeSDCardFile(char * fileName);
void readSDCardFile(FRESULT *fr, void * dataBuffer, UINT dataBufferLength, UINT * bytesread);
void closeSDCardFile(FRESULT *fr) ;
uint16_t getSDFileNumber(void);
void setFileName(uint16_t *fileCounter,char *SDFileName) ;
void writeFileHeader(FRESULT *fr, UINT *byteswritten);


#endif /* APPLICATION_USER_MYLIB_SD_INC_SD_H_ */
