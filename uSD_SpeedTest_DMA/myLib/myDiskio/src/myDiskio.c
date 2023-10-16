/*
 * myDiskio.c
 *
 *  Created on: May 2, 2023
 *      Author: dipietro
 */

#include "myDiskio.h"


extern Disk_drvTypeDef  disk;


DSTATUS disk_reinitialize (
    BYTE pdrv               /* Physical drive number to identify the drive */
)
{
  disk.is_initialized[pdrv] = 0 ;
  //return disk_initialize( pdrv ) ;
  return RES_OK;
}
