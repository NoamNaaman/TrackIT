/**
******************************************************************************
* File Name          : nor_flash.h
* Date               : 01/9/2019 
* Description        : NOR Flash driver
* Author	     : Lev Zoosmanovskiy
******************************************************************************

******************************************************************************
*/

#ifndef __NOR_FLASH_H
#define __NOR_FLASH_H

//#include "ll_includes.h"
#include "main.h"
#include "nor_flash.h"
#include "spi.h"
//-- external serial nor flash definition --//
#define FLASH_SPI                     SPI1

#define FLASH_SPI_SERVICE_INFO        (4)
#define FLASH_SPI_4SERVICE_INFO       (5)
#define FLASH_SPI_CMND_SERVICE_INFO   (1)

#define FLASH_SPI_OFFSET_CMND         (0)
#define FLASH_SPI_OFFSET_ADD2         (1) // MSB
#define FLASH_SPI_OFFSET_ADD1         (2)
#define FLASH_SPI_OFFSET_ADD0         (3)
#define FLASH_SPI_OFFSET_DATA         (4)

#define FLASH_SPI_4OFFSET_ADD3        (1) // MSB
#define FLASH_SPI_4OFFSET_ADD2        (2) 
#define FLASH_SPI_4OFFSET_ADD1        (3)
#define FLASH_SPI_4OFFSET_ADD0        (4)
#define FLASH_SPI_4OFFSET_DATA        (5)

//-- 4 bytes address commands --//
#define FLASH_4WRITE_PAGE_CMND        (0x12)
#define FLASH_4READ_DATA_CMND         (0x13)
#define FLASH_4ERASE_SUBSCT4K_CMND    (0x21) // 50mS
#define FLASH_4ERASE_SUBSCT32K_CMND   (0x5C) // 100mS
#define FLASH_4ERASE_SCT_CMND         (0xDC) // 150mS
#define FLASH_4ERASE_BULK_CMND        (0x60) // 150mS

//-- 3 bytes address commands --//
#define FLASH_READ_CHIP_ID            (0x9F)
#define FLASH_WRITE_ST_CMND           (0x01)
#define FLASH_READ_ST_CMND            (0x05)
#define FLASH_READ_FLST_CMND          (0x70)
#define FLASH_ERASE_DIE_CMND          (0xC4)
#define FLASH_ERASE_SUBSCT_CMND       (0x20)
#define FLASH_ERASE_SCT_CMND          (0xD8)
#define FLASH_WRITE_PAGE_CMND         (0x02)
#define FLASH_READ_DATA_CMND          (0x03)
#define FLASH_WRITE_ENB_CMND          (0x06)
#define FLASH_WRITE_DIS_CMND          (0x04)

#define FLASH_ENTER_4BYTE_ADD         (0xB7)
#define FLASH_EXIT_4BYTE_ADD          (0xE9)
#define FLASH_READ_NVOL_CNFG          (0xB5)
#define FLASH_WRITE_NVOL_CNFG         (0xB1)
#define FLASH_READ_VOL_CNFG           (0x85)
#define FLASH_WRITE_VOL_CNFG          (0x81)
#define FLASH_ENTER_DEEP_SLEEP        (0xB9)
#define FLASH_RELEASE_FROM_DEEP       (0xAB)

#define SZ_STREAM_BLOCK               (0x80)
#define SZ_STREAM_PKT                 (0x80)
#define SZ_NOR_FLASH_PAGE             (0x100)
#define SZ_NOR_FLASH_SUB_SCT          (0x1000)

#define NOR_FLASH_DEFAULT_VALUE       (0xFF)

#define NOR_FLASH_ST_BIT_WRBUSY       (0x01)
#define NOR_FLASH_ST_BIT_WRENB        (0x02)

typedef enum
{
  eNorIdle,
  eNorStartRead,  // start read through SPI3 block data
  eNorStopRead,   // stop read through SPI3 block data
  eNorStartErase, // start sending to SPI3 request to erase sub sector 4KB
  eNorStopErase,  // erase request was sent to SPI3, waiting bit WR_BUSY low
  eNorStartWrite, // start sending to SPI3 request to write one page 256B
  eNorStopWrite   // write page request was sent to SPI3, waiting bit WR_BUSY low
}NOR_STATE;

typedef enum
{
  eNorReadNone,
  eNorReadRqst,  
  eNorReadStart,   
  eNorReadSend   
} NOR_READ_PROCS;

typedef struct {
u8              u8NorFlashFirstBuff [SPI_BUFF_SIZE];
u8              u8NorFlashSecondBuff[SPI_BUFF_SIZE];
u8*             pNorFlashBuff;
u32             u32NorFlashFrame;
u16             u16CountTimeNextErase;
NOR_READ_PROCS  eNorReadProcs;
NOR_STATE       eNorFlashState;
u32             u32NbrErasedSectors;
u32             u32SetNorFlashAdd;
u32             u32GetNorFlashAdd;
u32             u32NorFlashSize;
u32             u32SemNorReadTime;
bool            bSemNorFlashErase;
bool            bSemNorFlashWrite; 
bool            bSemNorFlashPause; 
bool            bSemNorFlashBuffReady;
bool            bSemNorFlashFull;
} NOR_FLASH_STRUCT;

extern volatile bool g_bSemSpiOneBusy;
extern __no_init volatile NOR_FLASH_STRUCT nor_struct;

void  FlashReadChipId(u8* pB);
void  NorFlashWriteRegCmnd(u8 u8Cmnd);
void init_nor_flash(void);

#endif //__NOR_FLASH_H