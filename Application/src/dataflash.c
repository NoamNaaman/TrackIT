///////////////////////////////////////////////////////////////////////////////
// Source file nor_flash_opt.c
//
// Author:  Evgeny Kofman
// Modified by: Noam Naaman
// email:   noam@cardiacsense.com
// Copyright © 2018, Evgeny Kofman, CardiacSense Ltd.
//
// Version : 11.01.02.XX
// Added: lptm, adc, loader, adpd1, adpd2, 6d, ble, uart
///////////////////////////////////////////////////////////////////////////////

#include "stm32F1xx_hal.h"
#include "string.h"
#include "setup.h"
#include "main.h"
#include "serial.h"
#include "intrinsics.h"
#include "spi_one.h"
#include "nor_flash_v12.h"


#define FLUSH_DELAY_CYCLES 4

typedef enum {
    SPI_FLASH_SRWD		= 0x80,		/* Status Register Write Protect */
    SPI_FLASH_BP3		= 0x40,		/* Block Protect Bit3 */
    SPI_FLASH_TB		= 0x20,		/* Top/Bottom bit */
    SPI_FLASH_BP2		= 0x10,		/* Block Protect Bit2 */
    SPI_FLASH_BP1		= 0x08,		/* Block Protect Bit1 */
    SPI_FLASH_BP0		= 0x04,		/* Block Protect Bit0 */
    SPI_FLASH_WEL		= 0x02,		/* Write Enable Latch */
    SPI_FLASH_WIP		= 0x01		/* Write/Program/Erase in progress bit */
} SPI_FLASH_STATUS_BIT_MASKS;

void osDelay(u32);

u16 g_u16PpgPageNumber;

u32 gt_u32FlashEcgPages,gt_u32FlashPpgPages, g_u32FlashReadRequest, g_u32FlashAddressRequest;
u32 g_u32DontInitFlashSize;
bool g_bSemNorFlashOn;
u32 g_u32FillingPage = 0, g_u32LatestReportedVpeak, g_u32PpgPeakTimeDiff;
u32 g_u32ReportedPeakCount, gt_u32DeletedPeakCount, g_u32TickDiff;
u32 gt_u32DoublePeak, gt_u32TooCloseToBadTime;
u32 gt_u32PeakReverseTimestamps[256], gt_u32ReverseTS;
u32 gt_u32PeakLatestTimestamps[256];
u32 gt_UpdateVerifiedPeakTime, gt_u32PeakTransferred, gt_PpgTwoOrMorePeaks;
u32 gt_NewEcgPeaks, gt_PassedEcgPeaks, g_u32RecordSampleCounter;
u32 gt_u32PpgDtOverwrites, V12_DebugIndex, g_u32EcgStoreIndex;
static volatile u8 g_u8FlashPrefetchBuffer[SZ_NOR_FLASH_PAGE];
u16 g_u16NorFlashRegState;
u32 gt_u32PrevStoreTime, g_u32FlashReadRetry, gt_u32CountFlashRetries;

u32 gt_FlashErrorAddr[128], gt_FlashErrorAddrX;
u32 g_u32LatestPpgFftTime, g_u32SumEngAcc, g_u32RecordingOnOff;

bool g_bSemGetEraseState, g_bSemBleFlashErase;

__no_init bool g_bIisRecording;

volatile u8 *g_pFlashReadDestPtr;


u32 g_u32PageDataIndex;
u32 g_u32NorPageIndex, g_u32EcgPagePeakX, g_u32BpIndex;
u32 g_u32PpgPagePeakX, g_u32CurrentTick, gt_u32FlashErrors;

__no_init u32 g_u32NorFlashChecksumErrors;

u32 gt_u32CrcErrLoc[512], g_u32CRCerrors;
u32 gt_u32Ecg0Written,gt_u32Ecg1Written;

u32 g_u32FlshDataUploadInProgressCtr, g_u32FlashPrefetches;
u32 g_u32ExtraBits, g_u32LatestSuspectPeak, g_u32LatestVerifiedPeak;

u32 gt_u32PpgInput[60];

bool g_bFlashOperatingOK, g_bTestFlashIntegrity, g_bFirstRecBlock, g_bStoreHeaderTwice;

__no_init volatile NOR_FLASH_STRUCT nor_struct;
__no_init u32 nor_struct_guard;

bool g_bVibrateOnFullFlash;
u32 g_u32CountZeroDate;

bool g_bComputeCRC;
u16  g_u16ReadCrcIndex;
u32 gt_u32FlashReloadCounter;



u8   V12_u8FlashPagePrefetchBuffer[SZ_NOR_FLASH_PAGE];
u32  V12_u32FlashPagePrefetchAddress, g_u32FlushDelay;
bool V12_bFlashPagePrefetchDone, g_bFlushPages;
u32  g_u32PpgPageFlagIndex,g_u32EcgPageFlagIndex;
u32  g_u32PpgPageFlagCounter,g_u32EcgPageFlagCounter;
u32  g_u32FlashReadAddr, g_u32FlashPrefetchAddr = 0xFFFFFFFF;
u32 g_u32LatestRR;//, s_u32milliSecs;

u32 gt_u32DiscardedPeaks, gt_u32PassedPeaks, gt_MarkedPeaks;

u32 g_u32FlashEraseCtr, g_u32AddPeaks;

char g_cNorRecordReason;

FLUSH_STATES g_tFlushMachineState = FLUSH_STATE_IDLE;

bool g_bEcgActive, g_bDbgActive, g_bDbgActive2;

u16 HeaderPageChecksum, DebugPageChecksum0, DebugPageChecksum1, PpgPageChecksum, EcgPageChecksum[2];//, BpPageChecksum[2];

extern u8 *g_pSpi1DataDest;

extern u8         g_u8SpiOneRxBuff[];
extern u8         g_u8SpiOneTxBuff[];
extern volatile u32        g_u32SpiOneTxLen;
extern volatile u32        g_u32SpiOneRxLen;
extern volatile u32        g_u32SpiOneRxCount;
extern volatile u32        g_u32SpiOneTxCount;
extern volatile bool       g_bSemSpiOneBusy;

extern volatile bool     g_bSemSpiOneBusy;
extern      volatile bool g_bFlashPageReadFinished, g_bFlashPageWriteFinished;
extern SPI_SPL_STATE g_eSpiOneState;
extern SPI_ONE_CS g_eSpiOneCs;
extern u32 flash_storage_address, flash_last_sim_address;


u32  FlashReadStatusReg(void);
u8  NorFlashReadRegCmnd(u8 u8Cmnd);
u32  FlashReadFlagStatusReg(void);
void FlashChipEnable(void);
void FlashChipDisable(void); 
void delay_ms(u32 ms);


//#pragma optimize=speed 


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void FlashBlockingXfr(u8 *TxBuff, u8 *RxBuff, u32 TxLen, u32 RxLen)
  {
  u8 data, byte;
  g_bSemSpiOneBusy = true;
  __disable_interrupt();
  FlashChipEnable();
  delay_us(2);
  while (TxLen--)
    {
    byte = *TxBuff++;
    SPI1->DR = byte;
    while ((SPI1->SR & SPI_SR_TXE) == 0);
    data = SPI1->DR;
    }
  while (RxLen)
    {
    SPI1->DR = 0xFD;
    while ((SPI1->SR & SPI_SR_TXE) == 0);
    *RxBuff++ = SPI1->DR;
    RxLen--;
    }
  delay_us(5);
  FlashChipDisable();
  __enable_interrupt();
  g_bSemSpiOneBusy = false;
  }


#if 0
/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void AddByteToChecksum(u8 data, u16 *checksum)
  {
  *checksum += (u16)data;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void AddHalfWordToChecksum(u16 data, u16 *checksum)
  {
  *checksum += data & 0xFF;
  *checksum += (data >> 8) & 0xFF;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void AddWordToChecksum(u32 data, u16 *checksum)
  {
  AddHalfWordToChecksum(data & 0xFFFF, checksum);
  AddHalfWordToChecksum(data >> 16, checksum);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void AddByteArrayToChecksum(u8 *data, u32 len, u16 *checksum)
  {
  while (len--)
    {
    *checksum += (u16)*data++;
    }
  }
#endif

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void FlashWriteStatusRegister(u8 data)
  {
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND ] = FLASH_WRITE_ST_CMND;
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND+1] = data;
  g_u32SpiOneTxLen = 2;
//  StartSpiOneTx();
  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, 0); 
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void  FlashSctErase(u32 u32Add)
  {
  NorFlashWriteRegCmnd(FLASH_ENTER_4BYTE_ADD);
  NorFlashWriteRegCmnd(FLASH_WRITE_ENB_CMND);
  FlashWriteStatusRegister(0);
  g_bSemSpiOneBusy = true;
//  g_eSpiOneState = eSpiWrite;
  
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND ] = FLASH_4ERASE_SCT_CMND;
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD3] = (u8)(u32Add >> 24);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD2] = (u8)(u32Add >> 16);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD1] = (u8)(u32Add >> 8);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD0] = (u8)(u32Add >> 0);
  g_u32SpiOneTxLen = FLASH_SPI_4SERVICE_INFO;
  g_u32SpiOneRxLen = 0;
  nor_struct.eNorFlashState = eNorStartErase;
//  g_eSpiOneCs = eFlashCs;
//  StartSpiOneTx();
  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, g_u32SpiOneRxLen); 
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void  FlashErase128K(u32 u32Add)
  {
  static u32 stat;
  stat = FlashReadStatusReg();
  NorFlashWriteRegCmnd(FLASH_ENTER_4BYTE_ADD);
  stat = FlashReadStatusReg();
  NorFlashWriteRegCmnd(FLASH_WRITE_ENB_CMND);
  stat = FlashReadStatusReg();
  NorFlashWriteRegCmnd(FLASH_WRITE_ENB_CMND);
  stat = FlashReadStatusReg();
//  FlashWriteStatusRegister(0);
  g_bSemSpiOneBusy = true;
//  g_eSpiOneState = eSpiWrite;
  
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND ] = 0xD8;
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD3] = (u8)(u32Add >> 24);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD2] = (u8)(u32Add >> 16);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD1] = (u8)(u32Add >> 8);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD0] = (u8)(u32Add >> 0);
  g_u32SpiOneTxLen = FLASH_SPI_4SERVICE_INFO;
  g_u32SpiOneRxLen = 0;
//  nor_struct.eNorFlashState = eNorStartErase;
//  g_eSpiOneCs = eFlashCs;
//  StartSpiOneTx();
  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, g_u32SpiOneRxLen); 
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  stat = FlashReadStatusReg();
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 error;
void V12_FlashWritePage(u32 u32Add, u32 u32Len, u8* pBuff)
  {
  
  NorFlashWriteRegCmnd(FLASH_ENTER_4BYTE_ADD);
  NorFlashWriteRegCmnd(FLASH_WRITE_ENB_CMND);
  g_bSemSpiOneBusy = true;
  
  g_eSpiOneState = eSpiWrite;
  
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND ] = FLASH_4WRITE_PAGE_CMND;
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD3] = (u8)(u32Add >> 24);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD2] = (u8)(u32Add >> 16);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD1] = (u8)(u32Add >> 8);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD0] = (u8)(u32Add >> 0);
  memcpy(g_u8SpiOneTxBuff + FLASH_SPI_4OFFSET_DATA, pBuff, u32Len);
  g_u32SpiOneTxLen = FLASH_SPI_4SERVICE_INFO + u32Len;
  
  //  StartSpiOneTx();
  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, 0); 
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void V12_FlashReadData(u32 u32Add, u32 u32Len, u8 *DestBuffer)
  {
  g_bSemSpiOneBusy = true;
  g_eSpiOneState = eSpiRead;
  g_bFlashPageReadFinished = 0;
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND ] = FLASH_4READ_DATA_CMND;
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD3] = (u8)(u32Add >> 24);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD2] = (u8)(u32Add >> 16);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD1] = (u8)(u32Add >> 8);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD0] = (u8)(u32Add >> 0);
  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_DATA] = 0;
  g_u32SpiOneTxLen = FLASH_SPI_4SERVICE_INFO+1;
  g_u32SpiOneRxLen = u32Len;
  
  g_pFlashReadDestPtr = DestBuffer;
  nor_struct.eNorFlashState = eNorStartRead;

  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, g_u32SpiOneRxLen); 
  
  memcpy(DestBuffer, g_u8SpiOneRxBuff, u32Len);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//void FlashEraseData(void)
//  {
//  u32 faddress = 0, count, not_ff;
//  u8 buf[256];
//  send_message("\r\nErasing simulation data...\r\n");
//  while (faddress < nor_struct.u32NorFlashSize - 0x10000)
//    {
//    V12_FlashReadData(faddress, 256, buf);
//    for (not_ff = 0, count = 0; count < 256; count++)
//      {
//      if (buf[count] != 0xFF)
//        {
//        not_ff++;
//        }
//      }
//    if (not_ff == 0)
//      {
//      break;
//      }
//    FlashErase128K(faddress);
//    send_message(".");
//    delay_ms(1000);
//    faddress += 0x20000;
//    }
//  FlashErase128K(nor_struct.u32NorFlashSize - 0x20000);
//  send_message("\r\nDone\r\n");
//  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//void V12_PrefetchNextUploadPage(void)
//  {
//  if (nor_struct.u32GetNorFlashAdd & 255)
//    {
//    __NOP();
//    }
//  g_u32FlashPrefetchAddr = nor_struct.u32GetNorFlashAdd + SZ_NOR_FLASH_PAGE;
//  V12_FlashReadData(g_u32FlashPrefetchAddr, SZ_NOR_FLASH_PAGE, (u8 *)g_u8FlashPrefetchBuffer);
//  g_bComputeCRC     = true;
//  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void V12_FetchUploadPage(void)
  {
  if (nor_struct.u32GetNorFlashAdd & 255)
    {
    __NOP();
    }
  g_u32FlashReadAddr = nor_struct.u32GetNorFlashAdd;
  V12_FlashReadData(g_u32FlashReadAddr, SZ_NOR_FLASH_PAGE, (u8 *)g_u8FlashPrefetchBuffer);
  g_bComputeCRC     = true;
  g_pSpi1DataDest = (u8 *)&g_u8FlashPrefetchBuffer[0];
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//u32 gt_u32FlashErrors;
//u32 NOR_CheckFlashCondition(void)
//  {
//  u8 wbuf[256];
//  u32 addr;//, space = 0;
//  gt_u32FlashErrors = 0;
//  nor_struct.u32GetNorFlashAdd = 0;
//  g_bTestFlashIntegrity = true;
//  for (addr = 0; addr < 256; addr++)
//    {
//    wbuf[addr] = 256 - addr;
//    }
//  u32 size = nor_struct.u32NorFlashSize;
//  while (nor_struct.u32SetNorFlashAdd <= size - 256)
//    {
//    for (addr = 0; addr < 256; addr++)
//      {
//      wbuf[addr]++;
//      }
//    V12_FlashWritePage(nor_struct.u32SetNorFlashAdd, SZ_NOR_FLASH_PAGE, wbuf);// ~600uS
//    delay_us(2000);
//    V12_FlashReadData(nor_struct.u32SetNorFlashAdd, SZ_NOR_FLASH_PAGE, g_u8SpiOneRxBuff);
//    delay_us(2000);
//    g_bFlashPageReadFinished = 0;
//    for (addr = 0; addr < 256; addr++)
//      {
//      if (wbuf[addr] != g_u8SpiOneRxBuff[addr])
//        {
//        gt_u32FlashErrors++;
//        }
//      }
//    nor_struct.u32SetNorFlashAdd += 256;
//    
//    }
////exit:
//  nor_struct.eNorFlashState = eNorIdle;
//  g_bTestFlashIntegrity = false;
//  return gt_u32FlashErrors;
//  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void  ClearNorFlashModule(void)
  {
  if (nor_struct_guard != 0x1A2B3C4D)
    {
    nor_struct.u32SetNorFlashAdd = 0;
    nor_struct.u32GetNorFlashAdd = 0;
    nor_struct.eNorFlashState = eNorIdle;
    nor_struct.pNorFlashBuff = (u8 *)nor_struct.u8NorFlashFirstBuff;
    nor_struct.u32NorFlashFrame = 0;
    nor_struct.bSemNorFlashFull = false;
    nor_struct.bSemNorFlashWrite = false;
    nor_struct.bSemNorFlashPause = false;
    nor_struct.bSemNorFlashErase = false;
    memset((u8 *)nor_struct.u8NorFlashFirstBuff ,0, sizeof(nor_struct.u8NorFlashFirstBuff));
    memset((u8 *)nor_struct.u8NorFlashSecondBuff, 0,sizeof(nor_struct.u8NorFlashSecondBuff));
    nor_struct.pNorFlashBuff = 0;
    nor_struct.u32NorFlashFrame = 0;
    nor_struct.u32SemNorReadTime = 0;
    nor_struct.u16CountTimeNextErase = 0;
    g_u32RecordSampleCounter = 0;
    nor_struct_guard = 0x1A2B3C4D;
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
bool V12_checkPageIntegrity(u8 *page)
  {
  u32 count, checksum, compare;
  for (count = 0; count < 254; count++)
    {
    checksum += (u32)*page++;
    }
  compare = (u32)*page++;
  compare |= ((u32)*page) << 8;
  return checksum = compare;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//void  V12_CheckFlashDataRequest(void)
//  {
//  if (g_bSemSpiOneBusy)
//    {
//    return;
//    }
//  
//  if (g_u32FlashReadRequest && !g_u32FlashEraseCtr)
//    {
//    if (g_u32FlashReadRequest == 1)
//      {
//      gt_FlashErrorAddrX = 0;
//      g_bFlashPageReadFinished = false;
//      nor_struct.bSemNorFlashErase = false;
//      nor_struct.u32GetNorFlashAdd = g_u32FlashAddressRequest;
//retry_read:
//      V12_FetchUploadPage();
//      g_u32FlshDataUploadInProgressCtr = 30;
//      g_u32FlashReadRequest = 2;
//      }
//    else if (g_u32FlashReadRequest == 2)
//      {
//      if (g_bFlashPageReadFinished) // wait for prefetch to finish
//        {
//        g_bFlashPageReadFinished = false;
//        if (V12_checkPageIntegrity((u8 *)g_u8FlashPrefetchBuffer))
//          {
//          V12_SendOneNorFlashPage((u8 *)g_u8FlashPrefetchBuffer); 
//          }
//        else
//          {
//          gt_u32CountFlashRetries++;
//          if (g_u32FlashReadRetry < 2)
//            {
//            goto retry_read;
//            }
//          __NOP();
//          }
//        g_u32FlashReadRetry = 0;
//        g_u32FlashReadRequest = false;
//        }
//      }
//    }
//  else if (nor_struct.bSemNorFlashErase)
//    {
//    if (GetTmrFlag(TMR_1mS_FLASH_ERASE))
//      {
//      ClearTmrFlag(TMR_1mS_FLASH_ERASE);
//      nor_struct.bSemNorFlashFull = false;
//      if (g_u32FlashEraseCtr == 0)
//        {
//        nor_struct.bSemNorFlashWrite = false;
//        nor_struct.u32NbrErasedSectors = 0;
//        g_u32FlashEraseCtr = 1;
//        g_u16PpgPageNumber = 0;
//        V12_DebugIndex = 0;
//        g_u32RecordSampleCounter = 0;
//        nor_struct.u16CountTimeNextErase = 2; // 1 second delay
//        }
//      else if (--nor_struct.u16CountTimeNextErase == 0)
//        {
//        u32 SetAddr = nor_struct.u32SetNorFlashAdd;
//        if (nor_struct.u32NbrErasedSectors * 65536 < SetAddr)
//          {
//          FlashSctErase(nor_struct.u32NbrErasedSectors++ * 65536);// ~900uS
//          nor_struct.u16CountTimeNextErase = 105; // 1 second delay
//          g_u32FlashEraseCtr++;
//          }
//        else
//          {
//          nor_struct.bSemNorFlashErase = false;
//          g_bSemGetEraseState = true;
//          g_bSemBleFlashErase = false;
//          nor_struct_guard = 0;
//          g_u32FlashEraseCtr = 0;
//          ClearNorFlashModule();
//          }
//        }
//      }
//    }
//  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
extern SPI_ONE_CS g_eSpiOneCs;
extern SPI_SPL_STATE g_eSpiOneState;

void  NorFlashWriteRegCmnd(u8 u8Cmnd)
  {
  while (g_bSemSpiOneBusy);
  g_bSemSpiOneBusy = true;
  g_eSpiOneState = eSpiWrite;
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND] = u8Cmnd;
  
  g_u32SpiOneTxLen = FLASH_SPI_CMND_SERVICE_INFO;
  
  g_eSpiOneCs = eFlashCs;

  FlashBlockingXfr(g_u8SpiOneTxBuff, NULL, g_u32SpiOneTxLen, 0);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u8  NorFlashReadRegCmnd(u8 u8Cmnd)
  {
  while (g_bSemSpiOneBusy);
  g_eSpiOneState = eSpiRead;
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND] = u8Cmnd;;
  
  g_u32SpiOneTxLen = FLASH_SPI_CMND_SERVICE_INFO;
  g_u32SpiOneRxLen = 1;
  
  g_pFlashReadDestPtr = g_u8SpiOneRxBuff;
  g_eSpiOneCs = eFlashCs;
  
  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, g_u32SpiOneRxLen);
  
  return *g_u8SpiOneRxBuff;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void  FlashReadChipId(u8* pB)
  {
  while (g_bSemSpiOneBusy);
  g_bSemSpiOneBusy = true;
  g_eSpiOneState = eSpiRead;
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND] = FLASH_READ_CHIP_ID;
  
  g_u32SpiOneTxLen = FLASH_SPI_CMND_SERVICE_INFO;
  g_u32SpiOneRxLen = 3;
  
  g_pFlashReadDestPtr = pB;
  
  g_eSpiOneCs = eFlashCs;

  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, g_u32SpiOneRxLen);

  memcpy(pB, g_u8SpiOneRxBuff, 3);

  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32  FlashReadStatusReg(void)
  {
  u32 status;
  g_bSemSpiOneBusy = true;
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND] = FLASH_READ_ST_CMND;
  
  g_u32SpiOneTxLen = FLASH_SPI_CMND_SERVICE_INFO;
  g_u32SpiOneRxLen = 4;
  
  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, g_u32SpiOneRxLen); 
  
  status = g_u8SpiOneRxBuff[0];//(u16)g_u8SpiOneRxBuff[0] | ((u16)g_u8SpiOneRxBuff[1] << 8);
  return status;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32  FlashReadFlagStatusReg(void)
  {
  u32 status;
  g_bSemSpiOneBusy = true;
  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND] = FLASH_READ_FLST_CMND;
  
  g_u32SpiOneTxLen = FLASH_SPI_CMND_SERVICE_INFO;
  g_u32SpiOneRxLen = 2;
  
  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, g_u32SpiOneRxLen); 
  
  status = (u16)g_u8SpiOneRxBuff[0] | ((u16)g_u8SpiOneRxBuff[1] << 8);
  return status;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//void  FlashChipCompleteErase(void)
//  {
//  g_bSemSpiOneBusy = true;
//  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND] = FLASH_ERASE_DIE_CMND;
//  
//  g_u32SpiOneTxLen = FLASH_SPI_CMND_SERVICE_INFO;
//  g_u32SpiOneRxLen = 0;
//  
//  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, g_u32SpiOneRxLen); 
//  }
//

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
bool AllowRecord(void)
  {
  return nor_struct.bSemNorFlashWrite == true && nor_struct.bSemNorFlashPause == false;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void ContinueRecording(void)
  {
  if (nor_struct.bSemNorFlashPause == true)
    {
    nor_struct.bSemNorFlashWrite = true;
    nor_struct.bSemNorFlashPause = false;
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
bool FlashPageIsNotErased(u32 u32Add)
  {
  V12_FlashReadData(u32Add, SZ_NOR_FLASH_PAGE, g_u8SpiOneRxBuff); // ~450uS on /4/2 freq
  nor_struct.eNorFlashState = eNorIdle;
  g_bFlashPageReadFinished = 0;
  for (u32 u32I=0; u32I < SZ_NOR_FLASH_PAGE; u32I++)
    {
    if (g_u8SpiOneRxBuff[u32I] != 0xFF)
      {
      return true;
      }
    }
  return false; 
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void  InitExtFlash(void)
  {
  output_drive(FL_CS);
  output_drive(FL_RST);
  
  output_high(FL_CS);
  output_high(FL_RST);

  FlashChipDisable();
  
  ClearNorFlashModule();
  
  u8 pBuff[3] = { 0,0,0 };
  g_bFlashOperatingOK = true;
  NorFlashWriteRegCmnd(FLASH_RELEASE_FROM_DEEP);
  
  FlashReadChipId(pBuff);
  
  nor_struct.u32NorFlashSize = 0x4000000;
  
  g_bSemNorFlashOn = true;
  g_u32DontInitFlashSize = 0;
  
  //-- get out from deep sleep --//
  NorFlashWriteRegCmnd(FLASH_RELEASE_FROM_DEEP);
  NorFlashWriteRegCmnd(FLASH_WRITE_ENB_CMND);
  g_u16NorFlashRegState = NorFlashReadRegCmnd(FLASH_READ_ST_CMND);
  //-- reset --//
  NorFlashWriteRegCmnd(0x66);
  DelayCycles(5000);
  NorFlashWriteRegCmnd(0x99);
  DelayCycles(5000);
  
  g_u16NorFlashRegState = NorFlashReadRegCmnd(FLASH_READ_NVOL_CNFG);
  g_u16NorFlashRegState = NorFlashReadRegCmnd(FLASH_READ_VOL_CNFG);
  g_u16NorFlashRegState = NorFlashReadRegCmnd(FLASH_READ_ST_CMND);
  if (g_u16NorFlashRegState & NOR_FLASH_ST_BIT_WRENB)
    {
    NorFlashWriteRegCmnd(FLASH_WRITE_DIS_CMND);
    }
  
  //-- smart erase engine --//
  u32 u32PacketSize = nor_struct.u32NorFlashSize / 2;
  u32 u32CurrAdd = nor_struct.u32NorFlashSize / 2;

  V12_FlashReadData(u32CurrAdd, SZ_NOR_FLASH_PAGE, g_u8SpiOneRxBuff); // ~450uS on /4/2 freq

  do
    {
    u32PacketSize /= 2;
    if (FlashPageIsNotErased(u32CurrAdd))
      {
      u32CurrAdd += u32PacketSize;
      }
    else
      {
      u32CurrAdd -= u32PacketSize;
      }
    }
  while (u32PacketSize > SZ_NOR_FLASH_PAGE);
  
  if (FlashPageIsNotErased(u32CurrAdd))
    {
    u32CurrAdd += u32PacketSize;
    }
  if (u32CurrAdd == SZ_NOR_FLASH_PAGE)
    {
    if (!FlashPageIsNotErased(0))
      {
      u32CurrAdd = 0;
      }
    }
  
  nor_struct.u32SetNorFlashAdd = u32CurrAdd;
  nor_struct.u32GetNorFlashAdd = 0;
  
  u32 FlashAdd = nor_struct.u32SetNorFlashAdd;
  u32 FlashSize = nor_struct.u32NorFlashSize;
  if (FlashAdd >= FlashSize - 0x200)
    {
    nor_struct.bSemNorFlashFull = true;
    }
  else
    {
    g_bFlashOperatingOK = false;
    }
  flash_storage_address = flash_last_sim_address = nor_struct.u32SetNorFlashAdd;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void  ClearNorFlashFrame(void)
  {
  nor_struct.u32NorFlashFrame = 0;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//void  FlashSubSctErase(u32 u32Add)
//  {
//  NorFlashWriteRegCmnd(FLASH_ENTER_4BYTE_ADD);
//  NorFlashWriteRegCmnd(FLASH_WRITE_ENB_CMND);
//  g_bSemSpiOneBusy = true;
//  
//  g_u8SpiOneTxBuff[FLASH_SPI_OFFSET_CMND ] = FLASH_4ERASE_SUBSCT4K_CMND;
//  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD3] = (u8)(u32Add >> 24);
//  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD2] = (u8)(u32Add >> 16);
//  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD1] = (u8)(u32Add >> 8);
//  g_u8SpiOneTxBuff[FLASH_SPI_4OFFSET_ADD0] = (u8)(u32Add >> 0);
//  g_u32SpiOneTxLen = FLASH_SPI_4SERVICE_INFO;
//  
//  nor_struct.eNorFlashState = eNorStartErase;
//  FlashBlockingXfr(g_u8SpiOneTxBuff, g_u8SpiOneRxBuff, g_u32SpiOneTxLen, 0); 
//  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 FlashBulkErase(void)
  {
  u32 count = 0;
  NorFlashWriteRegCmnd(FLASH_ENTER_4BYTE_ADD);
  NorFlashWriteRegCmnd(FLASH_WRITE_ENB_CMND);
  NorFlashWriteRegCmnd(FLASH_4ERASE_BULK_CMND);
  while (1)
    {
    FlashReadStatusReg();
    if ((g_u8SpiOneRxBuff[0] & 1) == 0)
      {
      break;
      }
    count++;
    }
  
  return count;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 FlashWriteTest(void)
  {
  u8 wbuf[256], rbuf[256];
  u32 index, value;  
  g_eSpiOneCs = eFlashCs;

  FlashBulkErase();
  delay_ms(1000);
  V12_FlashReadData(EXT_FLASH_LAST_BLOCK, 256, rbuf);
  delay_us(100000);
  V12_FlashWritePage(EXT_FLASH_LAST_BLOCK, 128, wbuf);
  delay_us(200000);
  memset(rbuf, 0, 256);
  while (g_bSemSpiOneBusy);
  V12_FlashReadData(EXT_FLASH_LAST_BLOCK, 128, rbuf);
  delay_us(200000);
  while (g_bSemSpiOneBusy);
  g_bFlashPageReadFinished = 0;
  for (index = 0, value = 0; index < 128; index++)
    {
    if (wbuf[index] != g_u8SpiOneRxBuff[index])
      {
      value++;
      }
    }
  nor_struct.eNorFlashState = eNorIdle;
  
  return value == 0;
  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void FlashChipEnable(void)
  {
  output_low(FL_CS);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void FlashChipDisable(void) 
  {
  output_high(FL_CS);
  }


