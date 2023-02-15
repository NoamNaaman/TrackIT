///////////////////////////////////////////////////////////////////////////////
// Header file spi.h
//
//
// Version : 11.01.02.XX
// Added: lptm, adc, loader, adpd1, adpd2, 6d, ble, uart
///////////////////////////////////////////////////////////////////////////////

#ifndef __SPI_H__
#define __SPI_H__

#include "main.h"

#define _SPI1_DMA_     1
#define _SPI2_DMA_     1
#define _SPI3_DMA_     0

#define SPI_DUMMY_BYTE            (0xEA)

#define NBR_SGN_BUFF                (4)// @#$ (8) 

#define SPI_SERVICE_INFO          (1)

#define SPI_OFFSET_ADDR           (0)
#define SPI_OFFSET_DATA           (1)

#define SPI_SET_MB                (0x40)
#define SPI_SET_RD                (0x80)
#define SPI_MASTER_RD             (0x80)
#define SPI_MAX_RD                (0x01)
#define SPI_ADPD_WR               (0x01)

#define SPI_BUFF_SIZE             (0x120)
#define SPI_THR_BUFF_SIZE         (0x40)

#define SPI_THR_TX_OUT_LEN        (20)
#define SPI_THR_RX_MAX_LEN        (20)
#define SPI_THR_SZ_HEADER         (2)

#define SPT_SENSOR_ADD            (0x50)

#define OS_SPT_CMD                (0x00)
#define OS_SPT_LEN                (0x01)

#define _SPI_ONE_INT_             (1)

void delay_us(uint32_t);
#define DelayCycles delay_us

#endif
