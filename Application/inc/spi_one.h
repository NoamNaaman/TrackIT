/**
******************************************************************************
* File Name          : spi_one.h
* Date               : 01/9/2019 
* Description        : SPI 1 driver
* Author	     : Lev Zoosmanovskiy
******************************************************************************

******************************************************************************
*/

#ifndef __SPI_1_H
#define __SPI_1_H
//#include "ll_includes.h"
#include "spi.h"

#define _SPI1_DMA_ 1
#define SPI_1_TX_BUFF_SIZE             (0x120)
#define SPI_1_RX_BUFF_SIZE             (0x120)

#define FLASH_CS_ENB() output_low(FLASH_CS1)
#define FLASH_CS_DSB() output_high(FLASH_CS1)


typedef enum
{
  eSpiIdle,
  eSpiReadHeader,
  eSpiWrite,
  eSpiRead,
  eSpiStopRead,
  eSpiBuffAuth
}SPI_SPL_STATE;

typedef enum
{
  eDisplayCs,
  eFlashCs
}SPI_ONE_CS;

void Configure_SPI1_DMA(void);
void SPI1_StartFlashTransfer(void);
void spi_one_write(uint8_t * data, uint16_t len);
void spi_one_read(uint8_t * data, uint16_t len);
void StartSpiOneTx(void);
#endif //__SPI_1_H