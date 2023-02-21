#include "stm32f1xx_hal.h"


#include "main.h"
#include "stm32f1xx_hal.h"
#include "setup.h"
#include "prototypes.h"

// Using Atmel serial EEPROM's (AT24C512B or AT24C1024B)

#define EE_PAGE 256

#define SCL GPIOD,2
#define SDA GPIOD,1

u32 eeprom_write_timer;

typedef enum {
  I2C_MEM  = 1,
  I2C_TVOC = 2,
  I2C_PRES = 3,
  } I2C_CHANNEL;

I2C_CHANNEL I2C_channel;





//--------------------------------------------------------------------------
void SetSDA(u32 high_low)
  {
  switch (I2C_channel)
    {
    case I2C_MEM : output_pin(MEM_SDA,  high_low);
    case I2C_TVOC: output_pin(TVOC_SDA, high_low);
    case I2C_PRES: output_pin(PRES_SDA, high_low);
    }
  }

//--------------------------------------------------------------------------
void SetSCL(u32 high_low)
  {
  switch (I2C_channel)
    {
    case I2C_MEM : output_pin(MEM_SCL,  high_low);
    case I2C_TVOC: output_pin(TVOC_SCL, high_low);
    case I2C_PRES: output_pin(PRES_SCL, high_low);
    }
  }

//--------------------------------------------------------------------------
void delay_cycles(u32 cyc)
  {
  while (cyc--)
    {
//    __NOP();
    __NOP();
    __NOP();
    }
  }

//--------------------------------------------------------------------------
void IIC_Delay(void)
  {
  delay_cycles(8); // 9
  }

//--------------------------------------------------------------------------
void  init_IIC(void)
  {
  }

////--------------------------------------------------------------------------
//void  IIC_Write_Cycle(void)
//  {
//  output_low(EEPROM_WP);
//  delay_ms(10);
//  output_high(EEPROM_WP);
//  }
//
//--------------------------------------------------------------------------
// Sends 8 bits to I2C device
// Gets the byte to send from W register
//--------------------------------------------------------------------------
bool IIC_Write(u8  send_bits)
  {
  u16 count;
  u8   Ack;
//  output_drive(SDA);
  for (count = 0; count < 8; count++)
    {
    if (send_bits & 0x80)
      {
      SetSDA(1);
      }
    else
      {
      SetSDA(0);
      }
    send_bits <<= 1;

    delay_cycles(3); // 8
    SetSCL(1);
    delay_cycles(3); // 17
    SetSCL(0);
    }
  SetSDA(1);
  output_float(SDA);
  delay_cycles(4); // 8
  SetSCL(1);
  Ack = input(SDA);
  delay_cycles(4); // 7
  SetSCL(0);
  SetSDA(1);
  output_drive(SDA);
  return !Ack;
  }

//--------------------------------------------------------------------------
// Receives 8 bits from IýC device
// returns the received byte in the W reg.
//--------------------------------------------------------------------------
u8 IIC_ReadByte(void)
  {
  u16 rec_bits = 0;
  u16 count;
  delay_cycles(2);
  output_float(SDA);
  delay_cycles(4);
  for (count = 0; count < 8; count++)
    {
    SetSCL(1);
    delay_cycles(3); // 10
    rec_bits = (rec_bits << 1) | input(SDA);
    SetSCL(0);
    delay_cycles(3); // 12
    }
  output_drive(SDA);
  return rec_bits;
  }

//--------------------------------------------------------------------------
// Sends start unsigned to IýC device
//--------------------------------------------------------------------------
void IIC_Start(void)
  {
  SetSDA(1);
  SetSCL(1);
  IIC_Delay();
  SetSDA(0);
  IIC_Delay();
  SetSCL(0);
  IIC_Delay();
  }

//--------------------------------------------------------------------------
// Sends stop unsigned to IýC device
//--------------------------------------------------------------------------
void IIC_Stop(void)
  {
  output_drive(SDA);
  output_drive(SCL);
  SetSCL(0);
  SetSDA(0);
  IIC_Delay();
  SetSCL(1);
  IIC_Delay();
  IIC_Delay();
  SetSDA(1);
  }


//--------------------------------------------------------------------------
// Sends ack to IC device
//--------------------------------------------------------------------------
void IIC_SendAck(void)
  {
  output_drive(SDA);
  SetSDA(0);
  IIC_Delay();
  SetSCL(1);
  IIC_Delay();
  SetSCL(0);
  output_float(SDA);
  }

//--------------------------------------------------------------------------
// Sends one unsigned to IýC device
//--------------------------------------------------------------------------
void IIC_SendOne(void)
  {
  SetSDA(1);
  IIC_Delay();
  SetSCL(1);
  IIC_Delay();
  SetSCL(0);
  }

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------



////==============================================================
//void enable_ext_eeprom_write(void)
//  {
//  output_low(EEPROM_WP);
//  delay_us(50);
//  }
//
////==============================================================
//void disable_ext_eeprom_write(void)
//  {
//  output_high(EEPROM_WP);
//  }
//
//==============================================================
void wait_eeprom_write(u8 select)
  {
  while (eeprom_write_timer) // wait for previous eeprom write cycle to end
    {
    }
  }

//==============================================================

u8 set_eeprom_addr(u32 address)
  {
  u32 select;
  u8 ret;
  select = address >> 17;
  switch (select & 3)
    {
    case 0: ret = 0xA8 ; break;
    case 1: ret = 0xA4 ; break;
    case 2: ret = 0xAC ; break;
    case 3: ret = 0xA0 ; break;
    }
  if (address & 0x10000)
    {
    ret |= 2;
    }
  return ret;
  }

//==============================================================
u8 I2C_ack;
u8 PrepareEEsession(u32 address, bool write)
  {
  u8 select;
  init_IIC();
  delay_cycles(30);
  wait_eeprom_write(0xAA);
  select = set_eeprom_addr(address);
  IIC_Stop();
//  enable_ext_eeprom_write();
  IIC_Stop();
  IIC_Start();
  I2C_ack = IIC_Write(select);
  I2C_ack = IIC_Write(make8(address,1));
  I2C_ack = IIC_Write(make8(address,0));
  return select;
  }

//==============================================================
void write_ext_eeprom(u32 address, u8 *data, u16 len)
  {
  u8 select, dat;
  select = PrepareEEsession(address, 1);
  do
    {
    while (len)
      {
      dat = *data++;
      I2C_ack = IIC_Write( dat);
      address++;
      len--;
      if ((address & (EE_PAGE-1)) == 0)
        {
        break;
        }
      }
    IIC_Stop();
    if (len)
      {
      delay_ms(7);
      IIC_Start();
      I2C_ack = IIC_Write(select);
      I2C_ack = IIC_Write(make8(address,1));
      I2C_ack = IIC_Write(make8(address,0));
      }
    }
  while (len);
//  disable_ext_eeprom_write();
  eeprom_write_timer = 8;
  delay_ms(7);
  }

//==============================================================

void read_ext_eeprom(u8 caller, u32 address, u8 *data, u16 len)
  {
  u8 dat;
  u8 select;
  select = PrepareEEsession(address, 0);
  IIC_Start();
  IIC_Write(select+1);
  do
    {
    dat = IIC_ReadByte();
    if (len > 1)
      {
      IIC_SendAck();
      }
    *data++ = dat;
    }
  while (--len);
  IIC_Stop();
  }

//==============================================================

//bool check_eeprom_address(void)
//  {
//  u8 buf[3];
//  buf[0] = 0xFF;
//  buf[1] = 0xFF;
//  read_ext_eeprom(EE_CEA, ADDR_TEST, buf, 2);
//  if (buf[0] == 0xAA && buf[1] == 0x55)
//    {
//    return 1;
//    }
//  return 0;
//  }

//==============================================================
void init_eeprom(void)
  {
//  u32 raddr, x, addr, loop;
//  u8 x;//, loop, buf[4];
  eeprom_write_timer = 0;

  init_IIC();

//  for (addr = 0, loop = 0; loop < 131072; loop++, addr += 4)
//    {
//    write_ext_eeprom(addr, (u8 *)&addr, 4);
//    }
//  for (addr = 0, loop = 0; loop < 131072; loop++, addr += 4)
//    {  
//    read_ext_eeprom(EE_IEE, addr, (u8 *)&raddr, 4);
//    if (addr != raddr)
//      {
//      x++;
//      }
//    }
  }


