#include "main.h"
#include "stm32f1xx_hal.h"
#include "setup.h"

//=========== timer =======================================-
u32 TMR_1mS_Flags, TMR_1mS_Cnt;
u32 TMR_5mS_Flags;
u32 TMR_10mS_Flags, TMR_10mS_Cnt;
u32 TMR_100mS_Flags, TMR_100mS_Flags2, TMR_1Sec_Flags;
u32 TMR_1mS_Count,  TMR_10mS_Count; // used for timeout purposes by handler functions
u32 TMR_1min_count;
u32 TMR_1min_Flags;

u32 RunTime_milli;
u32 RunTime_seconds;
u32 RunTime_minutes;


u8 set_1sec_flags = 0;

//====== date/time =========================================
DATE_TIME sys_date_time;
u8 day_of_week;

//u8 TMR_Update_time;

//u8 const max_day[13] = {  0 ,31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
u32  const CummulativeMonthDays[12] = {  0, 31, 59, 90,120,151,
                                        181,212,243,273,304,334};



/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void delay_us(u32 microsec)
  {
//  TIM3->CNT = 0;
//  while (TIM3->CNT < microsec);
  while (microsec--)
    {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); 
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void delay_ms(u32 ms)
  {
  while (ms--)
    {
    delay_us(1000);
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void SYSTICK_handler(void)
  {
  TMR_1mS_Flags = 0xFFFFFFFF;
  TMR_1mS_Count++;
  RunTime_milli++;
  if (!TMR_1mS_Cnt || TMR_1mS_Cnt == 5)
    {
    TMR_5mS_Flags = 0xFFFFFFFF;
    }
  if (++TMR_1mS_Cnt >= 10)
    {
    TMR_1mS_Cnt = 0;
    TMR_10mS_Count++;
    TMR_10mS_Flags = 0xFFFFFFFF;
    if (++TMR_10mS_Cnt >= 10)
      {
      TMR_10mS_Cnt = 0;
      TMR_100mS_Flags = 0xFFFFFFFF;
      if (++set_1sec_flags >= 10)
        {
        set_1sec_flags = 0;
        TMR_1Sec_Flags = 0xFFFFFFFF;
        RunTime_seconds++;
        if (++TMR_1min_count >= 60)
          {
          TMR_1min_count = 0;
          TMR_1min_Flags = 0xFFFFFFFF;
          RunTime_minutes++;
          }
        }
//      if (++update_dt >= 100) // update date time from RTC every 10 seconds
//        {
//        update_dt = 0;
//        //      month = sys_date_time.month;
//        RTC_load_date_time();
//        }
      }
    }
  }




void setLED1(u32 onoff)
  {
  output_pin(LED1, onoff != 0);
  }

void setLED2(u32 onoff)
  {
  output_pin(LED2, onoff != 0);
  }

void setLED3(u32 onoff)
  {
  output_pin(LED3, onoff != 0);
  }


void setFL_PWR(u32 onoff)
  {
  if (onoff)
    {
    output_low(FL_PWR);
    }
  else
    {
    output_high(FL_PWR);
    }
  }

void setSENSE_PWR(u32 onoff)
  {
  if (onoff)
    {
    output_low(SENSE_PWR);
    }
  else
    {
    output_high(SENSE_PWR);
    }
  }

void setTVOC_PWR(u32 onoff)
  {
  if (onoff)
    {
    output_low(TVOC_PWR);
    }
  else
    {
    output_high(TVOC_PWR);
    }
  }

void setEN_GPS(u32 onoff)
  {
  if (onoff)
    {
    output_low(EN_GPS);
    }
  else
    {
    output_high(EN_GPS);
    }
  }

void setEN_5V(u32 onoff)
  {
  if (!onoff)
    {
    output_low(EN_5V);
    }
  else
    {
    output_high(EN_5V);
    }
  }

void setEN_CELLULAR(u32 onoff)
  {
  if (onoff)
    {
    output_low(EN_CELLULAR);
    }
  else
    {
    
    output_high(EN_CELLULAR);
    }
  }

void setEN_PRESSURE(u32 onoff)
  {
  if (onoff)
    {
    output_low(EN_PRESSURE);
    }
  else
    {
    output_high(EN_PRESSURE);
    }
  }

void setEN_PHOTO(u32 onoff)
  {
  output_drive(EN_PHOTO);
  if (!onoff)
    {
    output_low(EN_PHOTO);
    }
  else
    {
    output_high(EN_PHOTO);
    }
  }


void setFL_WP(u32 onoff)
  {
  if (onoff)
    {
    output_low(FL_WP);
    }
  else
    {
    output_high(FL_WP);
    }
  }

void setFL_CS(u32 onoff)
  {
  if (onoff)
    {
    output_low(FL_CS);
    }
  else
    {
    output_high(FL_CS);
    }
  }


void ResetCellular(void)
  {
  output_low(CELL_RST);
  delay_us(100);
  output_high(CELL_RST);
  }

void ResetFlash(void)
  {
  output_low(FL_RST);
  delay_us(100);
  output_high(FL_RST);
  }

void ResetTVOC(void)
  {
  output_low(TVOC_RES);
  delay_us(100);
  output_high(TVOC_RES);
  }


void EnableEEPROM(void)
  {
  output_drive(MEM_SCL);
  output_drive(MEM_SDA);
  output_high(MEM_SCL);
  output_high(MEM_SDA);
  }

void EnableTVOC(void)
  {
  output_drive(TVOC_SCL);
  output_drive(TVOC_SDA);
  output_high(TVOC_SCL);
  output_high(TVOC_SDA);
  setTVOC_PWR(1);
  }

void EnablePRES(void)
  {
  output_drive(PRES_SCL);
  output_drive(PRES_SDA);
  output_high(PRES_SCL);
  output_high(PRES_SDA);
  setEN_PRESSURE(1);
  }

void DisableTVOC(void)
  {
  output_low(TVOC_SCL);
  output_low(TVOC_SDA);
  output_float(TVOC_SCL);
  output_float(TVOC_SDA);
  setTVOC_PWR(0);
  }

void DisablePRES(void)
  {
  output_low(PRES_SCL);
  output_low(PRES_SDA);
  output_float(PRES_SCL);
  output_float(PRES_SDA);
  setEN_PRESSURE(0);
  }
