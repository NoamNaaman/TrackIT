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
        if (++TMR_1min_count >= 60)
          {
          TMR_1min_count = 0;
          TMR_1min_Flags = 0xFFFFFFFF;
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

