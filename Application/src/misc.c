/* Includes ------------------------------------------------------------------*/
#include "stm32F4xx_hal.h"

/* USER CODE BEGIN Includes */

//#include "string.h"
#include "setup.h"
#include "main.h"
#include "protocol.h"

#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <arm_itm.h>

/* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc;
//
//IWDG_HandleTypeDef hiwdg;
//
//RTC_HandleTypeDef hrtc;
//
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
extern RTC_HandleTypeDef hrtc;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

extern u32 g_u32TestFlashErasedCtr, g_u16SampleDumpRequested, g_u16SampleResendRequest;


void SDADC_InitConversions(void);
void test_relays(void);
u8 RTC_ComputeWeekDay(DATE_TIME *dt);
void collect_all_IOs(void);
void MX_ADC1_Init(void);
void ETHR_init(void);
void Setup_LSE(void);
void RTC_load_date_time(void);

u32 relay, store_dt;

u32 my_address;

u32 count_1ms_inter;


u32 TIM2_ticks    = 0;

u32 servo_time;
s32 servo_increment, servo_position;
u32 servo_steps;


u32 test_modbus_flag = 0;





void Configure_IWDG(void);
u32 get_DIP(void);
void SystemClock_Config(void);
void send_test_text(void);
void init_variables(void);
void store_mfg_defaults(void);
void read_global_setup(void);
void init_readers(void);
void RS485_comm_handler(void);
void calibrate_25cm_fuel_4_20ma(void);
void SendTesterCommandToWatch(SAMPLE_DUMP_TYPE data_type, u32 samples);
void watch_comm(void);






u16  event_index, event_counter;
u16  event_report_index, event_report_count;
u16  event_not_sent, event_next_send;

u8 update_dt;


//=========== timer =======================================-
extern u32 TMR_1mS_Flags, TMR_1mS_Cnt;
extern u32 TMR_5mS_Flags;
extern u32 TMR_10mS_Flags, TMR_10mS_Cnt;
extern u32 TMR_100mS_Flags, TMR_100mS_Flags2, TMR_1Sec_Flags;
extern u32 TMR_1mS_Count,  TMR_10mS_Count; // used for timeout purposes by handler functions
extern u32 TMR_1min_count;
extern u32 TMR_1min_Flags;


u8 set_1sec_flags = 0;

//====== date/time =========================================
DATE_TIME sys_date_time;
u8 day_of_week;

//u8 TMR_Update_time;

//u8 const max_day[13] = {  0 ,31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
u16  const CummulativeMonthDays[12] = {  0, 31, 59, 90,120,151,
                                        181,212,243,273,304,334};


//======== prototypes =============================================
//void    update_date_time(void);
//u32   date_time_diff(DATE_TIME *time1, DATE_TIME *time2);
//void delay_mst(u16 delay);
//void get_analog_inputs(void);
//void get_digital_inputs(void);



/**
  * @brief  Sets a single I/O pin to any operational mode.
  * @param  GPIOx:  Port A, B, C etc.
  * @Param  Pin:    Pin on port 0..15
  * @Param  Mode:   One of the modes in GPIOModeFunc_TypeDef
  * @retval None
  */
void SetPinMode(GPIO_TypeDef* GPIOx, u16 Pin, GPIOModeFunc_TypeDef PortMode)
  {
  u32 mask1, mask3;
  u32 Mode, Out, Speed, Pull;

  mask1 = ~((u32)1 << Pin);
  mask3 = ~((u32)3 << (Pin * 2));

  Mode = PortMode & 3;           Mode  <<= Pin * 2;
  Out = (PortMode >> 4) & 1;     Out   <<= Pin;
  Speed = (PortMode >> 8) & 3;   Speed <<= Pin * 2;
  Pull = (PortMode >> 12) & 3;   Pull  <<= Pin * 2;

  GPIOx->MODER   = (GPIOx->MODER   & mask3) | Mode;
  GPIOx->OTYPER  = (GPIOx->OTYPER  & mask1) | Out;
  GPIOx->OSPEEDR = (GPIOx->OSPEEDR & mask3) | Speed;
  GPIOx->PUPDR   = (GPIOx->PUPDR   & mask3) | Pull;
  }

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
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
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
void SetupServoPWM(u32 microSec, s32 increments, u32 steps)
  {
  if (microSec == 0 && increments == 0)
    {
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    }
  else
    {
    if (microSec == 0 && increments != 0)
      {
      microSec = servo_time;
      }
    else if (microSec)
      {
      servo_position = microSec;
      }
    servo_time      = microSec;
    servo_increment = increments;
    servo_steps     = steps;
    
    __HAL_TIM_ENABLE(&htim1);
    TIM1->CCR3 = TIM1->ARR - microSec;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void ModifyServoPWM(s32 change)
  {
  servo_time = change;
  TIM1->CCR3 = TIM1->ARR - servo_time;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void HandleServoPWMchange(void)
  {
  if (servo_steps)
    {
    if (GetTmrFlag(TMR_10MS_SERVO_STEPS))
      {
      ClearTmrFlag(TMR_10MS_SERVO_STEPS);
      servo_steps--;
      servo_position += servo_increment;
      ModifyServoPWM(servo_position);
      if (servo_position < 2500 || servo_position > 3000)
        {
        servo_steps = 0;
        }
      }
    }
  }
    
/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void ServoHomePos(void)
  {
  SetupServoPWM(2500, 0, 0);
  servo_position = 2500;
  }
  
/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void RequestArtSamples(u32 samples)
  {
  g_u16SampleDumpRequested = samples;
  g_u16SampleResendRequest = g_u16SampleDumpRequested;
  SendTesterCommandToWatch(DUMP_TYPE_ART, samples+2);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void WaitForArtSamples(u32 mS)
  {
  while (mS)
    {
    watch_comm();
//    event_loop();
    HandleServoPWMchange();
    if (GetTmrFlag(TMR_1mS_ART_TEST))
      {
      ClearTmrFlag(TMR_1mS_ART_TEST);
      mS--;
      }
    }
  }
  
  
/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void WaitForPWMfinish(void)
  {
  while (servo_steps)
    {
    HandleServoPWMchange();
    }
  }
  
/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void test_artifact(u32 loops, u32 inc, u32 steps)
  {
  ServoHomePos();
  WaitForPWMfinish();
  if (loops == 0)
    {
    loops = 5;
    inc   = 3;
    steps = 3;
    }
  RequestArtSamples(11);
  delay_ms(1000);
  SetupServoPWM(0, 10, 10);
  WaitForPWMfinish();
//  delay_ms(100);
  for (u32 loop = 0; loop < loops; loop++)
    {
    SetupServoPWM(0, inc, steps);
    RequestArtSamples(11);
    WaitForArtSamples(1000);
    if (servo_position > 3000)
      {
      break;
      }
    }
  ServoHomePos();
  }

  
/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void SYSTICK_handler(void)
  {
  TIM2_ticks++;

  TMR_1mS_Flags = 0xFFFFFFFF;
  TMR_1mS_Count++;
//  comm_timer++;
  count_1ms_inter++;
  
//  ServoControl();
  
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
        //      periodic_data_dump();
        if (++TMR_1min_count >= 60)
          {
          TMR_1min_count = 0;
          TMR_1min_Flags = 0xFFFFFFFF;
          }
        }
      if (++update_dt >= 100) // update date time from RTC every 10 seconds
        {
        update_dt = 0;
        //      month = sys_date_time.month;
        RTC_load_date_time();
        }
      }
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 CreateCradleID(void)
  {
  u8 *McuID;
  u32 ID = 0, loop;
  McuID = (u8 *)0x1FFF7A10;
  for (loop = 0; loop < 12; loop++)
    {
    if (*McuID != 0)
      {
      ID = ID * 2 + *McuID;
      }
    }
  return ID;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void event8(u8 evt)
  {
  ITM_EVENT8_WITH_PC(1,evt);
  }

void event_Stack_depth(void)
  {
  u32 psp = __get_PSP();
  ITM_EVENT32_WITH_PC(2, psp);
  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void relay_connect(u32 conn)
  {
  output_drive(EXT_RELAYS);
  if (conn)
    {
    output_low(EXT_RELAYS); // shut down DC-DC on external board
    }
  else
    {
    output_high(EXT_RELAYS); // enable DC-DC on external board
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 stepper_steps, stepper_speeds[8], stepper_time, stepper_seq_number;
u32 stepper_current_time, stepper_current_period;

void HandleStepperMotor(void)
  {
  u32 step_phase;
  output_low(STEPPER_OUT1);
  output_low(STEPPER_OUT2);
  output_low(STEPPER_OUT3);
  output_low(STEPPER_OUT4);
  if (stepper_steps)
    {
    step_phase = stepper_steps & 3;
    stepper_steps--;
    switch (step_phase)
      {
      case 0: output_high(STEPPER_OUT1); break;
      case 1: output_high(STEPPER_OUT2); break;
      case 2: output_high(STEPPER_OUT3); break;
      case 3: output_high(STEPPER_OUT4); break;
      }
    }
  else
    {
    __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
    }
  }
  
/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void start_stepper(u32 time, u32 *speed, u32 index)
  {
  stepper_time = time * 1000; // time in mS
  memcpy((u8 *)stepper_speeds, (u8 *)speed, index * 4);
  stepper_seq_number = 0;
  stepper_current_time = stepper_time;
  stepper_current_period = stepper_speeds[stepper_seq_number];
  TIM4->ARR = stepper_current_period;
  TIM4->CNT = 0;
  __HAL_TIM_ENABLE(&htim4);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void stop_stepper(void)
  {
  }

/* USER CODE END 4 */

/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
//void Error_Handler(void)
//  {
//  /* USER CODE BEGIN Error_Handler */
//  /* User can add his own implementation to report the HAL error return state */
//  while(1)
//    {
//    }
//  /* USER CODE END Error_Handler */
//  }
//
#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
  {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

  }

#endif

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

