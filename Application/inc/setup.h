
#ifndef __SETUP
#define __SETUP



#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define s8  int8_t
#define s16 int16_t
#define s32 int32_t
#define u64 uint64_t
#define s64 int64_t


#define bool u8
#define true 1
#define false 0


#define EE1024

#define EEPROM_TYPE_I2C 1
#define EEPROM_TYPE_SPI 2
#define __EEPROM_TYPE__ 2



#define SW_VERSION 1

//========== STM32 housekeeping functions ===================================
// I/O pin functionality definitions for use with SetPinMode()

// port modes
#define PORTMODE_INPUT         0
#define PORTMODE_OUTPUT        1
#define PORTMODE_ALTERNATE     2
#define PORTMODE_ANALOG        3

// output modes
#define OUTMODE_PP    0x00
#define OUTMODE_OD    0x10

// output speed
#define OUTSPEED_400KHz        0x000
#define OUTSPEED_2MHz          0x100
#define OUTSPEED_10MHz         0x200
#define OUTSPEED_40MHz         0x300

// pullup/pulldown
#define PULL_NONE              0x0000
#define PULL_UP                0x1000
#define PULL_DOWN              0x2000

// I/O pin functionality definitions for use with SetPinMode()
typedef enum
{ GPIO_FMode_AIN         = 0,          // analog input or DAC output
  GPIO_FMode_IN_FLOATING = 0x04,       // digital input, no pull-up/down resistor
  GPIO_FMode_IPD         = 0x08,       // digital input, pull-up resistor
  GPIO_FMode_IPU         = 0x18,       // digital input, pull-down resistor

  GPIO_FMode_10_Out_PP   = 0x01,       // 10MHz digital output, push-pull output
  GPIO_FMode_10_Out_OD   = 0x05,       // 10MHz digital output, open drain output
  GPIO_FMode_10_AF_PP    = 0x09,       // 10MHz digital output, push-pull alternate function output
  GPIO_FMode_10_AF_OD    = 0x0D,       // 10MHz digital output, open drain alternate function output

  GPIO_FMode_2_Out_PP    = 0x02,       // 2MHz digital output, push-pull output
  GPIO_FMode_2_Out_OD    = 0x06,       // 2MHz digital output, open drain output
  GPIO_FMode_2_AF_PP     = 0x0A,       // 2MHz digital output, push-pull alternate function output
  GPIO_FMode_2_AF_OD     = 0x0E,       // 2MHz digital output, open drain alternate function output

  GPIO_FMode_50_Out_PP   = 0x03,       // 50MHz digital output, push-pull output
  GPIO_FMode_50_Out_OD   = 0x07,       // 50MHz digital output, open drain output
  GPIO_FMode_50_AF_PP    = 0x0B,       // 50MHz digital output, push-pull alternate function output
  GPIO_FMode_50_AF_OD    = 0x0F        // 50MHz digital output, open drain alternate function output
} GPIOModeFunc_TypeDef;


#define SetOutputHigh(PORTx, Pin) PORTx->BSRR = (uint32_t)1L << Pin
#define SetOutputLow(PORTx, Pin) PORTx->BSRR = (uint32_t)1L << (Pin+16)

#define SetOutputPin(PORTx, Pin, value) \
  if (value)                            \
    SetOutputHigh(PORTx, Pin);          \
  else                                  \
    SetOutputLow(PORTx, Pin)

#define InputPin(PORTx, Pin) ((PORTx->IDR & (1 << Pin)) != 0)

#define OutputToggle(PORTx, Pin) \
    if (InputPin(PORTx, Pin))     \
    { \
    SetOutputLow(PORTx, Pin); \
    } \
    else                         \
    { \
    SetOutputHigh(PORTx, Pin); \
    }

#define EnableEXTI(Pin) EXTI->IMR |= 1 << Pin
#define DisableEXTI(Pin) EXTI->IMR &= ~(1 << Pin)

#define output_drive(x) SetPinMode(x, GPIO_FMode_10_Out_PP)
#define output_float(x) SetPinMode(x, GPIO_FMode_IN_FLOATING);
#define output_od(x) SetPinMode(x, GPIO_FMode_10_Out_OD)
#define input_pullup(x) SetPinMode(x, GPIO_FMode_IPU);
#define input_pulldown(x) SetPinMode(x, GPIO_FMode_IPD);

#define output_low(x)        SetOutputLow(x)
#define output_high(x)       SetOutputHigh(x)
#define output_toggle(x)     OutputToggle(x)
#define output_pin(x, value) SetOutputPin(x, value)
    
#define input(x)        InputPin(x)

#define enable_ext_interrupt(x) EnableEXTI(x)
#define disable_ext_interrupt(x) DisableEXTI(x)
#define clear_tim_interrupt(htim) __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE)

#define make32(x3,x2,x1,x0) (((u32)x3 << 24) | ((u32)x2 << 16) | ((u32)x1 << 8) | (u32)x0)
#define make16(x1,x0) (((u32)x1 << 8) | (u32)x0)
#define make8(data, byte) ((data >> (byte * 8)) & 0xFF)

//#define test_timer_1ms(flag)    ((Timer_1mS_Flags & (flag)) != 0)
//#define test_timer_10ms(flag)   ((Timer_10mS_Flags & (flag)) != 0)
//#define test_timer_100ms(flag)  ((Timer_100mS_Flags & (flag)) != 0)
//#define test_timer_1sec(flag)   ((Timer_1sec_Flags & (flag)) != 0)
//
//#define clear_timer_1ms(flag)   Timer_1mS_Flags   &= ~(flag)
//#define Timer_10mS_Flags  &= ~(flag)  Timer_10mS_Flags  &= ~(flag)
//#define Timer_100mS_Flags  &= ~(flag) Timer_100mS_Flags &= ~(flag)
//#define Timer_1sec_Flags  &= ~(flag)  Timer_1sec_Flags  &= ~(flag)

//void SetPinMode(GPIO_TypeDef* GPIOx, u16 Pin, GPIOModeFunc_TypeDef PortMode);

void NVIC_Configuration(void);
void SysTickConfig(void);



//====== date/time =======================================-
typedef struct  {
                u8 year;
                u8 month;
                u8 day;
                u8 hour;
                u8 minute;
                u8 second;
                } DATE_TIME;


extern u8 Month_special_days[31]; // buffer for a full month of up to 31 days

void SetPinMode(GPIO_TypeDef* GPIOx, uint16_t Pin, GPIOModeFunc_TypeDef Mode);




#define LED1          GPIOA,11
#define LED2          GPIOC,3
#define LED3          GPIOC,5

#define FL_PWR        GPIOB,12
#define SENSE_PWR     GPIOB,11
#define TVOC_PWR      GPIOC,1

#define EN_GPS        GPIOC,6
#define EN_5V         GPIOC,2
#define EN_CELLULAR   GPIOC,7
#define EN_PRESSURE   GPIOB,8
#define EN_PHOTO      GPIOB,10

#define FL_WP         GPIOC,8
#define FL_CS         GPIOA,8

#define CELL_RST      GPIOC,12
#define FL_RST        GPIOB,14
#define TVOC_RES      GPIOA,15

#define MEM_SCL       GPIOB,6
#define MEM_SDA       GPIOB,7
#define TVOC_SCL      GPIOD,0
#define TVOC_SDA      GPIOD,1
#define PRES_SCL      GPIOB,0
#define PRES_SDA      GPIOB,1




#define UV_INPUT      GPIOC,4
#define TEMPERATURE   GPIOA,4
#define MAG_RDY       GPIOC,11
#define PIR           GPIOA,0
#define PHOTO_INPUT   GPIOA,1

#define TVOC_INT      GPIOA,12
#define MAG_INT       GPIOC,9
#define ACC_INT       GPIOC,10

#define UART1_TX       GPIOA,9
#define UART1_RX       GPIOA,10
#define UART2_TX       GPIOA,2
#define UART2_RX       GPIOA,3



#endif