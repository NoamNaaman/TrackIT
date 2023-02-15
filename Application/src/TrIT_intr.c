#include "main.h"
#include "stm32f1xx_hal.h"
#include "setup.h"

u16 t100us = 0, prev_rdr2_bits;


  

//=============================================================================================
void enable_interrupt_pin(u32 pin)
  {
  EXTI->IMR |= pin;
  }

//=============================================================================================
void disable_interrupt_pin(u32 pin)
  {
  EXTI->IMR &= ~pin;
  }

