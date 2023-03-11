#include "main.h"
#include "setup.h"
#include "cmsis_os.h"
#include "misc.h"



/**
  * @brief  Sets a single I/O pin to any operational mode.
  * @param  GPIOx:  Port A, B, C etc.
  * @Param  Pin:    Pin on port 0..15
  * @Param  Mode:   One of the modes in GPIOModeFunc_TypeDef
  * @retval None
  */
void SetPinMode(GPIO_TypeDef* GPIOx, uint16_t Pin, GPIOModeFunc_TypeDef Mode)
  {
  uint32_t mask, bits;
  if (Mode == GPIO_FMode_IPU)           // is this digital input with pull-up?
    {
    GPIOx->ODR |= 1 << Pin;             // YES. set corresponding bit in ODR
    Mode = GPIO_FMode_IPD;
    }
  else if (Mode == GPIO_FMode_IPD)      // is this digital input with pull-down?
    {
    GPIOx->ODR &= ~(1 << Pin);          // YES. clear corresponding bit in ODR
    }
  bits = (uint32_t)Mode << ((Pin & 7) * 4);
  mask = 15L << ((Pin & 7) * 4);
  if (Pin <= 7)
    {
    GPIOx->CRL = (GPIOx->CRL & ~mask) | bits;
    }
  else
    {
    GPIOx->CRH = (GPIOx->CRH & ~mask) | bits;
    }
  }

