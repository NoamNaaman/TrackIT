#include "main.h"
#include "setup.h"
#include "prototypes.h"


void init_controls(void)
  {
  setLED1(0);
  setLED2(0);
  setLED3(0);
  setFL_PWR(0);
  setSENSE_PWR(0);
  setTVOC_PWR(0);
  setEN_GPS(0);
  setEN_5V(0);
  setEN_CELLULAR(0);
  setEN_PRESSURE(0);
  setEN_PHOTO(0);
  setFL_WP(0);
  setFL_CS(0);
  DisableTVOC();
  DisablePRES();
  
  disable_GPS_uart();
  disable_Cellular_uart();
  }  



void main_loop(void)
  {
  disable_systick();
  while (1)
    {
    __WFI();
    }
  }