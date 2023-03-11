#include "main.h"
#include "setup.h"
#include "misc.h"
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

  EnableEEPROM();
  EnableTVOC();
  EnablePRES();
  }  



void main_loop(void)
  {
//  disable_systick();
//  setEN_PHOTO(1);
  
  setTVOC_PWR(1);
  delay_ms(100);
  init_tvoc();
  delay_ms(100);
  
  init_zmod4410();
  delay_ms(100);
  calibrate_zmod4410();
  delay_ms(100);
  
  while (1)
    {
//    __WFI();
    get_all_gasses();
    acquire_gas_sample();
    delay_ms(3000);
    }
  }