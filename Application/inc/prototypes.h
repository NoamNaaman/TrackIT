#ifndef __PROTOTYPES__
#define __PROTOTYPES__

void setLED1(u32 onoff);
void setLED2(u32 onoff);
void setLED3(u32 onoff);
void setFL_PWR(u32 onoff);
void setSENSE_PWR(u32 onoff);
void setTVOC_PWR(u32 onoff);
void setEN_GPS(u32 onoff);
void setEN_5V(u32 onoff);
void setEN_CELLULAR(u32 onoff);
void setEN_PRESSURE(u32 onoff);
void setEN_PHOTO(u32 onoff);
void setFL_WP(u32 onoff);
void setFL_CS(u32 onoff);
void ResetCellular(void);
void ResetFlash(void);
void ResetTVOC(void);
void EnableEEPROM(void);
void EnableTVOC(void);
void EnablePRES(void);
void DisableTVOC(void);
void DisablePRES(void);






#endif