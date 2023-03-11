#ifndef __MISC__
#define __MISC__

#define EVENT_DESC_STRING_LEN 20

extern char EventDesc[][EVENT_DESC_STRING_LEN];


//typedef __packed struct {
//  s32        value;      // value attached to event
//  u32        time_delta; // time in samples from first event
//  EVENT_TYPE event;      // event type
//  u8         SPARE[3];
//} EVENT_DATA;
//

typedef enum {
  I2C_MEM  = 1,
  I2C_TVOC = 2,
  I2C_PRES = 3,
  } I2C_CHANNEL;

typedef enum {
  LTYPE_DATA = 1,
  LTYPE_EVENT = 2,
  LTYPE_ACC = 3,
  LTYPE_GYRO = 4,
} LTYPE;

typedef __packed struct {
LTYPE type;
u32   seconds;
float co2;
float tvoc;
float ethanol;
float h2;
s16   temperature;
s16   humidity;
u16   gas_resistance;
} LOG_STRUCT;


extern u32 RunTime_milli;
extern u32 RunTime_seconds;
extern u32 RunTime_minutes;

extern u32 TMR_1mS_Flags, TMR_1mS_Cnt;
extern u32 TMR_5mS_Flags;
extern u32 TMR_10mS_Flags, TMR_10mS_Cnt;
extern u32 TMR_100mS_Flags, TMR_100mS_Flags2, TMR_1Sec_Flags;
extern u32 TMR_1mS_Count,  TMR_10mS_Count; // used for timeout purposes by handler functions
extern u32 TMR_1min_count;


#endif //  __MISC__