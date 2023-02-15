#ifndef __MISC__
#define __MISC__

#define EVENT_DESC_STRING_LEN 20

extern char EventDesc[][EVENT_DESC_STRING_LEN];


typedef __packed struct {
  s32        value;      // value attached to event
  u32        time_delta; // time in samples from first event
  EVENT_TYPE event;      // event type
  u8         SPARE[3];
} EVENT_DATA;




#endif //  __MISC__