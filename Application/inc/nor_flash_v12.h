#ifndef __NOR_FLASH_V12__
#define __NOR_FLASH_V12__

#include "nor_flash.h"

#define PPG_SAMPLES_IN_PAGE 56

#define PPG_FLAG_REGISTERS (PPG_SAMPLES_IN_PAGE / 4)
#define ECG_FLAG_REGISTERS (PPG_SAMPLES_IN_PAGE / 8)

#define NUMBER_OF_PEAKS_IN_PAGE 5

//#define FLASH_ERASE_CYCLES 200 // approx. 200 * 16mS

typedef enum {
  PAGE_TYPE_HEADER,
  PAGE_TYPE_PPG,
  PAGE_TYPE_ECG0,
  PAGE_TYPE_DEBUG1,
  PAGE_TYPE_ECG1,
  PAGE_TYPE_BP0,
  PAGE_TYPE_BP1,
  PAGE_TYPE_DEBUG2,
} NOR_PAGE_TYPE;

typedef enum {
  PSEL_NONE,               // 0
  PSEL_PPG1,               // 1
  PSEL_AFIB_RR_COUNT,      // 2
  PSEL_AFIB_AF1_COUNT,     // 3
  PSEL_ECG_AFIB_EFFECTIVE_RADIUS_ACC,// 4
  PSEL_PPG_SNR,            // 5
  PSEL_ART_NOISE,          // 6
  PSEL_PPG_AFIB_AV,        // 7
  PSEL_FLAGS,              // 8
  PSEL_PPG_AFIB_EFFECTIVE_RADIUS,  // 9
  PSEL_PPG_AFIB_ANGLE,     // 10
  PSEL_ART_THRESHOLD,      // 11
  PSEL_PPG_AF1_FOUND,      // 12
  PSEL_PPG_AFIB_RADIUS,    // 13
  PSEL_PPG_AFIB_RR,        // 14
  PSEL_ART_LEVEL,          // 15
  PSEL_MAIN_LOOP_COUNT,            // 16
  PSEL_ECG_AFIB_EFFECTIVE_RADIUS,    // 17
  PSEL_ECG_AFIB_AV,        // 18
  PSEL_ECG_RAW_DATA,       // 19
  PSEL_EF60,               // 20
  PSEL_ECG_FILTERED,       // 21
  PSEL_ECG_AFIB_RR_ACC,    // 22
  PSEL_ECG_AF1_FOUND,      // 23
  PSEL_ECG_AFIB_ANGLE,     // 24
  PSEL_ACC_LEVEL,          // 25
  PSEL_ECG_DISP_DATA,      // 26
  PSEL_ACC_ENERGY,         // 27
  PSEL_BP_OUTPUT,          // 28
  PSEL_PPG_NO_SPIKES,      // 29
  PSEL_DISPLAY_PPG_HR,     // 30
  PSEL_PPG_RAW_DATA_MOV3,  // 31
} PARAM_SELECT;            

typedef enum {
  PSEL4_NONE = 0,               
  PSEL4_XYZ =1 ,                
  PSEL4_FLAGS = 2,              
  PSEL4_PPG_GEMINY_DATA = 3,    
  PSEL4_ECG_GEMINY_DATA = 4,    
  PSEL4_PPG_AFIB_DATA = 5,      
  PSEL4_ECG_AFIB_DATA = 6,      
  PSEL4_PPG_STEP5_DATA = 7,     
  PSEL4_DISPLAY_HEART_RATES = 8,
  PSEL4_ECG_STEPS_1_3 = 9,      
  PSEL4_PPG_STEPS_1_3 = 10,     
  PSEL4_BLOOD_PRESSURE = 32,    
  PSEL4_PPG_RELATED = 33,       
  PSEL4_ART_RELATED = 34,       
  PSEL4_ECG_RELATED = 35,       
  PSEL4_ECG_FILTER_SMOOTH  = 36,
  PSEL4_ECG_FILTER_QUALITY = 37,
  PSEL4_ECG_STD_INSTITUTE  = 38,
  PSEL4_ECG_RAD_DRR_AV_DIFF = 39,
  PSEL4_PPG_RAD_DRR_AV_DIFF = 40,
  PSEL4_PPG1_LOAD_ARTTH_ARTLVL = 41,
  PSEL4_ECG_UNFILTERED = 42,
  PSEL4_ECG_QUAL_SORT = 43,
  PSEL4_PPG1_MAVG = 44,
  PSEL4_PPG1_MAVG2 = 45,
  PSEL4_PPG1_SII_EF = 46,
} PARAM4_SELECT;            


typedef struct {
  u32 flags;
  u16 config;
} PPG_FLAG_CONFIG_STRUCT;


typedef enum {
  FEMALE,
  MALE
} GENDER;

typedef __packed struct {
  DATE_TIME dt;
  NOR_PAGE_TYPE type;
  u64 McuID;
  u32 UserID;
  u16 YearOfBirth;
  u8  SkinType;
  u8  BMI;
  GENDER mf;
  u8  FirmwareVersion[4];
  u8  DebugParameter[4];
  u8  ServiceParams[32];
  u8  spare[218-32];
  u16 PageChecksum;
} NOR_HEADER_STRUCT;


typedef __packed struct {
  DATE_TIME dt;
  NOR_PAGE_TYPE type;
  u8 PeakPlace[NUMBER_OF_PEAKS_IN_PAGE];
  u8 HeartRate[NUMBER_OF_PEAKS_IN_PAGE];
  u8 PpgArt[PPG_SAMPLES_IN_PAGE][3];
  u8 XYZ[PPG_SAMPLES_IN_PAGE/4][3];
  PPG_FLAG_CONFIG_STRUCT fc[4];
  u16 PpgAfibAB;
  u16 PageChecksum;
} NOR_PPG_STRUCT;

typedef struct {
  u8 protect1[128];
  NOR_PPG_STRUCT ppgs;
  u8 protect2[128];
} PROT_PPG_STRUCT;

typedef __packed struct {
  DATE_TIME dt;
  NOR_PAGE_TYPE type;
  u8 PeakPlace[NUMBER_OF_PEAKS_IN_PAGE];
  u8 HeartRate[NUMBER_OF_PEAKS_IN_PAGE];
  u16 ECGsample[PPG_SAMPLES_IN_PAGE*2];
  u8 flags[8];
  u16 spare[2];
  u16 PageChecksum;
} NOR_ECG_STRUCT;
  
typedef __packed struct {
  DATE_TIME dt;
  NOR_PAGE_TYPE type;
  u32 params[PPG_SAMPLES_IN_PAGE];
  PARAM_SELECT P1;
  PARAM_SELECT P2;
  PARAM_SELECT P3;
  PARAM4_SELECT P4;
  u8 spare[18];
  u16 PageChecksum;
} NOR_DEBUG_STRUCT;  

typedef __packed struct {
  DATE_TIME dt;
  NOR_PAGE_TYPE type;
  u16 BpSample[PPG_SAMPLES_IN_PAGE * 2];
  u8 spare[22];
  u16 PageChecksum;
} NOR_BP_STRUCT;


typedef enum {
FLUSH_STATE_IDLE,
FLUSH_STATE_WRT_BP0,
FLUSH_STATE_WRT_BP1,
FLUSH_STATE_WRT_DBG,
FLUSH_STATE_WRT_DBG2,
FLUSH_STATE_WRT_ECG0,
FLUSH_STATE_WRT_ECG1,
FLUSH_STATE_WRT_PPG,
FLUSH_STATE_FINISH
} FLUSH_STATES;

//typedef __packed struct {
//u32 spare1		            : 1;
//u32 ECG_Quality_high	             : 1;
//u32 PPG_SNR_High                   : 1;
//u32 Artifact_energy_high           : 1;
//u32 Accelerometer_energy_high      : 1;
//u32 PPG_Good                       : 1;
//u32 spare3		            : 1;
//u32 Low_Battery                    : 1;
//u32 Extremely_low_battery          : 1;
//u32 Low_Flash_memory               : 1;
//u32 Extremely_Low_Flash_memory     : 1;
//u32 No_watch_memory                : 1;
//u32 Enter_sleep                    : 1;
//u32 Button_sleep                   : 1;
//u32 Not_measuring                  : 1;
//u32 open_sensor                    : 1;
//u32 First_20_seconds               : 1;
//u32 FindParameters                 : 1;
//u32 PrePostFP_time                 : 1;
//u32 spare4		            : 1;
//u32 Button_wake                    : 1;
//u32 Short_press                    : 1;
//u32 High_low_power_mode            : 1;
//u32 Current_AFIB_state             : 1;
//u32 Detected_AFIB_in_this_recording: 1;
//} V12_PPG_FLAGS;

typedef __packed struct {
u32 PPG_LED_power_100pct_40pct: 1;
u32 TIA_gain:                   1;
u32 LED_current:                3;
u32 PPG_LED_pulse_width:        4;
u32 ART_LED_pulse_width:        4;
} V12_PPG_CONFIG;

typedef __packed struct {
u32 spare:                           1;
u32 ECG_leads_on:                    1;
u32 Current_AFIB_state:              1;
u32 Detected_AFIB_in_this_recording: 1;
} V12_ECG_FLAGS;


extern NOR_HEADER_STRUCT NOR_HeaderPage;
extern NOR_PPG_STRUCT    NOR_PpgPage;
extern NOR_ECG_STRUCT    NOR_EcgPage;
extern NOR_DEBUG_STRUCT  NOR_DebugPage;
        
#define EXT_FLASH_LAST_BLOCK (nor_struct.u32NorFlashSize - 8192)



#define PPG_FLAG_ECG_PEAK_MASKED_OUT                   0x00000001
#define PPG_FLAG_MASK_ECG_QUAL_HIGH                    0x00000002
#define PPG_FLAG_MASK_PPG_SNR_High                     0x00000004
#define PPG_FLAG_MASK_Artifact_energy_high             0x00000008
#define PPG_FLAG_MASK_Accelerometer_energy_high        0x00000010
#define PPG_FLAG_MASK_PPG_Good                         0x00000020
#define PPG_FLAG_MASK_OStoFP		                       0x00000040
#define PPG_FLAG_MASK_Low_Battery                      0x00000080
#define PPG_FLAG_MASK_Extremely_low_battery            0x00000100
#define PPG_FLAG_MASK_Low_Flash_memory                 0x00000200
#define PPG_FLAG_MASK_Extremely_Low_Flash_memory       0x00000400
#define PPG_FLAG_MASK_No_watch_memory                  0x00000800
#define PPG_FLAG_MASK_Enter_sleep                      0x00001000
#define PPG_FLAG_MASK_Button_sleep                     0x00002000
#define PPG_FLAG_MASK_Current_AFIB_state2              0x00004000
#define PPG_FLAG_MASK_open_sensor                      0x00008000
#define PPG_FLAG_MASK_First_20_seconds                 0x00010000
#define PPG_FLAG_MASK_FindParameters                   0x00020000
#define PPG_FLAG_MASK_PrePostFP_time                   0x00040000
#define PPG_FLAG_MASK_PrePostAcc                       0x00080000
#define PPG_FLAG_MASK_RR_TYPE_C                        0x00100000
#define PPG_FLAG_MASK_spare4                           0x00200000
#define PPG_FLAG_MASK_High_low_power_mode              0x00400000
#define PPG_FLAG_MASK_Current_AFIB_state               0x00800000
#define PPG_FLAG_MASK_Detected_AFIB_in_this_recording  0x01000000
#define PPG_FLAG_MASK_Bp_Fp_Running                    0x02000000
#define PPG_FLAG_MASK_PPG_AFIB_step5_150_seconds       0x04000000
#define PPG_FLAG_MASK_PPG_AFIB_step5_27pct             0x08000000
#define PPG_FLAG_MASK_ECG_AFIB_AF1_found               0x10000000
#endif



























