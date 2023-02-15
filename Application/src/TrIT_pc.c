/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


#include "stdio.h"
#include "string.h"
#include "ctype.h"
#include "stdlib.h"
#include "math.h"

#include "setup.h"
#include "main.h"
#include "serial.h"
#include "nor_flash_v12.h"
#include "protocol.h"
#include "misc.h"
#include "intrinsics.h"

#define __USE_ACK__ 0 // tester ack usage

#define SIM_BLOCK_NUMBER   4
#define SIM_BLOCK_TYPE     8
#define SIM_BLOCK_PAYLOAD  10
#define SIM_DATA_BLOCK_DOWNLOAD_LEN (4+4+2+256+2) // block#+type+payload+CRC



extern u32 g_u32TestFlashErasedCtr, g_u16SampleDumpRequested, g_u16SampleResendRequest;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern u8 RxBuffer[8][USART_IN_BUF_LEN];   // low level serial input buffer
extern u32 RxCount[8], RxInpIdx[8], RxOutIdx[8]; // low level input/output pointers and byte counter
extern u32 PC_chars, watch_chars, watch_data_length;
extern u8 service_params[];
extern u32 simulation_runs[SIM_DIRECTORY_SZ], simulation_runX, simulation_runLen;
extern u8 skip_this_simulation;
extern u16 g_u16SampleDumpBuffer[SAMPLE_DUMP_BUF_SZ];
extern V12_WATCH_STATUS g_sV12WatchStatus;
extern u32 watch_hw, watch_fw;
extern V12_WATCH_STATUS full_watch_status;
extern u32 latest_received_page, delay_between_chars;
extern SYSTEM_STATUS SystemStatus;
extern s32 servo_position;
extern bool QueryWatch;
extern bool g_bCheckIncomingEvents;




char g_cRequestType;
u32 cmd_state, cmdidx, comm_ptr, message_timeout, sim_lines, total_lines;
char command[512];
u8   upload_page[300], latest_upload_page[256];
char command_word[32][64];
u32 line_words, first_simulation_index, last_simulation_index, one_shot_simulation;
u32 simulation_index, flash_storage_address, flash_last_sim_address, simulation_base;
u32 pc_command_error, pc_upload_requests, g_u32ReturnWatchAck;
bool send_to_pc, g_bTesterChecksum;
u64 watch_id;
u32 current_simulation_entry, current_page_request;
u32 upload_flag;
u32 sim_blocks, g_u32EventUploadCounter, g_u32EventRepeat;

//u8  UplStep[4096];
//u32 GtNtimestamp[4096];
//u32 GtNtimestampX;

u32 total_received_bin_blocks, same_counter;

u32 latest_page_number = 0xFFFFFFFF;

u32 GtS_retries[2];

u32 BypassDiscoMCU, g_u32EventUploadType;

bool send_ok, g_bTesterMode;


// watch upload data
NOR_HEADER_STRUCT NOR_HeaderPage;
NOR_PPG_STRUCT    NOR_PpgPg;
NOR_ECG_STRUCT    NOR_EcgPg[2];   
NOR_DEBUG_STRUCT  NOR_DebugPg[2];
bool ecg_present[2], debug_present[2], ppg_present;
u32 upload_page_arrived, pages_sent_to_uploader;

u32 gt_PageUploadRequests, gt_u32T0, gt_u32TotalUploadTime;

BIN_UPLOAD_PAGE recorded_data_cache[BIN_CACHE_SZ];
u32 recorded_data_cache_filled, recorded_data_cache_last_addr;

u32 UploadChecksumErrorsCtr;

SIM_BLOCK_DATA sim_data[32];
SIM_DIR_BLOCK sim_block, sim_directory[SIM_DIRECTORY_SZ];
bool simulation_data_download;

bool simulation_flag, PC_uploader;

// bin downloader data
char latest_request[22];
u32  request_timer, download_requests, request_retries;
u32 download_bin_retry_timer;



char EventDesc[][EVENT_DESC_STRING_LEN] = {
  "NONE        ",   //  0 no event
  "EPEAK       ",   //  1 ECG peak
  "PPEAK       ",   //  2 PPG peak
  "RESP_RATE   ",   //  3 respiration rate
  "BP_SYS      ",   //  4 
  "BP_DIA      ",   //  5 
  "SPO2        ",   //  6 SPO2 peak
  "TEMPERATURE ",   //  7 
  "ECGAF       ",   //  8 ECG AF
  "PPGAF       ",   //  9 PPG AF
  "PPGNAF      ",   // 10 PPG AF finished
  "PPG_BAD     ",   // 11 PPG is disabled by bad conditions
  "PPG_GOOD    ",   // 12 PPG is now in good condition
  "ECG_QUAL_L  ",   // 13 
  "ECG_QUAL_H  ",   // 14 
  "LDON        ",   // 15 ECG lead on
  "LDOFF       ",   // 16 ECG lead off
  "PPGOS       ",   // 17 PPG open sensor
  "PPGSG       ",   // 18 PPG signal returned
  "BRADY_ON    ",   // 19 
  "BRADY_OFF   ",   // 20 
  "TACHY_ON    ",   // 21 
  "TACHY_OFF   ",   // 22 
  "PAUSE_ON    ",   // 23 
  "PAUSE_OFF   ",   // 24 
  "CRADLE_IN   ",   // 25 watch is put into cradle
  "CRADLE_OUT  ",   // 26 watch is removed from cradle
  "POWERUP     ",   // 27 watch started working
  "POWERDOWN   ",   // 28 button press or battery too low cause watch to go to deep sleep mode
  "            ",   // 29
  "            ",   // 30
  "STEPS       ",   // 31
  "CALORIES    ",   // 32
  "            ",   // 33
  "            ",   // 34
  "            ",   // 35
  "            ",   // 36
  "            ",   // 37
  "            ",   // 38
  "            ",   // 39
  "ECGOK       ",   // 40 ECG OK after 20 beats
  "ECGFAIL     ",   // 41 ECG timeout (10 minutes)
  "BLE_PROB    ",   // 42 BLE Problem
  "FE_RESET    ",   // 43 FE reset
  "CALIB_LOD_0 ",   // 44 Calibration #0
  "CALIB_LOD_1 ",   // 45
  "CALIB_LOD_2 ",   // 46
  "CALIB_LOD_3 ",   // 47
  "CALIB_LOD_4 ",   // 48
  "            ",   // 49
  "            ",   // 50
  "            ",   // 51
  "            ",   // 52
  "            ",   // 53
  "            ",   // 54
  "            ",   // 55
  "            ",   // 56
  "            ",   // 57
  "            ",   // 58
  "CALIB_LOD_5 ",   // 59
  "CALIB_LOD_6 ",   // 50
  "CALIB_LOD_7 ",   // 51 Calibration #7
};










void send_service_param(u32 index, u8 value);
void V12_FlashWritePage(u32 u32Add, u32 u32Len, u8* pBuff);
void V12_FlashReadData(u32 u32Add, u32 u32Len, u8 *DestBuffer);
void FlashBulkErase(void);
void RTC_store_date_time(DATE_TIME *idt);
void send_block(u32 length, u8 *message);
u16 CalculateCRC(u8 *BufStart, u16 BufLen, u16 crc_init);
void SetWatchDateTime(void);
void load_sim_directory(void);
void store_sim_dir_entry(u32 entry);
void SendTesterCommandToWatch(SAMPLE_DUMP_TYPE data_type, u32 samples);
void delay_ms(u32 delay);
void comm_reinit(void);
void erase_watch_memory(void);
void SendWatchDataRequest(u32 address, u32 source);
void SendTestDataToPC(void);
void SendWatchID(u64 dword);
u32  get_hex(void);
u32  get_bin(void);
void StartStopRecord(u32 start_stop);
void SendAppMessageToGui(u8 *data);
void SendEventToFlash(u8 *data);
void SendChangeSystemStatus(u8 *data);
void send_page_data_to_pc(u8 *data, u32 addr);
void parse_command_date_time(void);
void SetupServoPWM(u32 microSec, s32 inc, u32 steps);
void DumpUploadChecksumErrors(void);
void test_artifact(u32 loops, u32 inc, u32 steps);
void start_stepper(u32 time, u32 *speed, u32 index);
void stop_stepper(void);
void MX_UART5_Init(void);
void SendSimulationNameString(char *sim_name);




/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void bypass_comm(u32 bypass)
  {
  BypassDiscoMCU = bypass;
  MX_UART5_Init();
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//void TimestampUpload(u8 type)
//  {
//  if (GtNtimestampX < 4095)
//    {
//    UplStep[GtNtimestampX] = type;
//    GtNtimestamp[GtNtimestampX] = TMR_1mS_Count;
//    GtNtimestampX++;
//    }
//  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 get_char(void)
  {
  return command[comm_ptr++];
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 peek_char(void)
  {
  return command[comm_ptr];
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void skip_spc(void)
  {

//  event_Stack_depth();

  while (command[comm_ptr] && (command[comm_ptr] == ',' || command[comm_ptr] == 9 || command[comm_ptr] == ' '))
    {
    comm_ptr++;
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 get_string(char *str, char delim)
  {
  u32 len = 0;
  skip_spc();
  while (peek_char() && peek_char() != delim && peek_char() != '\r')
    {
    *str++ = get_char();
    len++;
    }
  *str = 0;
  return len;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
s32 get_int(void)
  {
  s32 num = 0, sign = 1;
  skip_spc();
  if (command[comm_ptr])
    {
    num = 0;
    if (toupper(peek_char()) == 'X') // hexadecimal number ?
      {
      get_char(); // skip 'x' char
      num = get_hex();
      return num;
      }
    else if (toupper(peek_char()) == 'B') // binary number ?
      {
      get_char(); // skip 'b' char
      num = get_bin();
      return num;
      }
    
    if (peek_char() == '-')
      {
      sign = -1;
      get_char(); // skip '-' char
      }
    while (isdigit(command[comm_ptr]))
      num = (num * 10) + (command[comm_ptr++] - '0');
    }
//  skip_spc();
  return num * sign;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void store_sim_block(void)
  {
  u32 dir_entry;
  for (dir_entry = 0; dir_entry < SIM_DIRECTORY_SZ; dir_entry++)
    {
    if (sim_directory[dir_entry].name[0] == 0xFF)
      {
      break;
      }
    }
  memcpy((void *)&sim_directory[dir_entry], (void *)&sim_block, sizeof(sim_block));
  store_sim_dir_entry(dir_entry);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void list_sim_entries(void)
  {
  u32 dir_entry;
  char buf[200];
  send_message("\r\nSimulation directory\r\n");
  for (dir_entry = 0; dir_entry < SIM_DIRECTORY_SZ; dir_entry++)
    {
    if (sim_directory[dir_entry].name[0] != 0xFF)
      {
      s32 blocks = sim_directory[dir_entry].last_block - sim_directory[dir_entry].first_block;
      if (blocks < 0)
        {
        blocks = 0;
        }
      sprintf(buf, "simulation %2d: %-32s;    from %6d to %6d, %6d samples, %6d seconds\r\n",
              dir_entry, sim_directory[dir_entry].name, 
              sim_directory[dir_entry].first_block, sim_directory[dir_entry].last_block,
              blocks * 4, (blocks * 4 + 1) / 256);
      send_message(buf);
      }
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 dir_entries(void)
  {
  u32 dir_entry, count = 0;
  for (dir_entry = 0; dir_entry < SIM_DIRECTORY_SZ; dir_entry++)
    {
    if (sim_directory[dir_entry].name[0] != 0xFF)
      {
      count++;
      }
    }
  return count;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_downloader_message(char *request)
  {
  strcpy(latest_request, request);
  send_message(request);
  request_timer = 0;
  download_requests++;
  download_bin_retry_timer = 10;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void receive_bin_simulation_data(void)
  {
  get_string(sim_block.name, ';');
  sim_block.first_block = flash_storage_address / 128;

  total_received_bin_blocks = 0;
  same_counter = 0;
  
  send_downloader_message("GtN\r"); // start sim data download
  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void end_of_bin_download(void)
  {
  flash_last_sim_address = flash_storage_address;
//  send_message("\r\nEnd download\r\n");
  sim_block.last_block = flash_last_sim_address / 128 - 1;
  store_sim_block();
  cmd_state = 0;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 flash_read_errors;
void dump_sim_data(u32 addr, u32 blocks)
  {
  u32 sample, sampleX;
  char buf[100];
  SIM_BLOCK_DATA simdat[8];
  SIM_BLOCK_DATA vsimdat[8];
  flash_read_errors = 0;
  sampleX = (addr / 16) + 1; // address is multiples of 8 samples
  send_message("\r\n Sample,  Ecg,  Ppg,  Art,   IR,  RED, AccX, AccY, AccZ,GyroX,GyroY,GyroZ,Rate\r\n");
  while (blocks--)
    {
    V12_FlashReadData(addr, 256, (u8 *)&simdat[0]);
    delay_ms(10);
    V12_FlashReadData(addr, 256, (u8 *)&vsimdat[0]);
    
    if (memcmp((u8 *)&simdat[0], (u8 *)&vsimdat[0], sizeof(simdat)) != 0)
      {
      __NOP();
      flash_read_errors++;
      }
    
    
    if (simdat[0].AccX == -1 && simdat[0].AccY == -1 && simdat[0].AccZ == -1 && 
        simdat[0].art == -1 && simdat[0].ppg == -1)
      {
      strcpy(buf, "\r\nFlash page empty\r\n");
      send_message(buf);
      }
    else
      {
      for (sample = 0; sample < 8; sample++)
        {
        sprintf(buf, "%7d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%5d,%d\r\n",
                sampleX++,
                simdat[sample].ecg,
                simdat[sample].ppg,     simdat[sample].art,
                simdat[sample].spo2_ir, simdat[sample].spo2_red,
                simdat[sample].AccX,    simdat[sample].AccY,    simdat[sample].AccZ,
                simdat[sample].GyroX,   simdat[sample].GyroY,   simdat[sample].GyroZ,
                simdat[sample].Rate*64);
        
        send_message(buf);
        delay_ms(10);
        if (get_comm_buffer_status(2))
          {
          char chr = get_usart_byte(2);
          if (chr == 3)
            {
            return;
            }
          else if (chr == ' ')
            {
            while (!get_comm_buffer_status(2))
              {
              }
            get_usart_byte(2);
            }
          }
        delay_ms(4);
        }
      addr += 256;
      }
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void print_mcu_id(char *id_str)
  {
  union {
  u8  byt[8];
  u32 wrd[2];
  } u;
  u32 id[2];
  u.wrd[0] = watch_id;
  u.wrd[1] = watch_id >> 32;
  id[0] = make32(u.byt[4], u.byt[5], u.byt[6], u.byt[7]);
  id[1] = make32(u.byt[0], u.byt[1], u.byt[2], u.byt[3]);
  sprintf(id_str, "%08X%08X", id[1], id[0]);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_mcu_id(void)
  {
  union {
  u8  b[4];
  u32 w;
  } u;
  char buf[66];
  strcpy(buf, "$ID ");
  print_mcu_id(buf+4);
  delay_ms(10);
  strcat(buf, "\r\n");
  send_message(buf);
  if (watch_id != 0)
    {
    u.w = full_watch_status.fw_version;
    sprintf(buf, "Ftype=%d Fversion=%d.%d.%d.%d  Bootloader=%08X\r\n",
            full_watch_status.firmware_type, 
            u.b[3], u.b[2], u.b[1], u.b[0],
            full_watch_status.Bootloader);
    send_message(buf);
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
#define MCU_ID_ADDRESS 0x1FFF7A10 
void send_simulator_id(void)
  {
  char buf[64];
  u32 McuID[2];
  McuID[0] = *(u32 *)MCU_ID_ADDRESS;
  McuID[1] = *(u32 *)MCU_ID_ADDRESS+4;
  sprintf(buf, "\r\nSimID=%08X%08X\r\n", McuID[1],McuID[0]);
  send_message(buf);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_year_of_birth(void)
  {
  char buf[33];
  u32 year_of_birth;
  delay_ms(10);
  year_of_birth = 1900 + (u32)service_params[11];
  sprintf(buf, "$year %d\r\n", year_of_birth);
  send_message(buf);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_service_parameters(void)
  {
  char buf[80];
  u32 index;
  if (g_bTesterMode)
    {
    send_message("#RR ");
    for (index = 0; index < 32; index++)
      {
      sprintf(buf, "%d ", (u32)service_params[index]);
      send_message(buf);
      }
    }
  else
    {
    send_message("\r\n\nService Parameters:\r\n\n");
    for (index = 0; index < 32; index++)
      {
      sprintf(buf, "R%2d: %3d\r\n", index, (u32)service_params[index]);
      send_message(buf);
      }
    }
  send_message("\r\n");
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void display_help(void)
  {
  char buf[80];
  send_message("\r\n\n");
  strcpy(buf, "$R                       Reset simulator\r\n"); send_message(buf);                
  strcpy(buf, "$F <b>                   First sim block\r\n"); send_message(buf);                
  strcpy(buf, "$L <b>                   Last sim block\r\n"); send_message(buf);                 
  strcpy(buf, "$T [<num>[ <num>...]     sTart simulation(s)\r\n"); send_message(buf);               
  strcpy(buf, "$P                       stoP simulation\r\n"); send_message(buf);                
  strcpy(buf, "$O                       One shot\r\n"); send_message(buf);                       
  strcpy(buf, "$M                       Multi pass\r\n"); send_message(buf);                     
  strcpy(buf, "$D [<name>]              Download simulation file\r\n"); send_message(buf);       
  strcpy(buf, "$C [<name>]              Continue with another simulation file download\r\n"); send_message(buf);   
  strcpy(buf, "$Y                       list simulation file directorY\r\n"); send_message(buf);   
  strcpy(buf, "$E                       Erase simulation memory\r\n"); send_message(buf);        
  strcpy(buf, "$U <b> <n>               Upload simulation blocks\r\n"); send_message(buf);       
  strcpy(buf, "$U @<y>                  Upload simulation data from simulation dir entry\r\n"); send_message(buf);       
  strcpy(buf, "$N                       Next simulation block\r\n"); send_message(buf);          
  strcpy(buf, "$Z                       Show simulation statistics\r\n"); send_message(buf);     
  strcpy(buf, "$?                       Dump first simulation block\r\n"); send_message(buf);    
  strcpy(buf, "$X                       Erase watch flash memory\r\n"); send_message(buf);       
  strcpy(buf, "$I                       Return watch ID\r\n"); send_message(buf);                
  strcpy(buf, "$W <p> <d>               Write parameter in watch\r\n"); send_message(buf);
  strcpy(buf, "$A <YOB>                 Set wearer's year of birth in watch\r\n"); send_message(buf);
  strcpy(buf, "$QY                      Return year of birth\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$QR                      Dump seRvice parameters\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$QP <samples>            Request PPG samples\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$QE <samples>            Request ECG samples\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$QL <samples>            Request Accelerometer samples\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$QA <samples>            Request Artifact samples\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$QV                      Request watch voltage\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$QD                      Dump latest data again\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$QI                      Request watch information\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$SR <1/0>                Set/reset external relays\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$SG <event> [<param>]    Send event to GUI\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$SS <element> <value>    Set SystemStatus element\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BR 99                   Reset watch\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BE 99                   Enable test mode\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BD                      Disable test mode\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BA                      Test watch RAM\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BF                      Test Data flash\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BC                      Interfere with comm CSC\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BM                      Interfere with flash checksum computation\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BV                      Stop vibrator\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BW                      Stop watchdog refresh\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BL                      Limit flash size (R&D only)\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BI                      Restore watch default service param values\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$BB                      disable comm timeout in watch\r\n"); send_message(buf);                delay_ms(2);
  strcpy(buf, "$SD <string>             Display string on watch debug screen\r\n"); send_message(buf);                delay_ms(2);
  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void get_simulation_parameters(void)
  {
  u32 index, guard = 0;
  char buf[80];
  index = get_int();
  skip_spc();
  if (peek_char() == '!')
    {
    get_char(); // skip !
    guard = 2975;
    }
  if (index < SIM_DIRECTORY_SZ)
    {
    current_simulation_entry = index;
    SendSimulationNameString(sim_directory[index].name);
    if (isalnum(sim_directory[index].name[0]) || guard == 2975)
      {
      first_simulation_index = sim_directory[index].first_block;
      last_simulation_index = sim_directory[index].last_block;
      sprintf(buf, "\r\nExecuting simulation #%d: %s\r\n",
              index, sim_directory[index].name);
      send_message(buf);
      }
    skip_spc();
    simulation_runX = 0;
    simulation_runLen = 0;
    one_shot_simulation = 1;
    while (isdigit(peek_char()))
      {
      index = get_int();
      if (index < SIM_DIRECTORY_SZ && isalnum(sim_directory[index].name[0]))
        {
        simulation_runs[simulation_runLen++] = index;
        }
      skip_spc();
      }
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_tester_message(char *str)
  {
  char *message = str, chr;
  u16 checksum = 0;
  if (g_bTesterChecksum)
    {
    while (*str)
      {
      checksum += (u16)*str++;
      }
    checksum &= 0xFF;
    sprintf(str, ";%d\r\n", checksum);
    comm_reinit();
#if __USE_ACK__ == 1
    for (u32 repeat = 0; repeat < 10; repeat++)
      {
      send_message(message);
      for (u32 delay = 0; delay < 50; delay++)
        {
        delay_ms(1);
        if (get_comm_buffer_status(2))
          {
          chr = get_usart_byte(2);
          if (chr == '1')
            {
            return;
            }
          }
        }
      }
#else
    send_message(message);
#endif    
    }
  else
    {
    strcat(str, "\r\n");
    send_message(str);
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void return_watch_samples(SAMPLE_DUMP_TYPE data_type)
  {
  u32 samples;

//  event_Stack_depth();

  skip_spc();
  samples = get_int();
  g_u16SampleDumpRequested = samples;
  g_u16SampleResendRequest = g_u16SampleDumpRequested;
  if (samples > 1024)
    {
    samples = 1024;
    }
  SendTesterCommandToWatch(data_type, samples+2);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void return_watch_voltage(void)
  {
  char buf[64];
  sprintf(buf, "#RV %4.2f %d %d", 
          (float)full_watch_status.battery_voltage / 100, 
          full_watch_status.battery_chrg_pct & 0x7F,
          full_watch_status.battery_chrg_pct >> 7
          );
  send_tester_message(buf);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void return_watch_button(void)
  {
  char buf[64];
  sprintf(buf, "#RB %d", !g_sV12WatchStatus.button_pressed);
  send_tester_message(buf);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void return_watch_info(void)
  {
  char buf[64], fw[16], hw[16], id_str[64];
  u32 id[2], fwi, hwi;
  id[0] = watch_id;
  id[1] = watch_id >> 32;
  hwi = full_watch_status.hw_version, 
  fwi = full_watch_status.fw_version,
  sprintf(fw, "%d.%d.%d.%d", make8(fwi, 3), make8(fwi, 2), make8(fwi, 1), make8(fwi, 0));
  sprintf(hw, "%d.%d", make8(hwi, 1), make8(hwi, 0));
  print_mcu_id(id_str);
  delay_ms(10);
  sprintf(buf, "#RI %s %s %s %s %d", 
          id_str,
          hw, fw,
          SW_VERSION,
          full_watch_status.BIT_result);
  send_tester_message(buf);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_tester_ack(char ack_nack)
  {
  HAL_UART_Transmit(&huart2, (u8 *)ack_nack, 1, 2);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
bool check_tester_command(void)
  {
  u16 checksum = 0, rec_chk = 0;
  char *ptr = command;
  if (!g_bTesterChecksum)
    {
    return true;
    }
  
  while (*ptr && *ptr != ';')
    {
    checksum += (u16)*ptr++;
    }
  if (*ptr == ';')
    {
    ptr++; // skip ';'
    while (isdigit(*ptr))
      {
      rec_chk = rec_chk * 10 + (u16)(*ptr++ - '0');
      }
    if (rec_chk == (checksum & 0xFF))
      {
      return true;
      }
    }
  return false;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void dump_system_status(void)
  {
  char buf[300];

  sprintf(buf, "\r\n\nWatch SN=%08X, BE=%s, FE=%s\r\n",
          (u32)SystemStatus.SerialNumber,     
          SystemStatus.BEversion,//[12],    
          SystemStatus.FEversion);//[12],    
  send_message(buf);
  
  sprintf(buf, "ArtF=%d, SnrF=%d, AccF=%d\r\n",
          SystemStatus.ArtFlag,          
          SystemStatus.SnrFlag,          
          SystemStatus.MoveFlag);
  send_message(buf);
  
  sprintf(buf, "Flash Rec=%d, Erase=%d, Pages=%d, Status=%d, Uploaded=%d\r\n",
          SystemStatus.Recording,        
          SystemStatus.Erasing,          
          SystemStatus.RecordedPages,    
          SystemStatus.AG_MemoryStatus,     
          SystemStatus.UploadBlock);
  send_message(buf);
  
  sprintf(buf, "Ecg HR=%d, Result=%s, Text=%s, Qual=%d, BarPct=%d, Status=%d\r\n",
          SystemStatus.EcgHR,            
          SystemStatus.EcgResult,//[4],     
          SystemStatus.EcgTestText,//[12],  
          SystemStatus.EcgQuality,       
          SystemStatus.StatusBarPct,     
          SystemStatus.EcgStatus);
  send_message(buf);
  
  sprintf(buf, "Ppg PR=%d, Result=%s, PpgAF=%d, AfStat=%d\r\n",
          SystemStatus.PpgHR,            
          SystemStatus.PpgResult,//[4],     
          SystemStatus.PpgStatus,        
          SystemStatus.AfStatus);        
  send_message(buf);
  
  sprintf(buf, "ArtTh=%d, Airplane=%d\r\n",
          SystemStatus.ArtThreshold,     
          SystemStatus.AirplaneMode);
  send_message(buf);

  sprintf(buf, "Resp=%d, SpO2=%d, BPsys=%d, BPdias=%d, Temp=%4.1f\r\n",
          SystemStatus.RespRateTotal,    
          SystemStatus.SpO2pct,          
          SystemStatus.BPsystolic,       
          SystemStatus.BPdiastolic,      
          (float)SystemStatus.TemperatureData.Temperature / 10);
  send_message(buf);
  
//  SystemStatus.DateTime,    	    

  sprintf(buf, "BLE paired=%d, Status=%d\r\n",
          SystemStatus.PairStatus,       
          SystemStatus.AG_BluetoothStatus);
  send_message(buf);

  sprintf(buf, "Resets=%d, Demo=%d, Lang=%d, BatPct=%d, Charge=%d\r\n",
          SystemStatus.Reset_number,
          SystemStatus.Demo_Mode,        
          SystemStatus.language,
          SystemStatus.BatteryPct,       
          SystemStatus.Charging);
  send_message(buf);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//void dump_uploader_times(void)
//  {
//  char buf[64];
//  u32 index;
//  send_message("\r\nUploader transaction times\r\n");
//  for (index = 1; index < GtNtimestampX; index++)
//    {
//    sprintf(buf, "Step %d: Tdiff=%d\r\n", UplStep[index], GtNtimestamp[index]-GtNtimestamp[index-1]);
//    send_message(buf);
//    }
//  }

//u8  UplStep[4096];
//u32 GtNtimestamp[4096];
//u32 GtNtimestampX;

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void SendWatchTemeprature(void)
  {
  char buf[33];
  sprintf(buf, "\r\n#RT %d\r\n", full_watch_status.temperature);
  send_message(buf);
  }
  
/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void handle_tester_commands(char type)
  {
  u32 resend;
  char buf[44];
  if (!check_tester_command())
    {
//    send_message("0"); 
    return;
    }
  
  g_cRequestType = type;
  switch (type)
    {
    case 'A': return_watch_samples(DUMP_TYPE_ART); break;
    case 'B': return_watch_button(); break;
    case 'C': // request checksum test
      SendChecksumTestRequest();
      break;
    case 'D':  // dump data again
      resend = g_u16SampleResendRequest;
      return_watch_samples(DUMP_TYPE_RESEND); 
      g_u16SampleDumpRequested = resend;
      break; 
    case 'E': return_watch_samples(DUMP_TYPE_ECG); break;
    case 'F': // dump number of checksum errors detected while uploading to PC
      DumpUploadChecksumErrors();
      break;
      // G
      // H
    case 'I': return_watch_info(); break;
      // J
      // K
    case 'L': return_watch_samples(DUMP_TYPE_ACC); break;
      // M
      // N
    case 'O': 
      sprintf(buf, "\r\n\nServo position: %d\r\n", servo_position);
      send_message(buf);
      break;
    case 'P': return_watch_samples(DUMP_TYPE_PPG); break;
      // Q
    case 'R': // return all service parameters
      send_service_parameters();
      break;
    case 'S': return_watch_samples(DUMP_TYPE_SPO2); break;
    case 'T': 
      SendWatchTemeprature();
      break;
//    case 'U': dump_uploader_times(); break;// dump uploader times
    case 'V': return_watch_voltage(); break;
      // W
    case 'X': ; break;
    case 'Y': // return year of birth
      send_year_of_birth();
      break;
    case 'Z': dump_system_status(); break;
    default: /* send_message("0"); */ break;
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_ack_to_pc(void)
  {
  send_message("\r\nok\r\n");
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void SendTestDataToPC(void)
  {
  u32 index;
  char buf[256], value[11];
  u32 numbers_per_line = 32;
//  event_Stack_depth();

  if (g_cRequestType == 'T') // is this battery voltage dump? ($BT/$QT)
    {
    numbers_per_line = 1;
    for (g_u16SampleDumpRequested = SAMPLE_DUMP_BUF_SZ-1; 
         g_u16SampleDumpRequested > 10; 
         g_u16SampleDumpRequested--)
      {
      if (g_u16SampleDumpBuffer[g_u16SampleDumpRequested] != 0)
        {
        break;
        }
      }
    }
  
  for (index = 0; index < g_u16SampleDumpRequested; )
    {
    sprintf(buf, "#R%c ", g_cRequestType);
    for (u32 sendX = 0; sendX < numbers_per_line && index < g_u16SampleDumpRequested; sendX++)
      {
      s32 data = (s16)g_u16SampleDumpBuffer[index];
      sprintf(value, "%d ", data);
      strcat(buf, value);
      index++;
      }
    strcat(buf, "\r\n");
    send_tester_message(buf);
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void SetWatchID(void)
  {
  union {
  u8  byt[8];
  u64 dword;
  } u;
  char hex[8];
  
  if (get_char() == '*')
    {
    skip_spc();
    if (isxdigit(peek_char()))
      {
      for (u32 index = 0; index < 8; index++)
        {
        u.byt[index] = get_hex();
        }
      SendWatchID(u.dword);
      }
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_event_to_gui(void)
  {
  u8 event_type, buf[22];
  u32 parameters[4];
  event_type = get_int();
  parameters[0] = get_int();
  parameters[1] = get_int();
  parameters[2] = get_int();
  parameters[3] = get_int();
  buf[0] = event_type;
  memcpy(buf+1,  (u8 *)&parameters[0], 4);
  memcpy(buf+5,  (u8 *)&parameters[1], 4);
  memcpy(buf+9,  (u8 *)&parameters[2], 4);
  memcpy(buf+13, (u8 *)&parameters[3], 4);
  SendAppMessageToGui(buf);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_message_to_watch(u32 mtype)
  {
  u8 event_type, buf[22];
  u32 parameters[4];
  event_type = get_int();
  parameters[0] = get_int();
  parameters[1] = get_int();
  parameters[2] = get_int();
  parameters[3] = get_int();
  buf[0] = event_type;
  memcpy(buf+1,  (u8 *)&parameters[0], 4);
  memcpy(buf+5,  (u8 *)&parameters[1], 4);
  memcpy(buf+9,  (u8 *)&parameters[2], 4);
  memcpy(buf+13, (u8 *)&parameters[3], 4);
  switch (mtype)
    {
    case 1: SendAppMessageToGui(buf);    break;
    case 2: SendEventToFlash(buf);       break;
    case 3: SendChangeSystemStatus(buf); break;
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void set_artifact_calib_value(void)
  {
  u16 value;
  u8 watch_comm_tx_buf[8];
  value = get_int();
  watch_comm_tx_buf[0] = make8(value,0);
  watch_comm_tx_buf[1] = make8(value,1);
  ProtocolSendMessage(DevTypeWatch, OpcodeSetArtCalibValue, watch_comm_tx_buf, 4);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void dump_gui_message(AGGA_DIR dir, u8 *data)
  {
  char buf[88], origin[32];
  u8 event_type;
  u32 p1, p2, p3, p4;
  event_type = *data++;
  memcpy((u8 *)&p1, data, 4); data += 4;
  memcpy((u8 *)&p2, data, 4); data += 4;
  memcpy((u8 *)&p3, data, 4); data += 4;
  memcpy((u8 *)&p4, data, 4);
  switch (dir)
    {
    case AGGA_TO_APP : strcpy(origin, "from GUI to App"); break;
    case AGGA_TO_GUI : strcpy(origin, "from App to GUI"); break;
    }
  sprintf(buf, "\r\n\nMessage %s: et=%d, p1=%d p2=%d p3=%d p4=%d\r\n#RC %d\r\n",
          origin, event_type, p1, p2, p3, p4, p1);
  send_message(buf);
  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void set_external_relays(void)
  {
  u32 onoff;
  onoff = get_int();
  relay_connect(onoff & 1);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//void set_watch_mode(void)
//  {
//  u8 buf[33];
//  bool ok = true;
//  u32 mode;
//  skip_spc();
//  switch (toupper(get_char()))
//    {
//    case 'N': mode = OPERATION_MODE_3_0; break;
//    case 'D': mode = OPERATION_MODE_3_5; break;
//    default:
//      ok = false;
//      break;
//    }
//  if (ok)
//    {
//    memcpy(buf, (u8 *)&mode, 4);
//    g_u32ReturnWatchAck = 10;
//    ProtocolSendMessage(DevTypeWatch, OpcodeSetWatchMode, buf, 8);
//    }
//  else
//    {
//    send_message("\r\nWhat?\r\n");
//    }
//  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void set_watch_language(void)
  {
  u8 buf[33];
  u8 mode = 0;
  skip_spc();
  if (isdigit(peek_char()))
    {
    mode = get_int();
    }
  buf[0] = mode;
  g_u32ReturnWatchAck = 10;
  ProtocolSendMessage(DevTypeWatch, OpcodeSetWatchLanguage, buf, 2);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void set_FE_calib_tables(void)
  {
  u16 values[12];
  u32 index = 0;
  u8 mode = 0;
  skip_spc();
  while (index < 12)
    {
    values[index++] = get_int();
    if (peek_char() == 13)
      {
      break;
      }
    }
  if (index < 12)
    {
    send_message("\r\nNot enough values\r\n");
    return;
    }
  g_u32ReturnWatchAck = 10;
  ProtocolSendMessage(DevTypeWatch, OpcodeUpdateCalibValues, (u8 *)values, sizeof(values));
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void set_operation_flags(void)
  {
  u8 buf[33];
  u32 mode;
  skip_spc();
  mode = get_int();
  mode |= OPERATION_MODE_GUARD;
  memcpy(buf, (u8 *)&mode, 4);
  ProtocolSendMessage(DevTypeWatch, OpcodeSetOperationModeFlags, buf, 8);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void set_servo_time(void)
  {
  u32 time, steps;
  s32 increment;
  time = get_int();
  increment = get_int();
  steps = get_int();
  if (time == 0)
    {
    if (increment && steps)
      {
      u32 result = servo_position + increment * steps;
      if (result < 2500 || result > 3000)
        {
        send_message("\r\nValue out of range\r\n");
        return;
        }
      }
    else
      {
      send_message("\r\nNo values provided\r\n");
      return;
      }
    }
  else if (time < 2500 || time > 3000)
    {
    send_message("\r\nValue out of range\r\n");
    return;
    }
  SetupServoPWM(time, increment, steps);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void parse_stepper_mot_sequence(void)
  {
  u32 time, index, speed[8];
  time = get_int();
  for (index = 0; index < 8; index++)
    {
    speed[index] = get_int();
    if (speed[index] == 0)
      {
      break;
      }
    }
  if (time == 0)
    {
    stop_stepper();
    }
  else if (index) // at least one speed specified
    {
    start_stepper(time, speed, index);
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        send_extra_commands
// Description: $Sx commands
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 gt_u32MessagesTowatch;

void send_extra_commands(void)
  {
  char op;
  u32 opcode, value;
  gt_u32MessagesTowatch++;
  skip_spc();
  g_u32ReturnWatchAck = 10;
  send_ok = false;
  op = toupper(get_char());
  opcode = (u32)op - 0x40 + 180;
  switch (op)
    {
    case '#': set_operation_flags();    break;

    case 'A': set_artifact_calib_value(); break;
    // B
    case 'C': set_FE_calib_tables(); break;
    case 'D': skip_spc(); SendWatchDisplayString(&command[comm_ptr]); break;
    // E
    case 'F': send_message_to_watch(2); break;
    case 'G': send_message_to_watch(1); break;
    // H
    // I
    // J
    // K
    case 'L': set_watch_language();     break;
    // M
    // N
    case 'O': ServoHomePos(); send_ack_to_pc(); break;
    case 'P': set_servo_time(); send_ack_to_pc(); break;
    // Q
    case 'R': set_external_relays(); send_ok = true;   break;
    case 'S': send_message_to_watch(3); break;
    case 'T': parse_command_date_time(); break;
    // U
    // V
    // W
    case 'X': // stepper motor sequence
      parse_stepper_mot_sequence();
      break;
    // Y
    // Z
    default:
      value = get_int();
      send_service_param(opcode, (u8)value);
      break;
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void DumpUploadChecksumErrors(void)
  {
  char buf[55];
  sprintf(buf, "\r\nUpload checksum errors=%d\r\n", UploadChecksumErrorsCtr);
  send_message(buf);
  UploadChecksumErrorsCtr = 0;
  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_simulator_status(void)
  {
  char buf[99];
  sprintf(buf, "\r\n\nV12.1 simulator V%s\r\n", SW_VERSION);
  send_message(buf);
  send_simulator_id();
  sprintf(buf, "\r\nSimulation samples: %d, blocks: %d, rec pages: %d\r\n", 
          flash_last_sim_address / 16, flash_last_sim_address / 128,
          (flash_last_sim_address / 16) / 56);
  send_message(buf);
  sprintf(buf, "From block: %d, To block: %d, %s\r\nSimulation: %s\r\n",
          first_simulation_index, last_simulation_index,
          (one_shot_simulation != 0) ? "One shot" : "Multi pass",
          simulation_flag ? "ON" : "OFF");
  send_message(buf);
//  sprintf(buf, "\r\n\nServo position: %d\r\n", servo_position);
//  send_message(buf);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void more_BIT_commands(void)
  {
  char chr;
  u32 index, value;
  g_u32ReturnWatchAck = 10;
  skip_spc();
  chr = tolower(get_char());
  index = (u32)chr + 101 - 'A';
  skip_spc();
  value = get_int();
  send_service_param(index, value);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void SendPeronalData(u32 type)
  {
  u32 data, dspec;
  u8 buf[8];
  send_ok = false;
  g_u32ReturnWatchAck = 10;
  data = get_int();
  switch (type)
    {
    case 0: dspec = 11; data -= 1900; break;
    case 1: dspec = SvcOpUserHeight; break;
    case 3: dspec = SvcOpUserWeight; break;
    case 2: dspec = SvcOpUserGender; break;
    default: return;
    }
  send_service_param(dspec, data);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void OperateVibrator(void)
  {
  u32 onoff = get_int();
  send_ok = false;
  g_u32ReturnWatchAck = 10;
  send_service_param(SvcOpVibratorControl, onoff & 0xFF);
  }

/////////////////////////////////////////////////////////////////////
// Name:        MoreExtraCommands
// Description: $Jx commands
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void MoreExtraCommands(void)
  {
  u32 yes_no, loops, inc, steps;
  u8 buf[4];
  switch (toupper(get_char()))
    {
    case 'A': // test artifact using servo
      loops = get_int();
      inc = get_int();
      steps = get_int();
      test_artifact(loops, inc, steps);
      break;
    case 'B': // bypass disco MCU for direct comm between watch and PC
      send_ack_to_pc();
      bypass_comm(1);
      break;
    // C  
    case 'D': QueryWatch = false; break;
    case 'E': QueryWatch = true; break;
    // F
    case 'G': SendPeronalData(2);   break; // USER GENDER
    case 'H': SendPeronalData(1);   break; // USER HEIGHT
    // I
    case 'J': // control message checksum generation
      send_ok = false;
      skip_spc();
      yes_no = get_int();
      g_bTesterChecksum = false;
      if (yes_no == 1)
        {
        g_bTesterChecksum = true;
        }
      break;
    // K
    // L
    // M
    // N
    // O
    // P
    // Q
    // R
    case 'S': // control watch constant screen lightup
      send_ok = false;
      g_u32ReturnWatchAck = 10;
      skip_spc();
      yes_no = get_int();
      buf[0] = yes_no;
      buf[1] = 0;
      ProtocolSendMessage(DevTypeWatch, OpcodeSetScreenConstantOn, buf, 2);
      break;
    case 'T': // dump latest timestampd
      send_ok = false;
      g_u32ReturnWatchAck = 10;
      ProtocolSendMessage(DevTypeWatch, OpcodeDumpLatestTimestamp, buf, 2);
      break;
    // U
    case 'V': OperateVibrator();    break; // 
    case 'W': SendPeronalData(3);   break; // USER WEIGHT
    // X
    case 'Y': SendPeronalData(0);   break; // USER YOB
    // Z
    }
  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void process_K_commands(void)
  {
  char chr;
  u32 index, value;
  skip_spc();
  chr = toupper(get_char());
  skip_spc();
  value = get_int();
  switch (chr) // special character affecting Disco?
    {
    case 'E': break;
    case 'D': break;
    case 'C': break;
    case 'T': break;
    case 'R': break;
    case 'A': break;
    case 'S': break;
    case 'G': break;
    case '0': break;
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void process_B_commands(void)
  {
  char chr;
  u32 index, value;
  g_u32ReturnWatchAck = 10;
  send_ok = false;
  skip_spc();
  chr = toupper(get_char());
  skip_spc();
  value = get_int();
  switch (chr) // special character affecting Disco?
    {
    case 'B': g_bCheckIncomingEvents = value != 0; break;
    case 'X': 
      g_u32EventUploadCounter = 0; 
      g_u32EventRepeat = 0;
      g_u32EventUploadType = get_int();
      break;
    case 'J': g_bTesterMode = true;        break;
    case 'K': g_bTesterMode = false;       break;
    }
  index = (u32)chr + 101 - 'A';
  send_service_param(index, value);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
#define PREV_COMMANDS 9
char previous_commands[PREV_COMMANDS][80];

u32 list_previous_commands(void)
  {
  u32 index;
  char buf[22];
  char chr;
  send_message("\r\nCommands:\r\n");
  for (index = 0; index < PREV_COMMANDS; index++)
    {
    sprintf(buf, "\r\n%d: ", index+1);
    send_message(buf);
    send_message(previous_commands[index]);
    }
  send_message("\r\nPress selection ? ");
  flush_rx_buffer(2); // clear buffer
  while (get_comm_buffer_status(2) == 0);
  chr = get_usart_byte(2);
  if (isdigit(chr))
    {
    if (chr == '0')
      {
      goto exit;
      }
    send_message("\r\n");
    chr -= '0';
    if (previous_commands[(u32)chr-1][0] == 0)
      {
      return 0;
      }
    send_message(previous_commands[(u32)chr-1]);
    return (u32)(chr);
    }
exit:
  send_message("\r\nAbort\r\n");
  return 0;  
  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void add_command_to_history(void)
  {
  for (u32 index = 0; index < PREV_COMMANDS - 1; index++)
    {
    strcpy(previous_commands[index], previous_commands[index+1]);
    }
  strcpy(previous_commands[PREV_COMMANDS-1], command);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void process_command(void)
  {
  u32 data, blocks, prm, index, value;
  static u32 address;
  send_ok = true;
  char buf[200], chr;
  comm_ptr = 2;
  switch (toupper(command[1]))
    {
    case 'A': // age - year of birth
      send_year_of_birth();
      break;
    case 'H': // help
      display_help();
      break;
      
    case 'Y': // list simulation directorY
      list_sim_entries();
      break;

    case 'W': // write new service parameter in watch
      send_ok = false;
      prm = get_int();
      data = get_int();
      send_service_param(prm, data);
      g_u32ReturnWatchAck = 10;
      break;
      
    case 'R': // reset simulator
      data = get_int();
      NVIC_SystemReset();
      break;

    case 'F': // first simulation block
      data = get_int();
      first_simulation_index = simulation_index = data;
      break;

    case 'T': // start simulation
      if (flash_last_sim_address == 0)
        {
        send_message("\r\nNo simulation samples\r\n");
        break;
        }
      simulation_flag = true;
      latest_received_page = 0;
      skip_spc();
      if (isdigit(peek_char()))
        {
        get_simulation_parameters();
        }
      simulation_index = first_simulation_index;
      simulation_base = 0;
      if (one_shot_simulation)
        {
        one_shot_simulation = 1; // restart one shot sim
        }
      send_service_param(13, 1);
      break;

    case 'O': // one shot simulation
      one_shot_simulation = 1;
      break;
      
    case 'M': // multi-pass simulation
      one_shot_simulation = 0;
      break;
      
    case 'L': // last simulation block
      data = get_int();
      last_simulation_index = simulation_index = data;
      simulation_flag = true;
      break;
      
    case 'P': // stop simulation
stop_simulation:
      simulation_flag = false;
      send_service_param(13, 0);
      delay_ms(100);
      simulation_runLen = 0;
      NVIC_SystemReset();
      break;
      
    case '*': // set watch ID
      g_u32ReturnWatchAck = 10;
      send_ok = false;
      SetWatchID();
      break;
      
    case '-':
      delay_between_chars = get_int();
      break;
      
    case 'S': // send/set commands
      send_extra_commands(); 
      break;
       
    case 'D': // BIT commands
      send_ok = false;
      more_BIT_commands();
      break;

    case 'C': // continue simulation download
      if (g_u32TestFlashErasedCtr)
        {
        send_message("\r\nFlash is not ready\r\n");
        break;
        }
      receive_bin_simulation_data();
      send_ok = false;
      break;
      
    case 'E': // erase external flash
      FlashBulkErase();
      send_message("\r\nPlease wait up to 3 minutes for bulk erase...\r\n");
      g_u32TestFlashErasedCtr = 150;
      flash_last_sim_address = 0;
      flash_storage_address = 0;
      memset((void *)&sim_directory[0], 0xFF, sizeof(sim_directory));
      break;

    case 'G':
      g_u32ReturnWatchAck = 10;
      send_ok = false;
      skip_spc();
      index = get_int(); // get sim dir entry
      StartStopRecord(index & 1);
      break;
      
    case 'U':
      skip_spc();
      if (peek_char() == '@')
        {
        get_char(); // skip @
        index = get_int(); // get sim dir entry
        u32 entries = dir_entries();
        if (index >= entries)
          {
          break;
          }
        address = sim_directory[index].first_block * 128;
        blocks = sim_directory[index].last_block - sim_directory[index].first_block;
        }
      else
        {
        address = get_int() * 128;
        blocks = get_int();
        }
      dump_sim_data(address, blocks);
      break;
      
    case 'N': // show next block
      address += 128;
      dump_sim_data(address, 1);
      break;
      
    case '?': // query flash erase finish
      dump_sim_data(0, 1);
      break;
      
    case 'Z': // return size of simulation data in samples
      send_simulator_status();
      break;

    case 'X': // erase watch flash memory
      g_u32ReturnWatchAck = 10;
      send_ok = false;
      erase_watch_memory();
      break;
      
    case 'I': // return watch id
      send_mcu_id();
      send_ok = false;
      PC_uploader = true;
      break;
    
    case 'J': // extra commands
      MoreExtraCommands();
      break;  
      
    case 'Q': // query data
      send_ok = false;
      comm_ptr = 2;
      skip_spc();
      chr = toupper(get_char());
      handle_tester_commands(chr);
      send_ok = false;
      PC_uploader = true;
      break;
      
    case 'B': 
      process_B_commands();
      break;
      
    case '&':
      BombWatchWithChars();
      break;
      
    case 'K': // QA commands
      process_K_commands();
      break;
      
    default:
      send_ok = false;
      send_message("\r\nWhat?\r\n");
      break;
    } 
  if (send_ok && !g_u32ReturnWatchAck)
    {
    send_ack_to_pc();
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 get_hex(void)
  {
  u32 num = 0;
  char chr;
  skip_spc();
  if (peek_char())
    {
    num = 0;
    while (isxdigit(peek_char()))
      {
      chr = get_char();
      chr = toupper(chr);
      if (chr <= '9')
        {
        chr -= '0';
        }
      else
        {
        chr = chr - ('A' - 10);
        }
      num = num * 16 + (u32)chr;
      }
    }
  return num;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 get_bin(void)
  {
  u32 num = 0;
  char chr;
  skip_spc();
  if (peek_char())
    {
    num = 0;
    while (isdigit(peek_char()))
      {
      chr = get_char();
      if (chr > '1')
        {
        break;
        }
      chr -= '0';
      num = num * 2 + (u32)chr;
      }
    }
  return num;
  }

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
u32 uploader_message_length, uploaderCX;
u8 uploader_opcode, uploader_command[32];

struct {
u16 u16HdrPreamble;
u16 u16HdrInfoLen;
u16 u16HdrInfoOpc;
u16 u16HdrInfoCrc;
u16 u16HdrCheckSum;
} uploader_comm_header;

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void parse_uploader_command(void)
  {
  switch (uploader_command[0])
    {
    case 12:
      sys_date_time.year     = uploader_command[1];
      sys_date_time.month    = uploader_command[2];
      sys_date_time.day      = uploader_command[3];
      sys_date_time.hour     = uploader_command[4];
      sys_date_time.minute   = uploader_command[5];
      sys_date_time.second   = uploader_command[6];
      RTC_store_date_time(&sys_date_time);
      SetWatchDateTime();
      break;
    default:
      __NOP();
      break;
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void parse_command_date_time(void)
  {
  sys_date_time.year     = get_int();
  sys_date_time.month    = get_int();
  sys_date_time.day      = get_int();
  sys_date_time.hour     = get_int();
  sys_date_time.minute   = get_int();
  sys_date_time.second   = get_int();
  RTC_store_date_time(&sys_date_time);
  SetWatchDateTime();
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
u32 gt_PageUploadNumber[256];
u32 gt_PageUploadNumberX;

void parse_page_request(void)
  {
  u32 page;
  static u8 buf[300];
//  TimestampUpload(1);
  page = make32(uploader_command[11], uploader_command[10], uploader_command[9], uploader_command[8]);
  current_page_request = page;
  send_to_pc = true;
  
  gt_PageUploadNumber[gt_PageUploadNumberX] = page;
  if (++gt_PageUploadNumberX >= 256)
    {
    gt_PageUploadNumberX = 0;
    }
  
//  if (page == 0)
//    {
//    GtNtimestampX = 0;
//    }
  gt_PageUploadRequests++;
  if (page == 0)
    {
    gt_PageUploadRequests = 0;
    gt_u32T0 = HAL_GetTick();
    }
  else
    {
    gt_u32TotalUploadTime = HAL_GetTick() - gt_u32T0;
    }
//  TimestampUpload(2);
  SendWatchDataRequest(page, 1);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void parse_event_elements(u8 *data)
  {
  char buf[99];
  u32 ctrs[6];
  memcpy((u8 *)&ctrs[0], data, 24);
  sprintf(buf, "\r\nEvent Ptrs/Ctrs %d,%d,%d  %d,%d,%d\r\n",
          ctrs[0], ctrs[1], ctrs[2], ctrs[3], ctrs[4], ctrs[5]);
  send_message(buf);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void send_recorded_page_to_uploader(Message *msg)
  {
  u32 addr, idx;
  u16 crc, checksum, received_chk;

//  TimestampUpload(2);

  addr = make32(msg->data[3], msg->data[2], msg->data[1], msg->data[0]);
  latest_page_number = addr;
  addr *= 256;

  memcpy(upload_page+16, &msg->data[8], 256);
  memcpy(latest_upload_page, &msg->data[8], 256);

  for (idx = 0, checksum = 0; idx < 254; idx++)
    {
    checksum += latest_upload_page[idx];
    }
  received_chk = make16(latest_upload_page[255], latest_upload_page[254]);
  if (received_chk != checksum)
    {
    UploadChecksumErrorsCtr++;
    }
  
  upload_page[0] = 0xAB;
  upload_page[1] = 0xCD;
  upload_page[2] = 0xCC;
  upload_page[3] = 0xFF;
  upload_page[4] = 274 - 256;
  upload_page[5] = 1;
  upload_page[6] = 5;
  upload_page[7] = 0;
  
  upload_page[ 8] = make8(addr,0);
  upload_page[ 9] = make8(addr,1);
  upload_page[10] = make8(addr,2);
  upload_page[11] = make8(addr,3);

  upload_page[12] = make8(watch_data_length,0);
  upload_page[13] = make8(watch_data_length,1);
  upload_page[14] = make8(watch_data_length,2);
  upload_page[15] = make8(watch_data_length,3);
  
  crc = CalculateCRC(upload_page, 272, 0xFFFF);
  upload_page[272] = crc & 255;
  upload_page[273] = crc >> 8;
  
  send_block(274, upload_page);

  if (GetTmrFlag(TMR_100MS_FAST_BLINK))
    {
    ClearTmrFlag(TMR_100MS_FAST_BLINK);
    output_toggle(LED1);
    }
  pages_sent_to_uploader++;

//  TimestampUpload(3);
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void store_bin_simulation_data(u32 flash_addr, u8 *payload, u32 len)
  {
  static u32 ppg_errors[16];
  static SIM_BLOCK_DATA simdat[16];
  static SIM_BLOCK_DATA rsimdat[16];
  memcpy((u8 *)simdat, payload, 256);
  for (u32 idx = 0; idx < 8; idx++)
    {
    if (simdat[idx].ppg < -1000)
      {
      ppg_errors[idx]++;
      }
    }
  
  V12_FlashWritePage(flash_storage_address, 256, (u8 *)&simdat);
  delay_ms(7);
  V12_FlashReadData(flash_storage_address, 256, (u8 *)&rsimdat);
  if (memcmp((u8 *)&simdat, (u8 *)&rsimdat, 256) != 0)
    {
    __NOP();
    }
  flash_storage_address += 256;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void parse_bin_sim_data(void)
  {
  static u16 computedCRC, receivedCRC;
  static u32 prev_time = 0, prev_addr = 0xFFFFFFF;
  u32 blockN;
  computedCRC = CalculateCRC(command, SIM_DATA_BLOCK_DOWNLOAD_LEN - 2, 0xFFFF);
  receivedCRC = make16(command[SIM_DATA_BLOCK_DOWNLOAD_LEN-1], command[SIM_DATA_BLOCK_DOWNLOAD_LEN-2]);
  if (computedCRC == receivedCRC)
    {
    if (command[SIM_BLOCK_TYPE] == 100) // end of download - 100%
      {
      end_of_bin_download();
      simulation_data_download = false;
      }
    else if (command[SIM_BLOCK_TYPE] == 1) // data block
      {
      blockN = make32(command[SIM_BLOCK_NUMBER+3], command[SIM_BLOCK_NUMBER+2], command[SIM_BLOCK_NUMBER+1], command[SIM_BLOCK_NUMBER+0]);
      blockN /= 8; // App sends sample number. divide by 8
      if (prev_addr != blockN)
        {
        __disable_interrupt();
        store_bin_simulation_data(blockN * 256, &command[SIM_BLOCK_PAYLOAD], 256);
        __enable_interrupt();
        memcpy((u8 *)&sim_data[0], command, 256);
//        if (sim_data[0].ppg < -1000)
//          {
//          __NOP();
//          }
        u32 timestamp = HAL_GetTick();
//        if (GtNtimestampX < 4090)
//          {
//          GtNtimestamp[++GtNtimestampX] = timestamp - prev_time;
//          }
        prev_time = timestamp;
        }
      else
        {
        same_counter++;
        }
      prev_addr = blockN;
      delay_ms(2);
      send_downloader_message("GtN\r");
      total_received_bin_blocks++;
      }
    }
  else
    {
    send_downloader_message("GtS\r");
    GtS_retries[0]++; // CRC error
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
//void pass_through(void)
//  {
//  static u8 prevChars[10];
//  char chr;
//  while (BypassDiscoMCU)
//    {
//    }
//  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void handle_master(void)
  {
  static u8 prevChars[10];
  char chr;//, r1, r2;
  static u32 command_timeout = 0, data_index;
  char buf[80];

//  event_Stack_depth();

  while (get_comm_buffer_status(2))
    {
    chr = get_usart_byte(2);
      {
      message_timeout = 0;
      
      switch (cmd_state)
        {
        case 0:
            prevChars[0] = prevChars[1];
            prevChars[1] = prevChars[2];
            prevChars[2] = prevChars[3];
            prevChars[3] = chr;
            if (prevChars[3] == 0xA5 && prevChars[2] == 0x12 && prevChars[1] == 0x34 && prevChars[0] == 0x56)
              {
              data_index = 4;
              command[0] = 0x56;
              command[1] = 0x34;
              command[2] = 0x12;
              command[3] = 0xA5;
              
              cmd_state = 12; // this is a sim data block download 
              }
            else if (prevChars[0] == 0xAB && prevChars[1] == 0xCD && prevChars[2] == 0xEF && prevChars[3] == 0xFF)
              {
              cmd_state = 2; // this is a PC uploader command/parameter
              upload_flag = 1;
              }
            else if (prevChars[2] == 0xCD && prevChars[3] == 0xAB)
              {
              uploaderCX = 0;  // request page upload command
              command_timeout = 20;
              cmd_state = 8;
              upload_flag = 1;
              }
            else if (chr == '$')
              {
              cmd_state = 1; // this is start of text command from terminal
              cmdidx = 0;
              command[cmdidx++] = chr;
              upload_flag = 0;
              }
            else if (chr == '%') // list previous commands
              {
              u32 sel = list_previous_commands();
              if (sel)
                {
                strcpy(command, previous_commands[sel-1]);
                goto execute_command;
                }
              }
            break;
          
        case 1:
          command[cmdidx++] = chr;
          if (chr == 13)
            {
            command[cmdidx] = 0;
            add_command_to_history();
execute_command:            
            cmd_state = 0;
            output_high(LED1);
            process_command();
            }
          else if (chr == 3 || chr == 0x7F) // ^C or Del
            {
            cmd_state = 0;
            cmdidx = 0;
            send_message("\r\n\nBreak\r\n");
            }
          else if (chr == 8) // ^H
            {
            if (cmdidx > 1)
              {
              cmdidx -= 2;
              command[cmdidx] = 0;
              send_message("\r                                                \r");
              strcpy(buf, command);
              send_message(buf);
              }
            }
          break;
          
        case 2: // get uploader command
          uploader_message_length = chr;
          cmd_state++;
          break;
        case 3:
          uploader_opcode = chr;
          uploaderCX = 0;
          cmd_state++;
          break;
          
        case 4:
          if (uploader_message_length)
            {
            uploader_command[uploaderCX++] = chr;
            if (--uploader_message_length == 0)
              {
              if (chr == 0xAA) // command terminator?
                {
                parse_uploader_command();
                }
              else
                {
                pc_command_error++;
                }
              cmd_state = 0;
              }
            }
          break;
          
        case 8: // request page upload command
          uploader_command[uploaderCX] = chr;
          if (++uploaderCX >= 12)
            {
            parse_page_request();
            pc_upload_requests++;
            cmd_state = 0;
            }
          break;
  
        case 12: // simulation data
          command[data_index] = chr;
          simulation_data_download = true;
          request_retries = 0;
          if (++data_index >= SIM_DATA_BLOCK_DOWNLOAD_LEN)
            {
            sim_blocks++;
            parse_bin_sim_data();
            cmd_state = 0;
            }
          break;
          
        default: // illegal state. reset state machine
          cmd_state = 0;
          break;
        }
      }
    }
  
//  if (BypassDiscoMCU)
//    {
//    pass_through();
//    }
  
  if (cmd_state > 0 || simulation_data_download)
    {
    if (GetTmrFlag(TMR_10MS_CMD_TIMEOUT))
      {
      ClearTmrFlag(TMR_10MS_CMD_TIMEOUT);
      u32 timeout = 500;
      if (simulation_data_download)
        {
        timeout = 10;
        }
      if (++message_timeout > timeout)
        {
        if (simulation_data_download) // sim data download timeout?
          {
          if (++request_retries > 50)
            {
            request_retries = 0;
            end_of_bin_download();
            simulation_data_download = false;
            }
          else
            {
            send_downloader_message("GtS\r"); // request data again
            GtS_retries[1]++; // timeout
            message_timeout = 0;
            }
          cmd_state = 0;
          }
        else
          {
          message_timeout = 0;
          send_message("\r\nCommand timeout\r\n\n");
          cmd_state = 0;
          comm_reinit();
          }
        }
      }
    }
  }
