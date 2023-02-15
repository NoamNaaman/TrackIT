/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


#include "stdio.h"
#include "string.h"
#include "ctype.h"
#include "stdlib.h"

#include "setup.h"
#include "main.h"
#include "serial.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern u32 BypassDiscoMCU;


u8 RxBuffer[6][USART_IN_BUF_LEN];   // low level serial input buffer
u32 RxCount[6], RxInpIdx[6], RxOutIdx[6]; // low level input/output pointers and byte counter

u8 RS485_sending = 0;

u32 PC_chars, watch_chars;


//extern u8 log[4096];
//extern u32 logx;

void MX_USART2_UART_Init(void);


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void UART_receive_char(UART_HandleTypeDef *huart, u32 uartN)
  {
  u8 chr;
  if (huart->Instance->SR & UART_FLAG_ORE)
    {
    huart->Instance->SR &= ~UART_FLAG_ORE;
    }
  if (huart->Instance->SR & UART_FLAG_NE)
    {
    huart->Instance->SR &= ~UART_FLAG_NE;
    }
  if (huart->Instance->SR & UART_FLAG_FE)
    {
    huart->Instance->SR &= ~UART_FLAG_FE;
    }
  if (huart->Instance->SR & UART_FLAG_PE)
    {
    huart->Instance->SR &= ~UART_FLAG_PE;
    }

  if (huart->Instance->SR & UART_FLAG_RXNE)
    {
    chr = huart->Instance->DR;
    RxBuffer[uartN][RxInpIdx[uartN]] = chr;
    if (++RxInpIdx[uartN] >= USART_IN_BUF_LEN)
      {
      RxInpIdx[uartN] = 0;
      }
    if (RxCount[uartN] < USART_IN_BUF_LEN)
      {
      RxCount[uartN]++;
      }
    else
      {
      RxOutIdx[uartN] = RxInpIdx[uartN];
      }
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void USART2_receive_char(void)
  {
  static u8 prevChars[8];
  u8 chr;
  u32 status;
  status = USART2->SR;
  if (status & UART_FLAG_RXNE)
    {
    chr = USART2->DR;
    if (BypassDiscoMCU)
      {
      UART5->DR = chr;
      prevChars[0] = prevChars[1];
      prevChars[1] = prevChars[2];
      prevChars[2] = prevChars[3];
      prevChars[3] = prevChars[4];
      prevChars[4] = chr;
      if (chr == '9' && prevChars[3] == 'B' && prevChars[2] == 'Z' && prevChars[1] == 'J' && prevChars[0] == '$')
        {
        bypass_comm(0);
//        bypass_comm(0);
//        return;
        }
      }
    else
      {
      RxBuffer[2][RxInpIdx[2]] = chr;
      if (++RxInpIdx[2] >= USART_IN_BUF_LEN)
        {
        RxInpIdx[2] = 0;
        }
      if (RxCount[2] < USART_IN_BUF_LEN)
        {
        RxCount[2]++;
        }
      else
        {
        RxOutIdx[2] = RxInpIdx[2];
        }
      PC_chars++;
      }
    }
  if (status & (UART_FLAG_ORE|UART_FLAG_NE|UART_FLAG_FE|UART_FLAG_PE))
    {
    USART2->SR &= ~(UART_FLAG_ORE|UART_FLAG_NE|UART_FLAG_FE|UART_FLAG_PE);
    }
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void UART5_receive_char(void)
  {
  u8 chr;
  u32 status;
  status = UART5->SR;
  if (status & UART_FLAG_RXNE)
    {
    chr = UART5->DR;
    RxBuffer[5][RxInpIdx[5]] = chr;
    if (++RxInpIdx[5] >= USART_IN_BUF_LEN)
      {
      RxInpIdx[5] = 0;
      }
    if (RxCount[5] < USART_IN_BUF_LEN)
      {
      RxCount[5]++;
      }
    else
      {
      RxOutIdx[5] = RxInpIdx[5];
      }
    PC_chars++;
    }
  if (status & (UART_FLAG_ORE|UART_FLAG_NE|UART_FLAG_FE|UART_FLAG_PE))
    {
    UART5->SR &= ~(UART_FLAG_ORE|UART_FLAG_NE|UART_FLAG_FE|UART_FLAG_PE);
    }
  }


/*==============================================================
 * function:     get_usart_byte()
 * Description:  returns oldest character in receive buffer
 * Parameters:   NONE
 * Returns:      oldest received character
===============================================================*/
u16 get_usart_byte(u32 channel)
  {
  u16 chr = 0;

  chr = RxBuffer[channel][RxOutIdx[channel]];                                 // get next char
  if (++RxOutIdx[channel] >= USART_IN_BUF_LEN)                       // if output pointer overflows buffer length,
    {
    RxOutIdx[channel] = 0;                                           // then reset to beginning of buffer
    }
  if (RxCount[channel])                                              // if (counter is greater than 0,
    {
    if (--RxCount[channel] == 0)                                              // then decrement
      {
      RxOutIdx[channel] = RxInpIdx[channel];
      }
    }
  return chr;                                               // return character
  }

/*==============================================================
 * function:     send_message()
 * Description:  sends a null terminated string to specified USART
 * Parameters:   message - a pointer to 8 bit character string to be sent
 * Returns:      NONE
===============================================================*/
void send_message(char *str)
  {
  u32 length;
  length = strlen(str);
  HAL_UART_Transmit(&huart2, (u8 *)str, length, 100);
  }

/*==============================================================
 * function:     send_block()
 * Description:  sends a null terminated string to specified USART
 * Parameters:   message - a pointer to a byte block to be sent
 * Returns:      NONE
===============================================================*/
void send_block(u32 length, u8 *message)
  {
//  HAL_UART_Transmit(&huart2, (u8 *)message, length, 100);
  while (length--)
    {
    HAL_UART_Transmit(&huart2, (u8 *)message++, 1, 100);
    delay_us(1);
    }
  }

/*==============================================================
 * function:     get_comm_buffer_status()
 * Description:  returns number of received characters waiting in buffer
 * Parameters:   NONE
 * Returns:      character count for specified channel
===============================================================*/
u32 get_comm_buffer_status(u32 channel)
  {
  return RxCount[channel];
  }


/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void flush_rx_buffer(u32 channel)
  {
  RxCount[channel] = 0;
  RxInpIdx[channel] = 0;
  RxOutIdx[channel] = 0;
  }

/////////////////////////////////////////////////////////////////////
// Name:        
// Description: 
// Parameters:  
// Returns:     NONE
/////////////////////////////////////////////////////////////////////
void comm_reinit(void)
  {
  init_pc_port();
  flush_rx_buffer(2);
   __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  }

