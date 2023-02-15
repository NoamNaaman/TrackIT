#ifndef __SERIAL
#define __SERIAL


u32 get_comm_buffer_status(u32 channel);
u16 get_usart_byte(u32 channel);
void send_message(char *message);
void send_RS485_message(char *str);
void flush_rx_buffer(u32 channel);

#define USART_IN_BUF_LEN 2048


extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;


#endif