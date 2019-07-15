#ifndef _USARTTT__H
#define _USARTTT__H

#include "stm32f4xx_HAL.h"
#include "stm32f4xx_hal.h"
#define BUFFER_SIZE 120*1024 
#define MAX_RECV_LEN 1024 
extern  uint8_t TxData[8];
extern  uint8_t RxData[8];
extern  UART_HandleTypeDef huart2;
extern uint8_t RxData[8];
extern uint8_t recv_end_flag;
extern uint8_t rx_len;
extern uint8_t rx_buffer[BUFFER_SIZE];
extern DMA_HandleTypeDef hdma_usart2_rx;
extern int count;
extern uint8_t msg_buff[MAX_RECV_LEN] ;
extern uint8_t * msg ;
extern int flag;


#define TX_SIZE 10
extern uint8_t tx_buffer1[TX_SIZE];
//byte 1:the smaller motor speed
//byte 2: bit 1 :laser 
//byte 3:servo motor
//byte 4:shooting moter speed
//byte 5:the pitch angle value of 6623 motor that need to be set
//byte 6:
//byte 7: CRC

#define RX_SIZE 10
extern uint8_t rx_buffer1[RX_SIZE];
//byte 1:the smaller motor speed
//byte 2: bit 1 :laser 
//byte 3:6623 motor speed
//byte 4:shooting moter speed
//byte 5:the pitch angle value of 6623 motor that need to be set
//byte 6; current yaw value
//byte 7: current pitch value
//byte 8: current roll value
//byte 9:
//byte 10: CRC


#endif


