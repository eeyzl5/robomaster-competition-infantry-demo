#ifndef _USARTTT__H
#define _USARTTT__H

#include "stm32f4xx_HAL.h"
#include "stm32f4xx_hal.h"
#define BUFFER_SIZE 50*1024 
extern UART_HandleTypeDef huart2;

extern uint8_t recv_end_flag;
extern uint32_t rx_len;
//byte 1:the smaller motor speed
//byte 2: bit 1 :laser 
//byte 3:servo motor
//byte 4:shooting moter speed
//byte 5:the pitch angle value of 6623 motor that need to be set
//byte 6:6623
//byte 7: CRC
#define RX_SIZE 10

extern uint8_t rx_buffer[RX_SIZE];
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
#define TX_SIZE 10
extern uint8_t tx_buffer[TX_SIZE];
#endif


