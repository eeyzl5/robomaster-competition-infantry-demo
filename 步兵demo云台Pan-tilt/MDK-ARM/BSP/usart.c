#include "usart.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"

 uint8_t recv_end_flag;
 uint32_t rx_len;
uint8_t tx_buffer[TX_SIZE]={0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0x7F};
uint8_t rx_buffer[RX_SIZE]={0x7F,0x7F,0x7F,0xFE,0,0x7F,0x7F,0x7F,0x7F,0x7F};
extern DMA_HandleTypeDef hdma_usart2_rx;
extern int count;

int flag = 0;
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       

//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
	USART6->DR = (uint8_t) ch;      
	return ch;
}
#endif

void USART2_IRQHandler(void)
{

       

        uint32_t tmp_flag = 0;
        uint32_t temp;
        tmp_flag =  __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); 
        if((tmp_flag != RESET))
       { 
                __HAL_UART_CLEAR_IDLEFLAG(&huart2);
                temp = huart2.Instance->SR;  
                temp = huart2.Instance->DR; 
                HAL_UART_DMAStop(&huart2);
                temp  = hdma_usart2_rx.Instance->NDTR;             
                rx_len =  BUFFER_SIZE - temp;                            
                 recv_end_flag = 1;
         }


}


