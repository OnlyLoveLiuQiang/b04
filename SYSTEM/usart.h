#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define UART1_RX_LEN 256
#define UART1_TX_LEN 256
#define UART2_RX_LEN 256
#define UART3_RX_LEN 1539

void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void u_p(char *fmt,...);
void Uart1_SendStr(char*SendBuf);
void Clear_Buffer2(void);
void Clear_Buffer3(void);
static void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch );
void Usart_SendStr_length( USART_TypeDef * pUSARTx, uint8_t *str,uint32_t strlen );
#endif


