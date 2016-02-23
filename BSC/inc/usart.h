//*********************************************************************************/
//* 文件名  ：usart.h
//* 描述    ：配置串口。         
//* 实验平台：EEPW_stm32zet6开发板
//* 库版本  ：ST3.5.0+uCOS
//*
//* 作者    ：wlb  
//**********************************************************************************/	
#ifndef	__USART_H__
#define __USART_H__
#include"stm32f10x_conf.h"
//#include <stdio.h>
//********************************************************************************/

void USART_Configuration(void);	  	
void Usart1_PutChar(u8 ch);				//发送单字节数据
void Usart1_PutStr(u8 *p);				//发送字符串
void NVIC_Configuration(void);			//配置串口中断
void DBG_Print(char ucPrintLevel,const char *fmt,...);

//int fputc(int ch, FILE *f);
//void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
#endif
