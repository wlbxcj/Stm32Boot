//*********************************************************************************/
//* �ļ���  ��usart.h
//* ����    �����ô��ڡ�         
//* ʵ��ƽ̨��EEPW_stm32zet6������
//* ��汾  ��ST3.5.0+uCOS
//*
//* ����    ��wlb  
//**********************************************************************************/	
#ifndef	__USART_H__
#define __USART_H__
#include"stm32f10x_conf.h"
//#include <stdio.h>
//********************************************************************************/

void USART_Configuration(void);	  	
void Usart1_PutChar(u8 ch);				//���͵��ֽ�����
void Usart1_PutStr(u8 *p);				//�����ַ���
void NVIC_Configuration(void);			//���ô����ж�
void DBG_Print(char ucPrintLevel,const char *fmt,...);

//int fputc(int ch, FILE *f);
//void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
#endif
