#ifndef __LCD_TOUCH_H__
#define __LCD_TOUCH_H__
#include "stm32f10x.h"

/************�������꣬12λ�ֱ��ʣ��͹���********************/
#define  TEST_Y	0X90		//����Y����
#define	 TEST_X	0xD0		//����X����	


void Touch_IRQ(void);
void Touch_SPIConfig(void);
void Touch_Init(void);
void Touch_WR_CMD(u8 Byte);
//void DelayUs(vu32 cnt);
int Read_Touch_AD(void);
int Read_X(void);
int Read_Y(void);
#endif
