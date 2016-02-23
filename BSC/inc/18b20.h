#ifndef __18B20_h__
#define __18B20_h__

#include "stm32f10x_gpio.h"			

#define	PassRom		0xcc		//����64λROM��ַ��ֱ����18B20���¶ȱ任
#define	GetTemp		0x44		//����18B20�����¶�ת����12λ750ms 9λ93.75ms
#define ReadReg		0xbe		//���ڲ�RAM��9�ֽڵ�����
#define WriteReg	0x4e		//�������ڲ�RAM��3��4�ֽ�д�ϡ������¶���������

#define DS18B20_DQ_SET	GPIO_SetBits(GPIOA,GPIO_Pin_1)		//IO������
#define	DS18B20_DQ_CLR	GPIO_ResetBits(GPIOA,GPIO_Pin_1)	//IO������
#define DS18B20_DQ_IN	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)

void 	DS18B20_GPIOIn(void);
void 	DS18B20_GPIOOut(void);
void 	DS18B20_Reset(void);		 //��λDS18B20
void 	DS18B20_Init(void);	   	
u8 	 	DS18B20_ReadData(void);
void	DS18B20_WriteData(u8 reg);
void	DS18B20_Start(void);		//׼��ת������
s16		DS18B20_GetTemp(void);		//��ȡת������


#endif
