#ifndef __18B20_h__
#define __18B20_h__

#include "stm32f10x_gpio.h"			

#define	PassRom		0xcc		//忽略64位ROM地址，直接向18B20发温度变换
#define	GetTemp		0x44		//启动18B20进行温度转换，12位750ms 9位93.75ms
#define ReadReg		0xbe		//读内部RAM中9字节的内容
#define WriteReg	0x4e		//发出向内部RAM的3、4字节写上、下限温度数据命令

#define DS18B20_DQ_SET	GPIO_SetBits(GPIOA,GPIO_Pin_1)		//IO口拉高
#define	DS18B20_DQ_CLR	GPIO_ResetBits(GPIOA,GPIO_Pin_1)	//IO口拉低
#define DS18B20_DQ_IN	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)

void 	DS18B20_GPIOIn(void);
void 	DS18B20_GPIOOut(void);
void 	DS18B20_Reset(void);		 //复位DS18B20
void 	DS18B20_Init(void);	   	
u8 	 	DS18B20_ReadData(void);
void	DS18B20_WriteData(u8 reg);
void	DS18B20_Start(void);		//准备转换数据
s16		DS18B20_GetTemp(void);		//获取转换数据


#endif
