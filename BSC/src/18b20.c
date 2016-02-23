#include "18b20.h"
#include "delay.h"

/*************************************************************
*	函数名称:DS18B20_GPIOIn
*	功    能:IO输入初始化
*	参    数:无 
*	返 回 值:无
*************************************************************/
void DS18B20_GPIOIn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //浮空输入
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*************************************************************
*	函数名称:DSB20_GPIOOut
*	功    能:IO输出初始化
*	参    数:无 
*	返 回 值:无
*************************************************************/
void DS18B20_GPIOOut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/*************************************************************
*	函数名称:DS18B20_Reset
*	功    能:复位DS18B20
*	参    数:无 
*	返 回 值:无
*************************************************************/
void DS18B20_Reset(void)
{
	DS18B20_GPIOOut();
	DS18B20_DQ_CLR;
	DelayUs(480);
	DS18B20_DQ_SET;
	DelayUs(480);	
}

/*************************************************************
*	函数名称:DS18B20_Init
*	功    能:初始化DS18B20
*	参    数:无 
*	返 回 值:无
*************************************************************/
void DS18B20_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	DS18B20_Reset();
	DS18B20_WriteData(PassRom);		//跳过ROM
	DS18B20_Reset();
}


/*************************************************************
*	函数名称:DS18B20_WriteData
*	功    能:向18B20写入数据
*	参    数:
*			reg:要写入的数据 
*	返 回 值:无
*************************************************************/
void DS18B20_WriteData(u8 reg)
{
	u8	i,temp;
	DS18B20_GPIOOut();
	for(i=0;i<8;i++)
	{	
		temp=reg&0x01;
		DS18B20_DQ_CLR;		//产生写信号
		DelayUs(8);			//15US内		
		if(temp)
			DS18B20_DQ_SET;
		else
			DS18B20_DQ_CLR;
		DelayUs(70);		 //至少60US
		DS18B20_DQ_SET;		//释放总线
		reg>>=1;
			
	} 	
}

/*************************************************************
*	函数名称:DS18B20_ReadData
*	功    能:向18B20写入数据
*	参    数:无 
*	返 回 值:
*			temp:DS18B20返回的温度值
*************************************************************/
u8 DS18B20_ReadData(void)
{
	u8	i,temp;
	
	for(i=0;i<8;i++)
	{
		temp>>=1;
		DS18B20_GPIOOut();
		DS18B20_DQ_CLR;			//拉低总线，产生读信号
		DelayUs(4);
		DS18B20_DQ_SET;		   	//释放总线，准备读数据
		
		DS18B20_GPIOIn();
		DelayUs(14);
		if(DS18B20_DQ_IN==1)	  	//开始读取数据
			temp|=0x80;
		DelayUs(70);			 //至少60US
		DS18B20_GPIOOut();		//准备下一次读取数据
		DS18B20_DQ_SET;					
	}
	return temp; 	
}

/*************************************************************
*	函数名称:DS18B20_Start
*	功    能:为数据转换做好准备
*	参    数:无 
*	返 回 值：
*************************************************************/
void DS18B20_Start(void)
{
	DS18B20_Reset();
	DS18B20_WriteData(PassRom);			//	跳过ROM
	DS18B20_WriteData(GetTemp);
}

/*************************************************************
*	函数名称:DS18B20_GetTemp
*	功    能:获取最终转换数据
*	参    数:无 
*	返 回 值：
*			tem:最终转换结果
*************************************************************/
s16	DS18B20_GetTemp(void)
{
	u8 TL,TH,temp;
	s16 tem;
	DS18B20_Start();
	DelayUs(720);	   //等待转换完成
	DS18B20_Reset();				   
	DS18B20_WriteData(PassRom);
	DS18B20_WriteData(ReadReg);
	TL=DS18B20_ReadData();
	TH=DS18B20_ReadData();
	if(TH>7)			  // 温度为负,取反+1
	{
		temp=0;
		if(TL==0)
		{
			TL=~TL+1;
			TH=~TH+1;
		}	
		else
		{
			TL=~TL+1;
			TH=~TH;
		}	
	}
	else	
		temp=1;		//温度为正
	tem=TH;		  //取高8位
	tem<<=8;
	tem=tem+TL;
	tem=(float)tem*0.0625;	//	12位精度
	if(temp)
		return tem;
	else
		return -tem;		
}

