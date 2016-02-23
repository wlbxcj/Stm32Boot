#include "18b20.h"
#include "delay.h"

/*************************************************************
*	��������:DS18B20_GPIOIn
*	��    ��:IO�����ʼ��
*	��    ��:�� 
*	�� �� ֵ:��
*************************************************************/
void DS18B20_GPIOIn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	  //��������
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*************************************************************
*	��������:DSB20_GPIOOut
*	��    ��:IO�����ʼ��
*	��    ��:�� 
*	�� �� ֵ:��
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
*	��������:DS18B20_Reset
*	��    ��:��λDS18B20
*	��    ��:�� 
*	�� �� ֵ:��
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
*	��������:DS18B20_Init
*	��    ��:��ʼ��DS18B20
*	��    ��:�� 
*	�� �� ֵ:��
*************************************************************/
void DS18B20_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	DS18B20_Reset();
	DS18B20_WriteData(PassRom);		//����ROM
	DS18B20_Reset();
}


/*************************************************************
*	��������:DS18B20_WriteData
*	��    ��:��18B20д������
*	��    ��:
*			reg:Ҫд������� 
*	�� �� ֵ:��
*************************************************************/
void DS18B20_WriteData(u8 reg)
{
	u8	i,temp;
	DS18B20_GPIOOut();
	for(i=0;i<8;i++)
	{	
		temp=reg&0x01;
		DS18B20_DQ_CLR;		//����д�ź�
		DelayUs(8);			//15US��		
		if(temp)
			DS18B20_DQ_SET;
		else
			DS18B20_DQ_CLR;
		DelayUs(70);		 //����60US
		DS18B20_DQ_SET;		//�ͷ�����
		reg>>=1;
			
	} 	
}

/*************************************************************
*	��������:DS18B20_ReadData
*	��    ��:��18B20д������
*	��    ��:�� 
*	�� �� ֵ:
*			temp:DS18B20���ص��¶�ֵ
*************************************************************/
u8 DS18B20_ReadData(void)
{
	u8	i,temp;
	
	for(i=0;i<8;i++)
	{
		temp>>=1;
		DS18B20_GPIOOut();
		DS18B20_DQ_CLR;			//�������ߣ��������ź�
		DelayUs(4);
		DS18B20_DQ_SET;		   	//�ͷ����ߣ�׼��������
		
		DS18B20_GPIOIn();
		DelayUs(14);
		if(DS18B20_DQ_IN==1)	  	//��ʼ��ȡ����
			temp|=0x80;
		DelayUs(70);			 //����60US
		DS18B20_GPIOOut();		//׼����һ�ζ�ȡ����
		DS18B20_DQ_SET;					
	}
	return temp; 	
}

/*************************************************************
*	��������:DS18B20_Start
*	��    ��:Ϊ����ת������׼��
*	��    ��:�� 
*	�� �� ֵ��
*************************************************************/
void DS18B20_Start(void)
{
	DS18B20_Reset();
	DS18B20_WriteData(PassRom);			//	����ROM
	DS18B20_WriteData(GetTemp);
}

/*************************************************************
*	��������:DS18B20_GetTemp
*	��    ��:��ȡ����ת������
*	��    ��:�� 
*	�� �� ֵ��
*			tem:����ת�����
*************************************************************/
s16	DS18B20_GetTemp(void)
{
	u8 TL,TH,temp;
	s16 tem;
	DS18B20_Start();
	DelayUs(720);	   //�ȴ�ת�����
	DS18B20_Reset();				   
	DS18B20_WriteData(PassRom);
	DS18B20_WriteData(ReadReg);
	TL=DS18B20_ReadData();
	TH=DS18B20_ReadData();
	if(TH>7)			  // �¶�Ϊ��,ȡ��+1
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
		temp=1;		//�¶�Ϊ��
	tem=TH;		  //ȡ��8λ
	tem<<=8;
	tem=tem+TL;
	tem=(float)tem*0.0625;	//	12λ����
	if(temp)
		return tem;
	else
		return -tem;		
}

