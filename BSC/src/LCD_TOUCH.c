#include "lcd_touch.h"
#include "delay.h"	   

#define CS_SET	GPIO_SetBits(GPIOF,GPIO_Pin_10)
#define CS_CLEAR	GPIO_ResetBits(GPIOF,GPIO_Pin_10)	


/*************************************************************
*	��������:Touch_IRQ
*	��    ��:�����жϳ�ʼ��
*	��    ��:�� 
*	�� �� ֵ:��
*************************************************************/
void Touch_IRQ(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;				
	//�򿪸���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//�жϽ�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	  //�͵�ƽ�жϣ���������
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	/***************�����ж���**************************/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource13);	   //�ж�1
	/***************�����жϽŵ�ģʽ********************/
	EXTI_InitStructure.EXTI_Line=EXTI_Line13;				//�ж�1
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;		//�����жϷ�ʽ
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling; 	//�½��ӷ�ʽ
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/****************�����ж����ȼ�**********************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;			 //�ⲿ�ж�1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;	 //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;		 //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;			 //ʹ��
	NVIC_Init(&NVIC_InitStructure);

}

/*************************************************************
*	��������:Touch_SPIConfig
*	��    ��:��ʼ��SPI
*	��    ��:��				
*	 		 	
*	�� �� ֵ:��
*************************************************************/
void Touch_SPIConfig(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//PA7 SPI������� PA5 SPI ʱ���ź�  					 //���
  	/* ���� SPI1 ����: SCK, MISO �� MOSI */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);								  
	/***********Ƭѡ*****************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	/* SPI1���� */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 	   //˫��ȫ˫��
  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //��SPI
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //8λ֡�ṹ
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							   //ʱ�����յ�
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						   //���������ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;	   //������Ԥ��ƵֵΪ4
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					   //���ݴ����MSB��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	/* ʹ��SPI1 */
  	SPI_Cmd(SPI1, ENABLE);
}

/*************************************************************
*	��������:Touch_Init
*	��    ��:��ʼ��������
*	��    ��:��				
*	 		 	
*	�� �� ֵ:��
*************************************************************/
void Touch_Init(void)
{
	Touch_IRQ();
	Touch_SPIConfig();
}

/*************************************************************
*	��������:Touch_GetPost
*	��    ��:��ȡX��Y����
*	��    ��:
*				Byte:TEST_X��TESTY
*	 		 	
*	�� �� ֵ:ȡ��X��Y�������
*************************************************************/
void Touch_WR_CMD(u8 Byte)
{
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPI1,Byte);
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET);
	SPI_I2S_ReceiveData(SPI1);	
}

/******************************************************
* ��������Read_touch_ad
* ����  ��ͨ��SPI��ʽ��ȡ 2046����õĴ���ADת��ֵ
* ����  : ��
* ���  ����
* ����  ����
* ע��  ���ڲ�ʹ��
*********************************************************/    
int Read_Touch_AD(void)  
{ 
    unsigned short buf,temp; 
    /*  �ȴ����ݼĴ����� */  
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 

    /* ͨ��SPI1�������� */  
    SPI_I2S_SendData(SPI1,0x0000); 

    /* �ȴ����յ�һ���ֽڵ����� */ 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 

    /* ��ȡSPI1���յ����� */		 
    temp=SPI_I2S_ReceiveData(SPI1); 

    buf=temp<<8; 
    DelayUs(1);

		/*  �ȴ����ݼĴ����� */  
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 

    /* ͨ��SPI1�������� */ 
    SPI_I2S_SendData(SPI1,0x0000); 

    /* �ȴ����յ�һ���ֽڵ����� */ 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
 
    /* ��ȡSPI1���յ����� */ 
    temp=SPI_I2S_ReceiveData(SPI1); 
 
    buf |= temp; 
    buf>>=3; 
    buf&=0xfff; 
    return buf; 

}

/******************************************************
* ��������Read_X
* ����  ��ͨ��SPI��ʽ��ȡ 2046����õĴ��� ͨ��X+ ADת��ֵ
* ����  : ��
* ���  ��X+ͨ��ADת��ֵ
* ����  ����
* ע��  ����
*********************************************************/    
int Read_X(void)  
{  
    int i; 
    CS_CLEAR; 
    DelayUs(1); 
    Touch_WR_CMD(TEST_X); 
    DelayUs(1); 
    i=Read_Touch_AD(); 
    CS_SET; 
    return i;    
} 

/******************************************************
* ��������Read_Y
* ����  ��ͨ��SPI��ʽ��ȡ 2046����õĴ��� ͨ��Y+ ADת��ֵ
* ����  : ��
* ���  ��Y+ͨ��ADת��ֵ
* ����  ����
* ע��  ����
*********************************************************/    
int Read_Y(void)  
{  
    int i; 
    CS_CLEAR; 
    DelayUs(1); 
    Touch_WR_CMD(TEST_Y); 
    DelayUs(1); 
    i=Read_Touch_AD(); 
    CS_SET; 
    return i;     
}

/******************************************************
* ��������Touch_GetAdXY
* ����  ��SPI��ʽ ��ȡ2046 ͨ��X+ ͨ��Y+��ADCֵ
* ����  : ��
* ���  ��ͨ��X+ ͨ��Y+��ADCֵ
* ����  ����
* ע��  ����
*********************************************************/    
void Touch_GetAdXY(int *x,int *y)  
{ 
    int adx,ady; 
    adx=Read_X(); 
    DelayUs(1); 
    ady=Read_Y(); 
    *x=adx; 
    *y=ady; 
}
