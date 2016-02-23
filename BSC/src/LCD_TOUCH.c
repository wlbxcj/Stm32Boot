#include "lcd_touch.h"
#include "delay.h"	   

#define CS_SET	GPIO_SetBits(GPIOF,GPIO_Pin_10)
#define CS_CLEAR	GPIO_ResetBits(GPIOF,GPIO_Pin_10)	


/*************************************************************
*	函数名称:Touch_IRQ
*	功    能:触摸中断初始化
*	参    数:无 
*	返 回 值:无
*************************************************************/
void Touch_IRQ(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;				
	//打开复用时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//中断脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	  //低电平中断，上拉输入
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	/***************设置中断量**************************/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource13);	   //中断1
	/***************配置中断脚的模式********************/
	EXTI_InitStructure.EXTI_Line=EXTI_Line13;				//中断1
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;		//采用中断方式
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling; 	//下降延方式
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/****************配置中断优先级**********************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;			 //外部中断1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;	 //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;		 //从占优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;			 //使能
	NVIC_Init(&NVIC_InitStructure);

}

/*************************************************************
*	函数名称:Touch_SPIConfig
*	功    能:初始化SPI
*	参    数:无				
*	 		 	
*	返 回 值:无
*************************************************************/
void Touch_SPIConfig(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//PA7 SPI数据输出 PA5 SPI 时钟信号  					 //输出
  	/* 配置 SPI1 引脚: SCK, MISO 和 MOSI */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);								  
	/***********片选*****************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	/* SPI1配置 */ 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 	   //双向全双工
  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						   //主SPI
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					   //8位帧结构
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							   //时钟悬空低
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						   //补获数据于第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							   //内部NSS信号由SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;	   //波特率预分频值为4
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					   //数据传输从MSB开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	/* 使能SPI1 */
  	SPI_Cmd(SPI1, ENABLE);
}

/*************************************************************
*	函数名称:Touch_Init
*	功    能:初始化触摸屏
*	参    数:无				
*	 		 	
*	返 回 值:无
*************************************************************/
void Touch_Init(void)
{
	Touch_IRQ();
	Touch_SPIConfig();
}

/*************************************************************
*	函数名称:Touch_GetPost
*	功    能:获取X或Y坐标
*	参    数:
*				Byte:TEST_X或TESTY
*	 		 	
*	返 回 值:取得X或Y点的坐标
*************************************************************/
void Touch_WR_CMD(u8 Byte)
{
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET);
	SPI_I2S_SendData(SPI1,Byte);
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET);
	SPI_I2S_ReceiveData(SPI1);	
}

/******************************************************
* 函数名：Read_touch_ad
* 描述  ：通过SPI方式读取 2046所获得的触摸AD转换值
* 输入  : 无
* 输出  ：无
* 举例  ：无
* 注意  ：内部使用
*********************************************************/    
int Read_Touch_AD(void)  
{ 
    unsigned short buf,temp; 
    /*  等待数据寄存器空 */  
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 

    /* 通过SPI1发送数据 */  
    SPI_I2S_SendData(SPI1,0x0000); 

    /* 等待接收到一个字节的数据 */ 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 

    /* 读取SPI1接收的数据 */		 
    temp=SPI_I2S_ReceiveData(SPI1); 

    buf=temp<<8; 
    DelayUs(1);

		/*  等待数据寄存器空 */  
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 

    /* 通过SPI1发送数据 */ 
    SPI_I2S_SendData(SPI1,0x0000); 

    /* 等待接收到一个字节的数据 */ 
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
 
    /* 读取SPI1接收的数据 */ 
    temp=SPI_I2S_ReceiveData(SPI1); 
 
    buf |= temp; 
    buf>>=3; 
    buf&=0xfff; 
    return buf; 

}

/******************************************************
* 函数名：Read_X
* 描述  ：通过SPI方式读取 2046所获得的触摸 通道X+ AD转换值
* 输入  : 无
* 输出  ：X+通道AD转换值
* 举例  ：无
* 注意  ：无
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
* 函数名：Read_Y
* 描述  ：通过SPI方式读取 2046所获得的触摸 通道Y+ AD转换值
* 输入  : 无
* 输出  ：Y+通道AD转换值
* 举例  ：无
* 注意  ：无
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
* 函数名：Touch_GetAdXY
* 描述  ：SPI方式 读取2046 通道X+ 通道Y+的ADC值
* 输入  : 无
* 输出  ：通道X+ 通道Y+的ADC值
* 举例  ：无
* 注意  ：无
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
