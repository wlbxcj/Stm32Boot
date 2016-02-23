/*******其实只要配置好了FMSC的时序以及其使用模式之后*****************
*		对LCD的控制是非常容易的，只须对相应的地址写入数据就可以了。
*******************************************************************/

#include	"GB1616.H"
#include	"8X16.H"
#include	"lcd_ssd1289.h"
#include	"math.h"

//数据区地址	 
#define Bank1_LCD_Data     ((u32)0x6c000002)	
//寄存器区地址																										 
#define Bank1_LCD_Reg      ((u32)0x6c000000)

unsigned char char_or_word=0;			   //为0则显示字符
unsigned char horizontal_or_vertical=0;	   //为0则水平显示

//LCD写寄存器地址函数	 
void LCD_WR_ADD(u16 index) 
{																
	*(vu16 *)(Bank1_LCD_Reg) = index; 		
}
//LCD写数据函数							
void LCD_WR_DATA(u16 val)		 
{															
	*(vu16 *)(Bank1_LCD_Data) = val;
}	

/*
 * 函数名：LCD_WR_REG
 * 描述  ：读 ILI9325 RAM 数据
 * 输出  ：读取的数据,16bit *         
 */
u16 LCD_RD_data(void)
{
    u16 a = 0;
    a = (*(__IO u16 *) (Bank1_LCD_Data)); 	//Dummy	
    a = *(__IO u16 *) (Bank1_LCD_Data);     //L
    
    return(a);	
}

/********************************************************
//函数名称：Lcd_GPIO_Config
//功能描述：对串口1进行初始化
//输入参数：无
//返回：	无
********************************************************/
void Lcd_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);		
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | 
								  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* 设置 PE.01(RESET) , PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	     PE.14(D11), PE.15(D12) 为复用推挽输出 */
	/* PE3,PE4 用于A19, A20, STM32F103ZE-EK(REV 2.0)必须使能 */
	/* PE5,PE6 用于A19, A20, STM32F103ZE-EK(REV 2.0)必须使能 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
	                      			GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
	                      			GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	  /* 设置 PF.00(A0 (RS))  为复用推挽输出 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	  /* 设置 PG.12(NE4 (LCD/CS)) 为复用推挽输出 - CE3(LCD /CS) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_SetBits(GPIOG, GPIO_Pin_1);
}

/********************************************************
//函数名称：Lcd_FMSC_Config
//功能描述：对FMSC进行初始化
//输入参数：无
//返回：	无
//说明：	配置好FMSC的时序以及其使用模式
********************************************************/
void Lcd_FMSC_Config(void)
{
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  	FSMC_NORSRAMTimingInitTypeDef  FSMC_NORSRAMTimingInitStructure;
	/***********Color LCD configuration***************************************
     LCD configured as follow:
        - Data/Address MUX = Disable
        - Memory Type = SRAM
        - Data Width = 16bit
        - Write Operation = Enable
        - Extended Mode = Enable
        - Asynchronous Wait = Disable 
	*************************************************************************/	
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressSetupTime = 2;
	FSMC_NORSRAMTimingInitStructure.FSMC_AddressHoldTime = 0;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataSetupTime =0X05;
	FSMC_NORSRAMTimingInitStructure.FSMC_BusTurnAroundDuration = 0;
	FSMC_NORSRAMTimingInitStructure.FSMC_CLKDivision = 0;
	FSMC_NORSRAMTimingInitStructure.FSMC_DataLatency = 0;
	FSMC_NORSRAMTimingInitStructure.FSMC_AccessMode = FSMC_AccessMode_B;
  	/*******FSMC Configuration********************************************/
  	/*******SRAM Bank 4***************************************************/
  	/*******FSMC_Bank1_NORSRAM4 configuration*****************************/
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4; //BANK1 的 NE4
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; //关掉地址复用
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM; //GRAM 当成MCU的SRAM处理
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b; //16位总线
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable; //突发模式访问，不是PSRAM所以Disable
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable; //写使能
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &FSMC_NORSRAMTimingInitStructure;
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  	/* - BANK  (of NOR/SRAM Bank 0~3) is enabled */
  	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/**********************************************************
	*	函数名称：Lcd_WR_CMD
	*	功能描述：先写入REG再写入数据
	*	输入参数：无
	*	返回：无
********************************************************/
void LCD_WR_CMD(u16 index, u16 val)
{
	*(vu16 *)(Bank1_LCD_Reg) = index; 
       *(vu16 *)(Bank1_LCD_Data) = val;	
}

/**********************************************************
	*	函数名称：LCD_Set_Ver
	*	功能描述：设置是水平显示还是垂直显示
	*	输入参数：无
	*	返回：无
********************************************************/
void LCD_Set_Ver(unsigned char ver)
{
	switch (ver)
	{
		case 0:	
			{
				horizontal_or_vertical=0; //水平显示
				/*********************************************************************
				*当R01为0x6b3f时RGB从S719到S0，为2b3f时从s0到s719...并且选择BGR模式***
				*********************************************************************/
				LCD_WR_CMD(0x0001,0x2b3f);     //驱动输出控制320*240  0x6B3F和0x2b3f可选
				//LCD_WR_CMD(0x0001,0x6b3f);
			
				/************************************************************************
				 *		  这里设置颜色的格式是16色还是262色，0x6000代表显示16色，
				 *		  0x4000代表显示262色，
				 *		  0x0030代表平行扫描，0x0038代表垂直扫描，详见SSD1289
				 *		  数据手册 寄存器R11 的说明。   
				************************************************************************/
				LCD_WR_CMD(0x0011,0x6030);   	//  16位色 竖屏 从左到右扫描
				//LCD_WR_CMD(0x0011,0x6038);	//16色 横屏显示		从左到右扫描
				/************************************************************************/
			}break;
		case 1:
			{
				horizontal_or_vertical=1; //垂直显示
				LCD_WR_CMD(0x0001,0x6b3f);
				LCD_WR_CMD(0x0011,0x6038);

			}break;
		default:horizontal_or_vertical=0; break; //水平显示
	}
}


/**********************************************************
	*	函数名称：Lcd_Init
	*	功能描述：先写入REG再写入数据
	*	输入参数：无
	*	返回：无
********************************************************/
void LCD_MyInit(u8 ver)
{
	unsigned long n;       
	LCD_Set_Ver(ver);					//0为水平显示

	LCD_WR_CMD(0x0000,0x0001);
    LCD_WR_CMD(0x0000,0x0001);      //打开晶振
	LCD_WR_CMD(0x0003,0x6664);      //0xA8A4//  调整功耗，使用默认值
	LCD_WR_CMD(0x000C,0x0000);      //电源控制   5.1V  	   ，好像没什么用
	LCD_WR_CMD(0x000D,0x080C);      //Vlcd63振幅的放大倍数 ，好像没什么用 
	LCD_WR_CMD(0x000E,0x2B00);      //Vlcd63使用外部参考电压  
	LCD_WR_CMD(0x001E,0x00b0);      //不重新设置LCD的复位电平    b0  
    	
	LCD_WR_CMD(0x0002,0x0600);     //LCD Driving Waveform control
	LCD_WR_CMD(0x0010,0x0000);     //禁止睡眠模式
	
	/*这两个为比较寄存器？？在读写数据时进行比较，不能在外部显示模式下使用***/
	LCD_WR_CMD(0x0005,0x0000);     
	LCD_WR_CMD(0x0006,0x0000);     	
	/************************************************************************/
	LCD_WR_CMD(0x0016,0xEF1C);     	//设置像素，这里使用最高 240（POR）像素,30个（hsync）延迟
	LCD_WR_CMD(0x0017,0x0003);     	//开始时 1个（hsync）延迟，结束时4个延迟
	LCD_WR_CMD(0x0007,0x0233);      //0x0233  //控制是否启动LCD     
	LCD_WR_CMD(0x000B,0x0000);     	//画面、周期控制
	LCD_WR_CMD(0x000F,0x0000);      //扫描开始地址，从G0开始，即原点
	/***********************************************************************/
	/**********这两个寄存器好像是设置显示效果的，如切换效果*****************/
	/**********这里不使用效果，所以全为0，以后可以试试**********************/
	LCD_WR_CMD(0x0041,0x0000);     
	LCD_WR_CMD(0x0042,0x0000);    
	/***********************************************************************/    
	LCD_WR_CMD(0x0048,0x0000); 	//指定开始时的扫描位置，这里默认    
	LCD_WR_CMD(0x0049,0x013F);  //指定结束时的扫描位置，这里默认   
	LCD_WR_CMD(0x004A,0x0000);  //指定二次开始时的扫描位置，这里默认   
	LCD_WR_CMD(0x004B,0x0000);	//指定二次结束时的扫描位置，这里默认  
	        
	LCD_WR_CMD(0x0044,0xEF00);	//指定水平扫描时的地址范围，这里默认   
	LCD_WR_CMD(0x0045,0x0000);  //指定垂直扫描时的地址范围，这里默认   
	LCD_WR_CMD(0x0046,0x013F);	//指定垂直扫描时的地址范围，这里默认
	/*******************************************************************        
	*************这些寄存器在数据手册里说是调整电压输出*****************
	*************好像是调整其灰度值的。这里使用默认值******************/
	LCD_WR_CMD(0x0030,0x0707);  
	LCD_WR_CMD(0x0031,0x0204);  
	LCD_WR_CMD(0x0032,0x0204);	     
	LCD_WR_CMD(0x0033,0x0502);     
	LCD_WR_CMD(0x0034,0x0507);     
	LCD_WR_CMD(0x0035,0x0204);     
	LCD_WR_CMD(0x0036,0x0204);     
	LCD_WR_CMD(0x0037,0x0502);     
	LCD_WR_CMD(0x003A,0x0302);     
	LCD_WR_CMD(0x003B,0x0302);
	/*****************************************************************/        
	LCD_WR_CMD(0x0023,0x0000);     //所有数据都可以写入GDDRAM
	LCD_WR_CMD(0x0024,0x0000);     //所有数据都可以写入GDDRAM  

	//LCD_WR_CMD(0x0025,0x8000);	   	//好像数据手册里没有提到这个寄存器
	
	//GRAM写入数据，白色清屏*/							   
	LCD_WR_ADD(0x0022);			   //准备写数据
	for(n=0;n<320*240;n++)
	{
		LCD_WR_DATA(White);
	}
}	

/*************************************************************
*	函数名称:LCD_SetPosition
*	功    能:定义显示窗体
*	参    数:
*				x0:  窗体中X坐标中较小者
*	 		 	y0:  窗体中Y坐标中较小者 
*	返 回 值:无
*************************************************************/
void LCD_SetPosition(u16 x0,u16 y0)
{
	switch (horizontal_or_vertical)
		{									 
		 	case 0:							 //水平显示的时候
				{									
					LCD_WR_CMD(0x0044,0xef00+x0); 
					LCD_WR_CMD(0x004e,x0);        //设置X方向初始值
		
					/*R45、R46   垂直方向的起、止点*/
					LCD_WR_CMD(0x0045,y0);	  
					LCD_WR_CMD(0x0046,0x13f);
	   				LCD_WR_CMD(0x004f,y0);        //设置y方向初始值		
				}break;
			case 1:							 //横屏显示的时候
				{				
					/*垂直方向，高位为终止位*/				
					LCD_WR_CMD(0x0044,0xef+y0); 
					LCD_WR_CMD(0x004e,y0);        //设置X方向初始值
		
					/*R45、R46   水平方向的起、止点*/
					LCD_WR_CMD(0x0045,x0);	  
					LCD_WR_CMD(0x0046,0x13f);
	   				LCD_WR_CMD(0x004f,x0);        //设置y方向初始值
				}break;
		}	
		LCD_WR_ADD(0x0022);			  //准备写数据	
}

/*************************************************************
*	函数名称:LCD_SetPos
*	功    能:定义显示窗体
*	参    数:
*				x0:  窗体中X坐标中较小者
*	 		 	y0:  窗体中Y坐标中较小者 
*	返 回 值:无
*************************************************************/
void LCD_SetPos(u16 x0,u16 y0)
{
		u16 xx;
		switch (horizontal_or_vertical)
		{									 //水平显示的时候
		 	case 0:
				{					
					if (char_or_word)	
		  				xx=(x0+15)<<8;	//若显示汉字
					else
						xx=(x0+7)<<8;	//若显示字符
					xx=xx+x0;				
					LCD_WR_CMD(0x0044,xx); 
					LCD_WR_CMD(0x004e,x0);        //设置X方向初始值
		
					/*R45、R46   垂直方向的起、止点*/
					LCD_WR_CMD(0x0045,y0);	  
					LCD_WR_CMD(0x0046,y0+15);
	   				LCD_WR_CMD(0x004f,y0);        //设置y方向初始值		
				}break;
			case 1:
				{					
					if (char_or_word)	
		  				xx=x0+15;	//显示汉字
					else
						xx=x0+7;	//若显示字符
					
					/*垂直方向，高位为终止位*/				
					LCD_WR_CMD(0x0044,(y0+15)<<8); 
					LCD_WR_CMD(0x004e,y0);        //设置X方向初始值
		
					/*R45、R46   水平方向的起、止点*/
					LCD_WR_CMD(0x0045,x0);	  
					LCD_WR_CMD(0x0046,xx);
	   				LCD_WR_CMD(0x004f,x0);        //设置y方向初始值
				}break;
		}	
		LCD_WR_ADD(0x0022);			  //准备写数据
}
/*************************************************************
*	函数名称:PutGB1616
*	功    能:在指定位置显示单个汉字
*	参    数:
*				x:  窗体中X坐标中较小者
*	 		 	y:  窗体中Y坐标中较小者
*				C[]：待显示的汉字
*				fColor:字体颜色
*				bColor:背景颜色 
*	返 回 值:无
*************************************************************/
void LCD_PutGB1616(u16 x, u16  y, u8 c[2], u16 fColor,u16 bColor)
{
	unsigned int i,j,k;
	char_or_word=1;		//设置为显示汉字，为0则为显示字符
	LCD_SetPos(x,y);

	for(k=0;k<67;k++) 
	{ //64标示自建汉字库中的个数，循环查询内码
	  if ((codeGB_16[k].Index[0]==c[0])&&(codeGB_16[k].Index[1]==c[1]))	 //找到对应的汉字
    	for(i=0;i<32;i++) 
		{											
		 	unsigned short m=codeGB_16[k].Msk[i];
		  	for(j=0;j<8;j++) 
		  	{
				if((m&0x80)==0x80) 
					LCD_WR_DATA(fColor);		 //开始写入汉字颜色
				else 
					LCD_WR_DATA(bColor);		 //开始写入背景颜色
				m<<=1;
			} 
		}  
	}	
}
/*************************************************************
*	函数名称:LCD_PutString
*	功    能:在指定位置显示多个汉字或字符
*	参    数:
*				x:  窗体中X坐标中较小者
*	 		 	y:  窗体中Y坐标中较小者
*				*s：待显示的汉字或字符
*				fColor:字体颜色
*				bColor:背景颜色 
*	返 回 值:无
*************************************************************/
void LCD_PutString(u16 x, u16 y, u8 *s, u16 fColor, u16 bColor) 
{
	unsigned char l=0;
	while(*s) 
	{
		if( *s < 0x80) 								 //判断是否为字符
		{
			LCD_PutChar(x+l*8,y,*s,fColor,bColor);
			s++;
			l++;
		}
		else
		{
			LCD_PutGB1616(x+l*8,y,(u8 *)s,fColor,bColor);
			s+=2;
			l+=2;
		}
	}
}
/*************************************************************
*	函数名称:LCD_PutChar
*	功    能:在指定位置显示一个字符
*	参    数:
*				x:  窗体中X坐标中较小者
*	 		 	y:  窗体中Y坐标中较小者
*				c：待显示的字符
*				fColor:字体颜色
*				bColor:背景颜色 
*	返 回 值:无
*************************************************************/
void LCD_PutChar(u16 x, u16 y, char c, u16 fColor, u16 bColor) 
{
	LCD_PutChar8x16( x, y, c, fColor, bColor );
}

/*************************************************************
*	函数名称:LCD_PutChar
*	功    能:在指定位置显示一个字符
*	参    数:
*				x:  窗体中X坐标中较小者
*	 		 	y:  窗体中Y坐标中较小者
*				c：待显示的字符
*				fColor:字体颜色
*				bColor:背景颜色 
*	返 回 值:无
*************************************************************/
void LCD_PutChar8x16(u16 x, u16 y, char c, u16 fColor, u16 bColor)
{
	unsigned int i,j;
	char_or_word=0;		//设置为显示为字符，1则为显示汉字	
	LCD_SetPos(x,y);
 	
	for(i=0; i<16;i++) 
	{
		unsigned char m=Font8x16[c*16+i];		  //直接找到字符
		for(j=0;j<8;j++) 
		{
			if((m&0x80)==0x80)
				LCD_WR_DATA(fColor);
			else
				LCD_WR_DATA(bColor);
			m<<=1;
		}
	}
}
/*************************************************************
*	函数名称:LCD_TEST_Picture1
*	功    能:显示图片
*	参    数:
*			*picture1：待显示的图片数组 
*	返 回 值:无
*	说	  明：直接把图片颜色数据烧入FLASH，实在太占空间，最好
*			  使用别的方法，比如放入SD卡，直接读BMP格式。
*			  如果这样使用时，一定要设置好待显示的图片的长宽，
*			  否则显示不出效果。
*************************************************************/
void LCD_TEST_Picture1(unsigned char const *picture1)
{
     unsigned	char i,j;
	 u16 picdata,pixH,pixL;
     if	(horizontal_or_vertical)
	 {
	 	LCD_WR_CMD(0x0044,0xef00);        	//hs		  横屏时
	 	LCD_WR_CMD(0x0046,0x00ef);        	//vs
	 }
	 else
	 {
	 	LCD_WR_CMD(0x0044,0xef00);        	//hs  竖屏时
		LCD_WR_CMD(0x0046,0x013f);       	
	 } 									 	
     
	 LCD_WR_CMD(0x004e,0x0000);        		//h	 	 
	 LCD_WR_CMD(0x045,0x0000);        		//he 
	 LCD_WR_CMD(0x004f,0x0000);        		//v     
     LCD_WR_ADD(0x0022);
     for (i=0;i<240;i++)
		for (j=0;j<180;j++)
		{
			pixH=*picture1++;
			pixL=*picture1++;
			picdata=((pixH<<8)+pixL);
			LCD_WR_DATA(picdata);                              
		}
}

/*************************************************************
*	函数名称:LCD_DrawLine
*	功    能:在指定位置画条长度为Length的线
*	参    数:
*				x:  窗体中X坐标中始点
*	 		 	y:  窗体中Y坐标中始点
*				Length：线的长度,长度大于屏幕时，截去
*				Colour:线的颜色 
*	返 回 值:无
*************************************************************/
void LCD_DrawLine(u16 x,u16 y,u16 Length,u16 Colour) 
{
	u16 i;
	if(horizontal_or_vertical)			 //横屏显示时
	{	
		if(x+Length>320)
			Length=320-Length;
	}
	else
	{
		if(x+Length>240)
			Length=240-Length;
	}
	LCD_SetPosition(x,y);
	for(i=0;i<Length;i++)
		LCD_WR_DATA(Colour);
}

/*************************************************************
*	函数名称:LCD_DrawRec
*	功    能:在指定位置画矩形
*	参    数:
*				x:  窗体中X坐标中始点
*	 		 	y:  窗体中Y坐标中始点
*				Length：条形的长
*				High:	矩形的高
*				Colour:矩形的颜色 
*	返 回 值:无
*************************************************************/
void LCD_DrawRec(u16 x,u16 y,u16 Length,u16 High,u16 Colour)
{
	u16 i;	
	if(horizontal_or_vertical)			 //横屏显示时
	{	
		if(x+Length>320)
			Length=320-Length;
		if(High+y>240)
			High=240-High;
		LCD_SetPosition(x,y);
		LCD_WR_CMD(0X0044,((High+y-1)<<8)+y);
		LCD_WR_CMD(0X0046,x+Length);
	}
	else								//竖屏显示时
	{
		if(x+Length>240)
			Length=240-Length;
		if(High+y>320)
			High=320-High;
		LCD_SetPosition(x,y);
		LCD_WR_CMD(0X0044,((Length+x-1)<<8)+x);
		LCD_WR_CMD(0X0046,y+High);
	}
	LCD_WR_ADD(0x0022);
	for(i=0;i<Length*High;i++)
	{
		LCD_WR_DATA(Colour);
	}	
}

/*************************************************************
*	函数名称:LCD_DrawCir
*	功    能:在指定位置画圆
*	参    数:
*				x:  窗体中圆心坐标
*	 		 	y:  窗体中圆心坐标
*				r： 圆的半径
*				Colour:圆的颜色 
*	返 回 值:无
*************************************************************/
void LCD_DrawCir(u16 x,u16 y,u16 r,u16 Colour)
{
	u16 i,j;	
	if(horizontal_or_vertical)			 //横屏显示时
	{	
		if(x+r>320||r-x>0)
			LCD_PutString(19,190,"ERRO!",Red,Black);
		if(r+y>240||r-y>0)
			LCD_PutString(19,190,"ERRO!",Red,Black);

		LCD_WR_CMD(0x0044,(y+r-1)<<8); 
		LCD_WR_CMD(0x004e,y-r);        //设置X方向初始值
		
		/*R45、R46   垂直方向的起、止点*/
		LCD_WR_CMD(0x0045,x-r);	  
		LCD_WR_CMD(0x0046,x+r-1);
	   	LCD_WR_CMD(0x004f,x-r);        //设置y方向初始值

	}
	else								//竖屏显示时
	{
		if(x+r>240||r-x>0)
			LCD_PutString(19,190,"ERRO!",Red,Black);;
		if(r+y>320||r-y>0)
			LCD_PutString(19,190,"ERRO!",Red,Black);;		
		LCD_WR_CMD(0x0044,((x+r-1)<<8)+x-r); 
		LCD_WR_CMD(0x004e,x-r);        //设置X方向初始值
		
		/*R45、R46   垂直方向的起、止点*/
		LCD_WR_CMD(0x0045,y-r);	  
		LCD_WR_CMD(0x0046,y+r-1);
	   	LCD_WR_CMD(0x004f,y-r);        //设置y方向初始值
	}
	LCD_WR_ADD(0x0022);
	for(i=0;i<2*r;i++)
		for(j=0;j<2*r;j++)
		{
			if((abs(r-i)*abs(r-i)+abs(r-j)*abs(r-j))<=r*r)
				LCD_WR_DATA(Colour);
			else
			{
				
				LCD_WR_DATA(Red);
			}
				
		}		
}

