/*******��ʵֻҪ���ú���FMSC��ʱ���Լ���ʹ��ģʽ֮��*****************
*		��LCD�Ŀ����Ƿǳ����׵ģ�ֻ�����Ӧ�ĵ�ַд�����ݾͿ����ˡ�
*******************************************************************/

#include	"GB1616.H"
#include	"8X16.H"
#include	"lcd_ssd1289.h"
#include	"math.h"

//��������ַ	 
#define Bank1_LCD_Data     ((u32)0x6c000002)	
//�Ĵ�������ַ																										 
#define Bank1_LCD_Reg      ((u32)0x6c000000)

unsigned char char_or_word=0;			   //Ϊ0����ʾ�ַ�
unsigned char horizontal_or_vertical=0;	   //Ϊ0��ˮƽ��ʾ

//LCDд�Ĵ�����ַ����	 
void LCD_WR_ADD(u16 index) 
{																
	*(vu16 *)(Bank1_LCD_Reg) = index; 		
}
//LCDд���ݺ���							
void LCD_WR_DATA(u16 val)		 
{															
	*(vu16 *)(Bank1_LCD_Data) = val;
}	

/*
 * ��������LCD_WR_REG
 * ����  ���� ILI9325 RAM ����
 * ���  ����ȡ������,16bit *         
 */
u16 LCD_RD_data(void)
{
    u16 a = 0;
    a = (*(__IO u16 *) (Bank1_LCD_Data)); 	//Dummy	
    a = *(__IO u16 *) (Bank1_LCD_Data);     //L
    
    return(a);	
}

/********************************************************
//�������ƣ�Lcd_GPIO_Config
//�����������Դ���1���г�ʼ��
//�����������
//���أ�	��
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
	
	/* ���� PE.01(RESET) , PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	     PE.14(D11), PE.15(D12) Ϊ����������� */
	/* PE3,PE4 ����A19, A20, STM32F103ZE-EK(REV 2.0)����ʹ�� */
	/* PE5,PE6 ����A19, A20, STM32F103ZE-EK(REV 2.0)����ʹ�� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
	                      			GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
	                      			GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	  /* ���� PF.00(A0 (RS))  Ϊ����������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	  /* ���� PG.12(NE4 (LCD/CS)) Ϊ����������� - CE3(LCD /CS) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);
	GPIO_SetBits(GPIOG, GPIO_Pin_1);
}

/********************************************************
//�������ƣ�Lcd_FMSC_Config
//������������FMSC���г�ʼ��
//�����������
//���أ�	��
//˵����	���ú�FMSC��ʱ���Լ���ʹ��ģʽ
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
	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4; //BANK1 �� NE4
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; //�ص���ַ����
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM; //GRAM ����MCU��SRAM����
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b; //16λ����
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable; //ͻ��ģʽ���ʣ�����PSRAM����Disable
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable; //дʹ��
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
	*	�������ƣ�Lcd_WR_CMD
	*	������������д��REG��д������
	*	�����������
	*	���أ���
********************************************************/
void LCD_WR_CMD(u16 index, u16 val)
{
	*(vu16 *)(Bank1_LCD_Reg) = index; 
       *(vu16 *)(Bank1_LCD_Data) = val;	
}

/**********************************************************
	*	�������ƣ�LCD_Set_Ver
	*	����������������ˮƽ��ʾ���Ǵ�ֱ��ʾ
	*	�����������
	*	���أ���
********************************************************/
void LCD_Set_Ver(unsigned char ver)
{
	switch (ver)
	{
		case 0:	
			{
				horizontal_or_vertical=0; //ˮƽ��ʾ
				/*********************************************************************
				*��R01Ϊ0x6b3fʱRGB��S719��S0��Ϊ2b3fʱ��s0��s719...����ѡ��BGRģʽ***
				*********************************************************************/
				LCD_WR_CMD(0x0001,0x2b3f);     //�����������320*240  0x6B3F��0x2b3f��ѡ
				//LCD_WR_CMD(0x0001,0x6b3f);
			
				/************************************************************************
				 *		  ����������ɫ�ĸ�ʽ��16ɫ����262ɫ��0x6000������ʾ16ɫ��
				 *		  0x4000������ʾ262ɫ��
				 *		  0x0030����ƽ��ɨ�裬0x0038����ֱɨ�裬���SSD1289
				 *		  �����ֲ� �Ĵ���R11 ��˵����   
				************************************************************************/
				LCD_WR_CMD(0x0011,0x6030);   	//  16λɫ ���� ������ɨ��
				//LCD_WR_CMD(0x0011,0x6038);	//16ɫ ������ʾ		������ɨ��
				/************************************************************************/
			}break;
		case 1:
			{
				horizontal_or_vertical=1; //��ֱ��ʾ
				LCD_WR_CMD(0x0001,0x6b3f);
				LCD_WR_CMD(0x0011,0x6038);

			}break;
		default:horizontal_or_vertical=0; break; //ˮƽ��ʾ
	}
}


/**********************************************************
	*	�������ƣ�Lcd_Init
	*	������������д��REG��д������
	*	�����������
	*	���أ���
********************************************************/
void LCD_MyInit(u8 ver)
{
	unsigned long n;       
	LCD_Set_Ver(ver);					//0Ϊˮƽ��ʾ

	LCD_WR_CMD(0x0000,0x0001);
    LCD_WR_CMD(0x0000,0x0001);      //�򿪾���
	LCD_WR_CMD(0x0003,0x6664);      //0xA8A4//  �������ģ�ʹ��Ĭ��ֵ
	LCD_WR_CMD(0x000C,0x0000);      //��Դ����   5.1V  	   ������ûʲô��
	LCD_WR_CMD(0x000D,0x080C);      //Vlcd63����ķŴ��� ������ûʲô�� 
	LCD_WR_CMD(0x000E,0x2B00);      //Vlcd63ʹ���ⲿ�ο���ѹ  
	LCD_WR_CMD(0x001E,0x00b0);      //����������LCD�ĸ�λ��ƽ    b0  
    	
	LCD_WR_CMD(0x0002,0x0600);     //LCD Driving Waveform control
	LCD_WR_CMD(0x0010,0x0000);     //��ֹ˯��ģʽ
	
	/*������Ϊ�ȽϼĴ��������ڶ�д����ʱ���бȽϣ��������ⲿ��ʾģʽ��ʹ��***/
	LCD_WR_CMD(0x0005,0x0000);     
	LCD_WR_CMD(0x0006,0x0000);     	
	/************************************************************************/
	LCD_WR_CMD(0x0016,0xEF1C);     	//�������أ�����ʹ����� 240��POR������,30����hsync���ӳ�
	LCD_WR_CMD(0x0017,0x0003);     	//��ʼʱ 1����hsync���ӳ٣�����ʱ4���ӳ�
	LCD_WR_CMD(0x0007,0x0233);      //0x0233  //�����Ƿ�����LCD     
	LCD_WR_CMD(0x000B,0x0000);     	//���桢���ڿ���
	LCD_WR_CMD(0x000F,0x0000);      //ɨ�迪ʼ��ַ����G0��ʼ����ԭ��
	/***********************************************************************/
	/**********�������Ĵ���������������ʾЧ���ģ����л�Ч��*****************/
	/**********���ﲻʹ��Ч��������ȫΪ0���Ժ��������**********************/
	LCD_WR_CMD(0x0041,0x0000);     
	LCD_WR_CMD(0x0042,0x0000);    
	/***********************************************************************/    
	LCD_WR_CMD(0x0048,0x0000); 	//ָ����ʼʱ��ɨ��λ�ã�����Ĭ��    
	LCD_WR_CMD(0x0049,0x013F);  //ָ������ʱ��ɨ��λ�ã�����Ĭ��   
	LCD_WR_CMD(0x004A,0x0000);  //ָ�����ο�ʼʱ��ɨ��λ�ã�����Ĭ��   
	LCD_WR_CMD(0x004B,0x0000);	//ָ�����ν���ʱ��ɨ��λ�ã�����Ĭ��  
	        
	LCD_WR_CMD(0x0044,0xEF00);	//ָ��ˮƽɨ��ʱ�ĵ�ַ��Χ������Ĭ��   
	LCD_WR_CMD(0x0045,0x0000);  //ָ����ֱɨ��ʱ�ĵ�ַ��Χ������Ĭ��   
	LCD_WR_CMD(0x0046,0x013F);	//ָ����ֱɨ��ʱ�ĵ�ַ��Χ������Ĭ��
	/*******************************************************************        
	*************��Щ�Ĵ����������ֲ���˵�ǵ�����ѹ���*****************
	*************�����ǵ�����Ҷ�ֵ�ġ�����ʹ��Ĭ��ֵ******************/
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
	LCD_WR_CMD(0x0023,0x0000);     //�������ݶ�����д��GDDRAM
	LCD_WR_CMD(0x0024,0x0000);     //�������ݶ�����д��GDDRAM  

	//LCD_WR_CMD(0x0025,0x8000);	   	//���������ֲ���û���ᵽ����Ĵ���
	
	//GRAMд�����ݣ���ɫ����*/							   
	LCD_WR_ADD(0x0022);			   //׼��д����
	for(n=0;n<320*240;n++)
	{
		LCD_WR_DATA(White);
	}
}	

/*************************************************************
*	��������:LCD_SetPosition
*	��    ��:������ʾ����
*	��    ��:
*				x0:  ������X�����н�С��
*	 		 	y0:  ������Y�����н�С�� 
*	�� �� ֵ:��
*************************************************************/
void LCD_SetPosition(u16 x0,u16 y0)
{
	switch (horizontal_or_vertical)
		{									 
		 	case 0:							 //ˮƽ��ʾ��ʱ��
				{									
					LCD_WR_CMD(0x0044,0xef00+x0); 
					LCD_WR_CMD(0x004e,x0);        //����X�����ʼֵ
		
					/*R45��R46   ��ֱ�������ֹ��*/
					LCD_WR_CMD(0x0045,y0);	  
					LCD_WR_CMD(0x0046,0x13f);
	   				LCD_WR_CMD(0x004f,y0);        //����y�����ʼֵ		
				}break;
			case 1:							 //������ʾ��ʱ��
				{				
					/*��ֱ���򣬸�λΪ��ֹλ*/				
					LCD_WR_CMD(0x0044,0xef+y0); 
					LCD_WR_CMD(0x004e,y0);        //����X�����ʼֵ
		
					/*R45��R46   ˮƽ�������ֹ��*/
					LCD_WR_CMD(0x0045,x0);	  
					LCD_WR_CMD(0x0046,0x13f);
	   				LCD_WR_CMD(0x004f,x0);        //����y�����ʼֵ
				}break;
		}	
		LCD_WR_ADD(0x0022);			  //׼��д����	
}

/*************************************************************
*	��������:LCD_SetPos
*	��    ��:������ʾ����
*	��    ��:
*				x0:  ������X�����н�С��
*	 		 	y0:  ������Y�����н�С�� 
*	�� �� ֵ:��
*************************************************************/
void LCD_SetPos(u16 x0,u16 y0)
{
		u16 xx;
		switch (horizontal_or_vertical)
		{									 //ˮƽ��ʾ��ʱ��
		 	case 0:
				{					
					if (char_or_word)	
		  				xx=(x0+15)<<8;	//����ʾ����
					else
						xx=(x0+7)<<8;	//����ʾ�ַ�
					xx=xx+x0;				
					LCD_WR_CMD(0x0044,xx); 
					LCD_WR_CMD(0x004e,x0);        //����X�����ʼֵ
		
					/*R45��R46   ��ֱ�������ֹ��*/
					LCD_WR_CMD(0x0045,y0);	  
					LCD_WR_CMD(0x0046,y0+15);
	   				LCD_WR_CMD(0x004f,y0);        //����y�����ʼֵ		
				}break;
			case 1:
				{					
					if (char_or_word)	
		  				xx=x0+15;	//��ʾ����
					else
						xx=x0+7;	//����ʾ�ַ�
					
					/*��ֱ���򣬸�λΪ��ֹλ*/				
					LCD_WR_CMD(0x0044,(y0+15)<<8); 
					LCD_WR_CMD(0x004e,y0);        //����X�����ʼֵ
		
					/*R45��R46   ˮƽ�������ֹ��*/
					LCD_WR_CMD(0x0045,x0);	  
					LCD_WR_CMD(0x0046,xx);
	   				LCD_WR_CMD(0x004f,x0);        //����y�����ʼֵ
				}break;
		}	
		LCD_WR_ADD(0x0022);			  //׼��д����
}
/*************************************************************
*	��������:PutGB1616
*	��    ��:��ָ��λ����ʾ��������
*	��    ��:
*				x:  ������X�����н�С��
*	 		 	y:  ������Y�����н�С��
*				C[]������ʾ�ĺ���
*				fColor:������ɫ
*				bColor:������ɫ 
*	�� �� ֵ:��
*************************************************************/
void LCD_PutGB1616(u16 x, u16  y, u8 c[2], u16 fColor,u16 bColor)
{
	unsigned int i,j,k;
	char_or_word=1;		//����Ϊ��ʾ���֣�Ϊ0��Ϊ��ʾ�ַ�
	LCD_SetPos(x,y);

	for(k=0;k<67;k++) 
	{ //64��ʾ�Խ����ֿ��еĸ�����ѭ����ѯ����
	  if ((codeGB_16[k].Index[0]==c[0])&&(codeGB_16[k].Index[1]==c[1]))	 //�ҵ���Ӧ�ĺ���
    	for(i=0;i<32;i++) 
		{											
		 	unsigned short m=codeGB_16[k].Msk[i];
		  	for(j=0;j<8;j++) 
		  	{
				if((m&0x80)==0x80) 
					LCD_WR_DATA(fColor);		 //��ʼд�뺺����ɫ
				else 
					LCD_WR_DATA(bColor);		 //��ʼд�뱳����ɫ
				m<<=1;
			} 
		}  
	}	
}
/*************************************************************
*	��������:LCD_PutString
*	��    ��:��ָ��λ����ʾ������ֻ��ַ�
*	��    ��:
*				x:  ������X�����н�С��
*	 		 	y:  ������Y�����н�С��
*				*s������ʾ�ĺ��ֻ��ַ�
*				fColor:������ɫ
*				bColor:������ɫ 
*	�� �� ֵ:��
*************************************************************/
void LCD_PutString(u16 x, u16 y, u8 *s, u16 fColor, u16 bColor) 
{
	unsigned char l=0;
	while(*s) 
	{
		if( *s < 0x80) 								 //�ж��Ƿ�Ϊ�ַ�
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
*	��������:LCD_PutChar
*	��    ��:��ָ��λ����ʾһ���ַ�
*	��    ��:
*				x:  ������X�����н�С��
*	 		 	y:  ������Y�����н�С��
*				c������ʾ���ַ�
*				fColor:������ɫ
*				bColor:������ɫ 
*	�� �� ֵ:��
*************************************************************/
void LCD_PutChar(u16 x, u16 y, char c, u16 fColor, u16 bColor) 
{
	LCD_PutChar8x16( x, y, c, fColor, bColor );
}

/*************************************************************
*	��������:LCD_PutChar
*	��    ��:��ָ��λ����ʾһ���ַ�
*	��    ��:
*				x:  ������X�����н�С��
*	 		 	y:  ������Y�����н�С��
*				c������ʾ���ַ�
*				fColor:������ɫ
*				bColor:������ɫ 
*	�� �� ֵ:��
*************************************************************/
void LCD_PutChar8x16(u16 x, u16 y, char c, u16 fColor, u16 bColor)
{
	unsigned int i,j;
	char_or_word=0;		//����Ϊ��ʾΪ�ַ���1��Ϊ��ʾ����	
	LCD_SetPos(x,y);
 	
	for(i=0; i<16;i++) 
	{
		unsigned char m=Font8x16[c*16+i];		  //ֱ���ҵ��ַ�
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
*	��������:LCD_TEST_Picture1
*	��    ��:��ʾͼƬ
*	��    ��:
*			*picture1������ʾ��ͼƬ���� 
*	�� �� ֵ:��
*	˵	  ����ֱ�Ӱ�ͼƬ��ɫ��������FLASH��ʵ��̫ռ�ռ䣬���
*			  ʹ�ñ�ķ������������SD����ֱ�Ӷ�BMP��ʽ��
*			  �������ʹ��ʱ��һ��Ҫ���úô���ʾ��ͼƬ�ĳ���
*			  ������ʾ����Ч����
*************************************************************/
void LCD_TEST_Picture1(unsigned char const *picture1)
{
     unsigned	char i,j;
	 u16 picdata,pixH,pixL;
     if	(horizontal_or_vertical)
	 {
	 	LCD_WR_CMD(0x0044,0xef00);        	//hs		  ����ʱ
	 	LCD_WR_CMD(0x0046,0x00ef);        	//vs
	 }
	 else
	 {
	 	LCD_WR_CMD(0x0044,0xef00);        	//hs  ����ʱ
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
*	��������:LCD_DrawLine
*	��    ��:��ָ��λ�û�������ΪLength����
*	��    ��:
*				x:  ������X������ʼ��
*	 		 	y:  ������Y������ʼ��
*				Length���ߵĳ���,���ȴ�����Ļʱ����ȥ
*				Colour:�ߵ���ɫ 
*	�� �� ֵ:��
*************************************************************/
void LCD_DrawLine(u16 x,u16 y,u16 Length,u16 Colour) 
{
	u16 i;
	if(horizontal_or_vertical)			 //������ʾʱ
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
*	��������:LCD_DrawRec
*	��    ��:��ָ��λ�û�����
*	��    ��:
*				x:  ������X������ʼ��
*	 		 	y:  ������Y������ʼ��
*				Length�����εĳ�
*				High:	���εĸ�
*				Colour:���ε���ɫ 
*	�� �� ֵ:��
*************************************************************/
void LCD_DrawRec(u16 x,u16 y,u16 Length,u16 High,u16 Colour)
{
	u16 i;	
	if(horizontal_or_vertical)			 //������ʾʱ
	{	
		if(x+Length>320)
			Length=320-Length;
		if(High+y>240)
			High=240-High;
		LCD_SetPosition(x,y);
		LCD_WR_CMD(0X0044,((High+y-1)<<8)+y);
		LCD_WR_CMD(0X0046,x+Length);
	}
	else								//������ʾʱ
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
*	��������:LCD_DrawCir
*	��    ��:��ָ��λ�û�Բ
*	��    ��:
*				x:  ������Բ������
*	 		 	y:  ������Բ������
*				r�� Բ�İ뾶
*				Colour:Բ����ɫ 
*	�� �� ֵ:��
*************************************************************/
void LCD_DrawCir(u16 x,u16 y,u16 r,u16 Colour)
{
	u16 i,j;	
	if(horizontal_or_vertical)			 //������ʾʱ
	{	
		if(x+r>320||r-x>0)
			LCD_PutString(19,190,"ERRO!",Red,Black);
		if(r+y>240||r-y>0)
			LCD_PutString(19,190,"ERRO!",Red,Black);

		LCD_WR_CMD(0x0044,(y+r-1)<<8); 
		LCD_WR_CMD(0x004e,y-r);        //����X�����ʼֵ
		
		/*R45��R46   ��ֱ�������ֹ��*/
		LCD_WR_CMD(0x0045,x-r);	  
		LCD_WR_CMD(0x0046,x+r-1);
	   	LCD_WR_CMD(0x004f,x-r);        //����y�����ʼֵ

	}
	else								//������ʾʱ
	{
		if(x+r>240||r-x>0)
			LCD_PutString(19,190,"ERRO!",Red,Black);;
		if(r+y>320||r-y>0)
			LCD_PutString(19,190,"ERRO!",Red,Black);;		
		LCD_WR_CMD(0x0044,((x+r-1)<<8)+x-r); 
		LCD_WR_CMD(0x004e,x-r);        //����X�����ʼֵ
		
		/*R45��R46   ��ֱ�������ֹ��*/
		LCD_WR_CMD(0x0045,y-r);	  
		LCD_WR_CMD(0x0046,y+r-1);
	   	LCD_WR_CMD(0x004f,y-r);        //����y�����ʼֵ
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

