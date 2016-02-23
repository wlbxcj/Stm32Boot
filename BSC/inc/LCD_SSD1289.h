#ifndef __LCD_SSD1289_H__
#define	__LCD_SSD1289_H__
#include	"stm32f10x_fsmc.h"

/* LCD color */
#define White          0xFFFF			//白色
#define Black          0x0000			//黑色
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0


void Lcd_GPIO_Config(void);				//IO口初始化
void Lcd_FMSC_Config(void);				//设定FMSC
void LCD_WR_ADD(u16 index);				//写地址
void LCD_WR_DATA(u16 val);				//写数据
u16 LCD_RD_data(void);					//读数据
void LCD_WR_CMD(u16 index, u16 val);	//写数据跟地址
void LCD_Set_Ver(unsigned char ver);	//设置横屏还是竖屏
void LCD_MyInit(u8 ver);				//LCD初始化设置,并设置是竖屏还是横屏

void LCD_SetPosition(u16 x0,u16 y0);	//设置点的位置
void LCD_SetPos(u16 x0,u16 y0);			//设置字体显示位置
void LCD_PutString(u16 x, u16 y, u8 *s, u16 fColor, u16 bColor);  	//在屏幕上显示相应数据
void LCD_PutChar(u16 x, u16 y, char c, u16 fColor, u16 bColor);   	//在屏幕上显示字符数据
void LCD_PutChar8x16(u16 x, u16 y, char c, u16 fColor, u16 bColor);	//在屏幕上显示字符数据
void LCD_PutGB1616(u16 x, u16  y, u8 c[2], u16 fColor,u16 bColor);	//在屏幕上显示汉字

void LCD_TEST_Picture1(unsigned char const *picture1);				//显示图片
void LCD_DrawLine(u16 x,u16 y,u16 Length,u16 Colour);				//画线
void LCD_DrawRec(u16 x,u16 y,u16 Length,u16 High,u16 Colour);		//画矩形
void LCD_DrawCir(u16 x,u16 y,u16 r,u16 Colour);						//画实心圆

#endif
