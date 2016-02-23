#ifndef __LCD_SSD1289_H__
#define	__LCD_SSD1289_H__
#include	"stm32f10x_fsmc.h"

/* LCD color */
#define White          0xFFFF			//��ɫ
#define Black          0x0000			//��ɫ
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0


void Lcd_GPIO_Config(void);				//IO�ڳ�ʼ��
void Lcd_FMSC_Config(void);				//�趨FMSC
void LCD_WR_ADD(u16 index);				//д��ַ
void LCD_WR_DATA(u16 val);				//д����
u16 LCD_RD_data(void);					//������
void LCD_WR_CMD(u16 index, u16 val);	//д���ݸ���ַ
void LCD_Set_Ver(unsigned char ver);	//���ú�����������
void LCD_MyInit(u8 ver);				//LCD��ʼ������,���������������Ǻ���

void LCD_SetPosition(u16 x0,u16 y0);	//���õ��λ��
void LCD_SetPos(u16 x0,u16 y0);			//����������ʾλ��
void LCD_PutString(u16 x, u16 y, u8 *s, u16 fColor, u16 bColor);  	//����Ļ����ʾ��Ӧ����
void LCD_PutChar(u16 x, u16 y, char c, u16 fColor, u16 bColor);   	//����Ļ����ʾ�ַ�����
void LCD_PutChar8x16(u16 x, u16 y, char c, u16 fColor, u16 bColor);	//����Ļ����ʾ�ַ�����
void LCD_PutGB1616(u16 x, u16  y, u8 c[2], u16 fColor,u16 bColor);	//����Ļ����ʾ����

void LCD_TEST_Picture1(unsigned char const *picture1);				//��ʾͼƬ
void LCD_DrawLine(u16 x,u16 y,u16 Length,u16 Colour);				//����
void LCD_DrawRec(u16 x,u16 y,u16 Length,u16 High,u16 Colour);		//������
void LCD_DrawCir(u16 x,u16 y,u16 r,u16 Colour);						//��ʵ��Բ

#endif
