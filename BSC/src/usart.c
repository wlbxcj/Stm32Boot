#include    "usart.h"
#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <stdarg.h>
//#include    <ctype.h>
//#include    <math.h>
/********************************************************

//�������ƣ�USART_Configuration(void)
//�����������Դ���1���г�ʼ��
//�����������
//���أ���

//˵������ʼ����Ϊ����飬��������ʹ��I/O�ڵĳ�ʼ���ʹ��ڹ��ܵĳ�ʼ����
//����USART����Ҫ�õ����ţ����������ݷ���Ҫ������ΪGPIO_M_IN_FLOATING��������
//��GPIO_Mode_AF_PP������������������ĺ�GPIO��������һ����
********************************************************/
void USART_Configuration(void)
{
	USART_InitTypeDef	USART_InitStructure;
	GPIO_InitTypeDef 	GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	//ʹ�ܴ���1ʱ��		 ����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//ʹ�ܴ���2ʱ��		 ����ʱ��
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//ʹ�ܴ���3ʱ��		 ����ʱ��
	//����USART1 RX ��PA.10��Ϊ��������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//����USART1 RX ��PA.9��Ϊ�����������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//******************************************************************************
	// ����1������ʼ�����岿��,����1����Ϊ38400 �� 8 ��1 ��N  �����жϷ�ʽ
	//******************************************************************************
	USART_InitStructure.USART_BaudRate=115200;						//�趨��������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//��������λ��
    USART_InitStructure.USART_StopBits = USART_StopBits_1; 			//�趨ֹͣλ����	
    USART_InitStructure.USART_Parity = USART_Parity_No; 			//���ü���λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
																	//������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//ʹ�÷��ͺͽ��ն˿�
	USART_Init(USART1,&USART_InitStructure);						//��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  				//ʹ�ܴ���1�����ж�
	USART_Cmd(USART1,ENABLE);										//ʹ�ܴ���1
}

/*****************************************************
//�������ƣ�void NVIC_Configuration(void)
//�����������ж�����	
//�����������
//���أ���
*****************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);				
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	   //��ռ���ȼ�1λ�������ȼ�3λ
	//ʹ�ܴ���1�ж�
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}
/*****************************************************
//�������ƣ�Uart1_PutChar
//��������������1���͵��ֽں���
//���������unsigned char ch	Ҫ���͵�����
//���أ���
*****************************************************/
void Usart1_PutChar(u8 ch)
{	
	USART_SendData(USART1,ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET); 	//�ȴ����ݷ������
}

/*****************************************************
//�������ƣ�Uart1_PutChar
//��������������1���͵��ֽں���
//���������unsigned char ch	Ҫ���͵�����
//���أ���
*****************************************************/
void Usart1_PutStr(u8 *p)
{	
	while(*p!='\0')
	{
		Usart1_PutChar(*p);
		p++;
	}
}

/*
 * ��������fputc
 * ����  ���ض���c�⺯��printf��USART1
 * ����  ����
 * ���  ����
 * ����  ����printf����
 */
/*int fputc(int ch, FILE *f)
{
// ��Printf���ݷ������� /
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}
*/
/*
 * ��������itoa
 * ����  ������������ת�����ַ���
 * ����  ��-radix =10 ��ʾ10���ƣ��������Ϊ0
 *         -value Ҫת����������
 *         -buf ת������ַ���
 *         -radix = 10
 * ���  ����
 * ����  ����
 * ����  ����USART1_printf()����
 */
/*static char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    // This implementation only works for decimal numbers. //
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    // if this is a negative value insert the minus sign. //
    if (value < 0)
    {
        *ptr++ = '-';

        // Make the value positive. //
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    // Null terminate the string. //
    *ptr = 0;

    return string;

} */// NCL_Itoa */

/*
 * ��������USART1_printf
 * ����  ����ʽ�������������C���е�printf��������û���õ�C��
 * ����  ��-USARTx ����ͨ��������ֻ�õ��˴���1����USART1
 *		     -Data   Ҫ���͵����ڵ����ݵ�ָ��
 *			   -...    ��������
 * ���  ����
 * ����  ���� 
 * ����  ���ⲿ����
 *         ����Ӧ��USART1_printf( USART1, "\r\n this is a demo \r\n" );
 *            		 USART1_printf( USART1, "\r\n %d \r\n", i );
 *            		 USART1_printf( USART1, "\r\n %s \r\n", j );
 */
/*void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
  int d;   
  char buf[16];

  va_list ap;
  va_start(ap, Data);

	while ( *Data != 0)     // �ж��Ƿ񵽴��ַ���������
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //���з�
					USART_SendData(USARTx, 0x0a);	
					Data ++;
					break;
				
				default:
					Data ++;
				    break;
			}			 
		}
		else if ( *Data == '%')
		{									  //
			switch ( *++Data )
			{				
				case 's':										  //�ַ���
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;

        case 'd':										//ʮ����
          d = va_arg(ap, int);
          itoa(d, buf, 10);
          for (s = buf; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;
				 default:
						Data++;
				    break;
			}		 
		} // end of else if //
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}*/
/*******************************************************************************
 *????: DBG_Print
 *????: ???????????????
         ????g_ulTaskDebugFlag???????????,
         ????????????32,????OS_LOWEST_PRIO?????????
 *????: ucPrintLevel ????
            fmt ???????
 *????: void
 *? ? ?: void
 * 
 *????:
 *        1. 2010-3-29,by jcb Created
          2. 2010-4-09,by zhj changed,?_vsnprintf??_vsprintf,????????
 ******************************************************************************/
void DBG_Print(char ucPrintLevel,const char *fmt,...)
{
//#if (DEBUG_PRINT_EN)

    #define PRINT_BUF_LEN   1024
    unsigned int g_szDebugPrintBuf[PRINT_BUF_LEN / 4 + 1] = {0};
    char   *pBuf = (char *)g_szDebugPrintBuf;
    int i = 0;
    int  ulLen;
    va_list ap;
    memset(pBuf, 0, sizeof(g_szDebugPrintBuf));
    va_start(ap,fmt);
    //ulLen = _vsnprintf(pBuf, PRINT_BUF_LEN, fmt, ap);
    ulLen = vsnprintf(pBuf, PRINT_BUF_LEN, fmt, ap);    // ????????
    va_end(ap);
    for (i = 0; i < ulLen; i++)
        Usart1_PutChar(pBuf[i]);

    return;
}

/*******************************************************************************
 *????: DBG_PrintBuf
 *????: ?????????????????,??????????
         ????g_ulTaskDebugFlag???????????,
         ????????????32,????OS_LOWEST_PRIO?????????
 *????: ucPrintLevel ????
         char *szPrompt: ??????????
         char *pData: ?????
         uint32 ulLen: ???????
 *????: void
 *? ? ?: void
 * 
 *????:
 *        1. 2010-3-29,by jcb Created
 ******************************************************************************/
#if (DEBUG_PRINT_EN)
void DBG_PrintBuf(uint8 ucPrintLevel, char *szPrompt, char *pData, uint32 ulLen)
{

    uint32 i;
    static char szDbgBuf[64];
    char *ptr = szDbgBuf;

    if ((NULL == pData) || (0 == ulLen))
    {
        return;
    }

    if (ucPrintLevel > g_ucPrintDebugFlag)
    {
        return;
    }

    if (!(g_ulTaskDebugFlag & (1u << CPU_GetCurTaskPrio())))
    {
        return;
    }

    if (NULL != szPrompt)
    {
        DBG_Print(ucPrintLevel, "\r\n%s\r\n", szPrompt);
    }

    for (i = 1; i <= ulLen; i++)
    {
        ptr += snprintf(ptr,sizeof(szDbgBuf), "%02X ", *pData);
        pData++;

        if (0 == i % MAX_DUMP_PER_LINE)
        {
            *ptr = 0;
            DBG_Print(ucPrintLevel, "%s\r\n", szDbgBuf);
            ptr = szDbgBuf;
        }
    }
    if (ulLen % MAX_DUMP_PER_LINE)
    {
        *ptr = 0;
        DBG_Print(ucPrintLevel, "%s\r\n", szDbgBuf);
    }

    return;
}
#endif

/*********************************************************************************
**                            End of File
*********************************************************************************/

