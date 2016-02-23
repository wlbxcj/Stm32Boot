#include    "usart.h"
#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <stdarg.h>
//#include    <ctype.h>
//#include    <math.h>
/********************************************************

//函数名称：USART_Configuration(void)
//功能描述：对串口1进行初始化
//输入参数：无
//返回：无

//说明：初始化分为两大块，即串口所使用I/O口的初始化和串口功能的初始化。
//对于USART串口要用的引脚，根据其数据方向，要设置其为GPIO_M_IN_FLOATING浮空输入
//或GPIO_Mode_AF_PP复用推挽输出，其他的和GPIO引脚设置一样。
********************************************************/
void USART_Configuration(void)
{
	USART_InitTypeDef	USART_InitStructure;
	GPIO_InitTypeDef 	GPIO_InitStructure;	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	//使能串口1时钟		 高速时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//使能串口2时钟		 低速时钟
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//使能串口3时钟		 低速时钟
	//配置USART1 RX （PA.10）为浮空输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//配置USART1 RX （PA.9）为复用推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//******************************************************************************
	// 串口1参数初始化定义部分,串口1参数为38400 ， 8 ，1 ，N  接收中断方式
	//******************************************************************************
	USART_InitStructure.USART_BaudRate=115200;						//设定传输速率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 	//传输数据位数
    USART_InitStructure.USART_StopBits = USART_StopBits_1; 			//设定停止位个数	
    USART_InitStructure.USART_Parity = USART_Parity_No; 			//不用检验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
																	//不用流量控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//使用发送和接收端口
	USART_Init(USART1,&USART_InitStructure);						//初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  				//使能串口1接收中断
	USART_Cmd(USART1,ENABLE);										//使能串口1
}

/*****************************************************
//函数名称：void NVIC_Configuration(void)
//功能描述：中断配置	
//输入参数：无
//返回：无
*****************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef	NVIC_InitStructure;
	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);				
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);	   //先占优先级1位，从优先级3位
	//使能串口1中断
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}
/*****************************************************
//函数名称：Uart1_PutChar
//功能描述：串口1发送单字节函数
//输入参数：unsigned char ch	要发送的数据
//返回：无
*****************************************************/
void Usart1_PutChar(u8 ch)
{	
	USART_SendData(USART1,ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET); 	//等待数据发送完毕
}

/*****************************************************
//函数名称：Uart1_PutChar
//功能描述：串口1发送单字节函数
//输入参数：unsigned char ch	要发送的数据
//返回：无
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
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
/*int fputc(int ch, FILE *f)
{
// 将Printf内容发往串口 /
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}
*/
/*
 * 函数名：itoa
 * 描述  ：将整形数据转换成字符串
 * 输入  ：-radix =10 表示10进制，其他结果为0
 *         -value 要转换的整形数
 *         -buf 转换后的字符串
 *         -radix = 10
 * 输出  ：无
 * 返回  ：无
 * 调用  ：被USART1_printf()调用
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
 * 函数名：USART1_printf
 * 描述  ：格式化输出，类似于C库中的printf，但这里没有用到C库
 * 输入  ：-USARTx 串口通道，这里只用到了串口1，即USART1
 *		     -Data   要发送到串口的内容的指针
 *			   -...    其他参数
 * 输出  ：无
 * 返回  ：无 
 * 调用  ：外部调用
 *         典型应用USART1_printf( USART1, "\r\n this is a demo \r\n" );
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

	while ( *Data != 0)     // 判断是否到达字符串结束符
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //换行符
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
				case 's':										  //字符串
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;

        case 'd':										//十进制
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

