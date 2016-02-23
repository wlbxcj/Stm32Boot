#include "sdio.h"
#include "usart.h"

//#define BSize            512 /* Block Size in Bytes */
//#define BWordsSize      (BSize >> 2)
//#define NOfBlocks       2  /* For Multi Blocks operation (Read/Write) */
//#define MWordsSize ((BSize * NBlocks) >> 2)



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define NULL 0
#define SDIO_STATIC_FLAGS               ((uint32_t)0x000005FF)
#define SDIO_CMD0TIMEOUT                ((uint32_t)0x00002710)
#define SDIO_FIFO_Address               ((uint32_t)0x40018080)

/* Mask for errors Card Status R1 (OCR Register) */
#define SD_OCR_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000)
#define SD_OCR_ADDR_MISALIGNED          ((uint32_t)0x40000000)
#define SD_OCR_BLOCK_LEN_ERR            ((uint32_t)0x20000000)
#define SD_OCR_ERASE_SEQ_ERR            ((uint32_t)0x10000000)
#define SD_OCR_BAD_ERASE_PARAM          ((uint32_t)0x08000000)
#define SD_OCR_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000)
#define SD_OCR_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000)
#define SD_OCR_COM_CRC_FAILED           ((uint32_t)0x00800000)
#define SD_OCR_ILLEGAL_CMD              ((uint32_t)0x00400000)
#define SD_OCR_CARD_ECC_FAILED          ((uint32_t)0x00200000)
#define SD_OCR_CC_ERROR                 ((uint32_t)0x00100000)
#define SD_OCR_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000)
#define SD_OCR_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000)
#define SD_OCR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000)
#define SD_OCR_CID_CSD_OVERWRIETE       ((uint32_t)0x00010000)
#define SD_OCR_WP_ERASE_SKIP            ((uint32_t)0x00008000)
#define SD_OCR_CARD_ECC_DISABLED        ((uint32_t)0x00004000)
#define SD_OCR_ERASE_RESET              ((uint32_t)0x00002000)
#define SD_OCR_AKE_SEQ_ERROR            ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS                ((uint32_t)0xFDFFE008)

/* Masks for R6 Response */
#define SD_R6_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00002000)
#define SD_R6_ILLEGAL_CMD               ((uint32_t)0x00004000)
#define SD_R6_COM_CRC_FAILED            ((uint32_t)0x00008000)

#define SD_VOLTAGE_WINDOW_SD            ((uint32_t)0x80100000)
#define SD_HIGH_CAPACITY                ((uint32_t)0x40000000)
#define SD_STD_CAPACITY                 ((uint32_t)0x00000000)
#define SD_CHECK_PATTERN                ((uint32_t)0x000001AA)

#define SD_MAX_VOLT_TRIAL               ((uint32_t)0x0000FFFF)
#define SD_ALLZERO                      ((uint32_t)0x00000000)

#define SD_WIDE_BUS_SUPPORT             ((uint32_t)0x00040000)
#define SD_SINGLE_BUS_SUPPORT           ((uint32_t)0x00010000)
#define SD_CARD_LOCKED                  ((uint32_t)0x02000000)
#define SD_CARD_PROGRAMMING             ((uint32_t)0x00000007)
#define SD_CARD_RECEIVING               ((uint32_t)0x00000006)
#define SD_DATATIMEOUT                  ((uint32_t)0x000FFFFF)
#define SD_0TO7BITS                     ((uint32_t)0x000000FF)
#define SD_8TO15BITS                    ((uint32_t)0x0000FF00)
#define SD_16TO23BITS                   ((uint32_t)0x00FF0000)
#define SD_24TO31BITS                   ((uint32_t)0xFF000000)
#define SD_MAX_DATA_LENGTH              ((uint32_t)0x01FFFFFF)

#define SD_HALFFIFO                     ((uint32_t)0x00000008)
#define SD_HALFFIFOBYTES                ((uint32_t)0x00000020)

/* Command Class Supported */
#define SD_CCCC_LOCK_UNLOCK             ((uint32_t)0x00000080)
#define SD_CCCC_WRITE_PROT              ((uint32_t)0x00000040)
#define SD_CCCC_ERASE                   ((uint32_t)0x00000020)

/* Following commands are SD Card Specific commands.
   SDIO_APP_CMD should be sent before sending these commands. */
#define SDIO_SEND_IF_COND               ((uint32_t)0x00000008)

#define SDIO_INIT_CLK_DIV                  ((uint8_t)0xB2)
#define SDIO_TRANSFER_CLK_DIV              ((uint8_t)0x1) 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t CardType =  SDIO_STD_CAPACITY_SD_CARD_V1_1;
static uint32_t CSD_Tab[4], CID_Tab[4], RCA = 0;
static uint32_t DeviceMode = SD_POLLING_MODE;
static uint32_t TotalNumberOfBytes = 0, StopCondition = 0;
uint32_t *SrcBuffer, *DestBuffer;
volatile SD_Error TransferError = SD_OK;
__IO uint32_t TransferEnd = 0;
__IO uint32_t NumberOfBytes = 0;
SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;

/* Private function prototypes -----------------------------------------------*/
static SD_Error CmdError(void);
static SD_Error CmdResp1Error(uint8_t cmd);
static SD_Error CmdResp7Error(void);
static SD_Error CmdResp3Error(void);
static SD_Error CmdResp2Error(void);
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca);
static SD_Error SDEnWideBus(FunctionalState NewState);
static SD_Error IsCardProgramming(uint8_t *pstatus);
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr);
static uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes);
static void GPIO_Configuration(void);
static void DMA_TxConfiguration(uint32_t *BufferSRC, uint32_t BufferSize);
static void DMA_RxConfiguration(uint32_t *BufferDST, uint32_t BufferSize);

/* Private functions ---------------------------------------------------------*/

/*
 * 函数名：SD_Init
 * 描述  ：初始化SD卡，使卡处于就绪状态(准备传输数据)
 * 输入  ：无
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：外部调用
 */
SD_Error SD_Init(void)
{
  SD_Error errorstatus = SD_OK;

  /* Configure SDIO interface GPIO */
  GPIO_Configuration();

  /* Enable the SDIO AHB Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, ENABLE);

  /* Enable the DMA2 Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

  SDIO_DeInit();

  errorstatus = SD_PowerON();

  if (errorstatus != SD_OK)
  {
    /* CMD Response TimeOut (wait for CMDSENT flag) */
    return(errorstatus);
  }

  errorstatus = SD_InitializeCards();

  if (errorstatus != SD_OK)
  {
    /* CMD Response TimeOut (wait for CMDSENT flag) */
    return(errorstatus);
  }

  /* Configure the SDIO peripheral */
  /* HCLK = 72 MHz, SDIOCLK = 72 MHz, SDIO_CK = HCLK/(2 + 1) = 24 MHz */  
  SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
  SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
  SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
  SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
  SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
  SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
  SDIO_Init(&SDIO_InitStructure);

  return(errorstatus);
}

/*
 * 函数名：SD_PowerON
 * 描述  ：确保SD卡的工作电压和配置控制时钟
 * 输入  ：无
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：在 SD_Init() 调用
 */
SD_Error SD_PowerON(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t response = 0, count = 0;
  bool validvoltage = FALSE;
  uint32_t SDType = SD_STD_CAPACITY;

  /* Power ON Sequence -------------------------------------------------------*/
  /* Configure the SDIO peripheral */
  SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV; /* HCLK = 72MHz, SDIOCLK = 72MHz, SDIO_CK = HCLK/(178 + 2) = 400 KHz */
  SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising ;
  SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
  SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
  SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
  SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
  SDIO_Init(&SDIO_InitStructure);

  /* Set Power State to ON */
  SDIO_SetPowerState(SDIO_PowerState_ON);

  /* Enable SDIO Clock */
  SDIO_ClockCmd(ENABLE);

  /* CMD0: GO_IDLE_STATE -------------------------------------------------------*/
  /* No CMD response required */
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_GO_IDLE_STATE;	   //cmd0
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No;	   //无响应
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;		   //CPSM在开始发送数据之前等待数据传输结束
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdError();	   			//	检测是否正确接收到CMD0

  if (errorstatus != SD_OK)
  {
    /* CMD Response TimeOut (wait for CMDSENT flag) */
    return(errorstatus);
  }

  /* CMD8: SEND_IF_COND --------------------------------------------------------*/
  /* Send CMD8 to verify SD card interface operating condition */
  /* Argument: - [31:12]: Reserved (shall be set to '0')
               - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
               - [7:0]: Check Pattern (recommended 0xAA) */
  /* CMD Response: R7 */
  SDIO_CmdInitStructure.SDIO_Argument = SD_CHECK_PATTERN;	  //接收到命令SD会返回这个参数
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_IF_COND;	  //CMD8
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r7
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;			  //关闭等待
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp7Error();

  if (errorstatus == SD_OK)
  {
    CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0; /* SD Card 2.0 */
    SDType = SD_HIGH_CAPACITY;	 //这个变量用作acmd41的参数，用来询问是sdsc卡还是sdhc卡 
  }
  else		  //无响应，说明是1.x的或mmc的卡   
  {
    /* CMD55 */
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SDIO_APP_CMD);
  }
  /* CMD55 */	   
  //为什么在else里和else外面都要发送CMD55?        
  //发送cmd55，用于检测是sd卡还是mmc卡，或是不支持的卡 
  SDIO_CmdInitStructure.SDIO_Argument = 0x00;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SDIO_APP_CMD); //是否响应，没响应的是mmc或不支持的卡

  /* If errorstatus is Command TimeOut, it is a MMC card */
  /* If errorstatus is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)
     or SD card 1.x */
 
  if (errorstatus == SD_OK)	 //响应了cmd55，是sd卡，可能为1.x,可能为2.0 
  {
    /*下面开始循环地发送sdio支持的电压范围，循环一定次数*/
	/* SD CARD */							 
    /* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
    while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
    {
	  //因为下面要用到ACMD41，是ACMD命令，在发送ACMD命令前都要先向卡发送CMD55   
      /* SEND CMD55 APP_CMD with RCA as 0 */
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_APP_CMD);

      if (errorstatus != SD_OK)	//没响应CMD55，返回 
      {
        return(errorstatus);
      }
	  //acmd41，命令参数由支持的电压范围及HCS位组成，HCS位置一来区分卡是SDSc还是sdhc   
      SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SDType;	//参数为主机可供电压范围及hcs位
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_APP_OP_COND;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp3Error();	//检测是否正确接收到数据
      if (errorstatus != SD_OK)
      {
        return(errorstatus);			//没正确接收到acmd41，出错，返回 
      }

      response = SDIO_GetResponse(SDIO_RESP1);
      validvoltage = (bool) (((response >> 31) == 1) ? 1 : 0);	 //读取卡的ocr寄存器的pwr_up位，
	  															//看是否已工作在正常电压
      count++;													//计算循环次数 
    }
    if (count >= SD_MAX_VOLT_TRIAL)								//循环检测超过一定次数还没上电
    {
      errorstatus = SD_INVALID_VOLTRANGE;						//SDIO不支持card的供电电压
      return(errorstatus);
    }
	 /*检查卡返回信息中的HCS位*/ 
    if (response &= SD_HIGH_CAPACITY)			//判断ocr中的ccs位 ，如果是sdsc卡则不执行下面的语句
    {
      CardType = SDIO_HIGH_CAPACITY_SD_CARD;   //把卡类型从初始化的sdsc型改为sdhc型
    }

  }/* else MMC Card */

  return(errorstatus);
}


/*
 * 函数名：SD_PowerOFF
 * 描述  ：关掉SDIO的输出信号
 * 输入  ：无
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：外部调用
 */
SD_Error SD_PowerOFF(void)
{
  SD_Error errorstatus = SD_OK;

  /* Set Power State to OFF */
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  return(errorstatus);
}


/*
 * 函数名：SD_InitializeCards
 * 描述  ：初始化所有的卡或者单个卡进入就绪状态
 * 输入  ：无
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：在 SD_Init() 调用
 */
SD_Error SD_InitializeCards(void)
{
  SD_Error errorstatus = SD_OK;
  uint16_t rca = 0x01;

  if (SDIO_GetPowerState() == SDIO_PowerState_OFF)
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }
  /*如果不是SDIO卡，则发送CMD2*/
  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)
  {
    /* Send CMD2 ALL_SEND_CID */
	/*发送CMD2 要求所有牌就绪状态下的卡发送卡识别寄存器（CID）*/
	/*参数：-【31：0】：填充位（‘0’）*/
	/*响应类型：R2*/
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_ALL_SEND_CID;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();
	/*命令响应超时或CRC校验失败*/
    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
	/*从SDIO_RESP1..4寄存器中读取响应R2（CID寄存器）*/
    CID_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
    CID_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CID_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CID_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
  }
  /*下面开始SD卡初始化流程*/
  if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) ||  (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) ||  (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType)
      ||  (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {
    /* Send CMD3 SET_REL_ADDR with argument 0 */
    /* 发送CMD3要求卡发布一新的相对卡地址（RCA） . */
	/*参数：-【31：0】;填充位（‘0’）*/
	/*响应类型：R6*/
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_REL_ADDR;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
	//把接收到的卡相对地址存起来。
    errorstatus = CmdResp6Error(SDIO_SET_REL_ADDR, &rca);
	/*如果发生了错误，则返回错误代码*/
    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }
  /*如果不是SDIO卡*/
  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)
  {
    RCA = rca;

    /* Send CMD9 SEND_CSD with argument as card's RCA */
	/*DMD9:SEND_CSD*/
	/*发送CMD9要求被寻址的卡发送卡特定数据寄存器（CSD）*/
	/*参数：-【31：16】;相对卡地址RCA   -【15：0】;填充位（‘0’）*/
	/*响应类型：R2*/
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)(rca << 16);
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_CSD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
	/*在之前发送DMD2时，已经分析过*/
    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
    /*从SDIO_RESP1..4寄存器中读取响应R2（CSD寄存器）*/
    CSD_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
    CSD_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CSD_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CSD_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
  }
  /*卡识别结束，返回SD_OK*/
  errorstatus = SD_OK; /* All cards get intialized */

  return(errorstatus);
}

/*
 * 函数名：SD_GetCardInfo
 * 描述  ：获取SD卡的具体信息
 * 输入  ：-cardinfo 指向SD_CardInfo结构体的指针
 *         这个结构里面包含了SD卡的具体信息
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：外部调用
 */
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
  SD_Error errorstatus = SD_OK;
  uint8_t tmp = 0;

  cardinfo->CardType = (uint8_t)CardType;		 // 卡的类型，初始化为0
  cardinfo->RCA = (uint16_t)RCA;						 // ?

  // Byte 0 //
  tmp = (uint8_t)((CSD_Tab[0] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
  cardinfo->SD_csd.Reserved1 = tmp & 0x03;

  // Byte 1 //
  tmp = (uint8_t)((CSD_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.TAAC = tmp;

  // Byte 2 //
  tmp = (uint8_t)((CSD_Tab[0] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.NSAC = tmp;

  // Byte 3 //
  tmp = (uint8_t)(CSD_Tab[0] & 0x000000FF);
  cardinfo->SD_csd.MaxBusClkFrec = tmp;

  // Byte 4 //
  tmp = (uint8_t)((CSD_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CardComdClasses = tmp << 4;

  // Byte 5 //
  tmp = (uint8_t)((CSD_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
  cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;

  // Byte 6 //
  tmp = (uint8_t)((CSD_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.Reserved2 = 0; //Reserved //

  if ((CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0))
  {
    cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

    // Byte 7 //
    tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize |= (tmp) << 2;

    // Byte 8 //
    tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);
    cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

    cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
    cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

    // Byte 9 //
    tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
    cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;
    // Byte 10 //
    tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;
    
    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
    cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
    cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
    cardinfo->CardCapacity *= cardinfo->CardBlockSize;
  }
  else if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    // Byte 7 //
    tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;

    // Byte 8 //
    tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);

    cardinfo->SD_csd.DeviceSize |= (tmp << 8);

    // Byte 9 //
    tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);

    cardinfo->SD_csd.DeviceSize |= (tmp);

    // Byte 10 //
    tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);
    
    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) * 512 * 1024;
    cardinfo->CardBlockSize = 512;    
  }


  cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;

  // Byte 11 //
  tmp = (uint8_t)(CSD_Tab[2] & 0x000000FF);
  cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);

  // Byte 12 //
  tmp = (uint8_t)((CSD_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
  cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
  cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;

  // Byte 13 //
  tmp = (uint8_t)((CSD_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.Reserved3 = 0;
  cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

  // Byte 14 //
  tmp = (uint8_t)((CSD_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
  cardinfo->SD_csd.ECC = (tmp & 0x03);

  // Byte 15 //
  tmp = (uint8_t)(CSD_Tab[3] & 0x000000FF);
  cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_csd.Reserved4 = 1;


  // Byte 0 //
  tmp = (uint8_t)((CID_Tab[0] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ManufacturerID = tmp;

  // Byte 1 //
  tmp = (uint8_t)((CID_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.OEM_AppliID = tmp << 8;

  // Byte 2 //
  tmp = (uint8_t)((CID_Tab[0] & 0x000000FF00) >> 8);
  cardinfo->SD_cid.OEM_AppliID |= tmp;

  // Byte 3 //
  tmp = (uint8_t)(CID_Tab[0] & 0x000000FF);
  cardinfo->SD_cid.ProdName1 = tmp << 24;

  // Byte 4 //
  tmp = (uint8_t)((CID_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdName1 |= tmp << 16;

  // Byte 5 //
  tmp = (uint8_t)((CID_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdName1 |= tmp << 8;

  //Byte 6 //
  tmp = (uint8_t)((CID_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdName1 |= tmp;

  // Byte 7 //
  tmp = (uint8_t)(CID_Tab[1] & 0x000000FF);
  cardinfo->SD_cid.ProdName2 = tmp;

  // Byte 8 //
  tmp = (uint8_t)((CID_Tab[2] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdRev = tmp;

  // Byte 9 //
  tmp = (uint8_t)((CID_Tab[2] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdSN = tmp << 24;

  // Byte 10 //
  tmp = (uint8_t)((CID_Tab[2] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdSN |= tmp << 16;

  // Byte 11 //
  tmp = (uint8_t)(CID_Tab[2] & 0x000000FF);
  cardinfo->SD_cid.ProdSN |= tmp << 8;

  // Byte 12 //
  tmp = (uint8_t)((CID_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdSN |= tmp;

  // Byte 13 //
  tmp = (uint8_t)((CID_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
  cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

  // Byte 14 //
  tmp = (uint8_t)((CID_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ManufactDate |= tmp;

  // Byte 15 
  tmp = (uint8_t)(CID_Tab[3] & 0x000000FF);
  cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_cid.Reserved2 = 1;
  
  return(errorstatus);
}

/*
 * 函数名：SD_EnableWideBusOperation
 * 描述  ：配置卡的数据宽度(但得看卡是否支持)
 * 输入  ：-WideMode 指定SD卡的数据线宽
 *         具体可配置如下
 *         @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
 *         @arg SDIO_BusWide_4b: 4-bit data transfer
 *         @arg SDIO_BusWide_1b: 1-bit data transfer (默认)
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：外部调用
 */
SD_Error SD_EnableWideBusOperation(uint32_t WideMode)
{
  SD_Error errorstatus = SD_OK;

  /* MMC Card doesn't support this feature */
  /* MMC不支持宽总线操作模式 */
  if (SDIO_MULTIMEDIA_CARD == CardType)
  {
    errorstatus = SD_UNSUPPORTED_FEATURE;
    return(errorstatus);
  }
  else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {
    /* 以上这些卡不支持8位宽总线操作模式 */
	if (SDIO_BusWide_8b == WideMode)
    {
      errorstatus = SD_UNSUPPORTED_FEATURE;
      return(errorstatus);
    }
    else if (SDIO_BusWide_4b == WideMode)
    {
      errorstatus = SDEnWideBus(ENABLE);

      if (SD_OK == errorstatus)
      {
        /* Configure the SDIO peripheral */
		/* 配置SDIO外设的总线宽度为4位 */
        SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
        SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
        SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;
        SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&SDIO_InitStructure);
      }
    }
    else
    {
      errorstatus = SDEnWideBus(DISABLE);

      if (SD_OK == errorstatus)
      {
        /* Configure the SDIO peripheral */
		/* 配置SDIO外设的总线宽度为1位 */
        SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
        SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
        SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
        SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&SDIO_InitStructure);
      }
    }
  }

  return(errorstatus);
}

/*
 * 函数名：SD_SetDeviceMode
 * 描述  ：配置卡的数据传输方式(Polling,Interrupt or DMA 模式)
 * 输入  ：-Mode 指定数据传输模式
 *         具体可配置如下
 *         @arg SD_DMA_MODE: Data transfer using DMA.
 *         @arg SD_INTERRUPT_MODE: Data transfer using interrupts.
 *         @arg SD_POLLING_MODE: Data transfer using flags.
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：外部调用
 */
SD_Error SD_SetDeviceMode(uint32_t Mode)
{
  SD_Error errorstatus = SD_OK;

  if ((Mode == SD_DMA_MODE) || (Mode == SD_INTERRUPT_MODE) || (Mode == SD_POLLING_MODE))
  {
    DeviceMode = Mode;
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
  }
  return(errorstatus);

}

/*
 * 函数名：SD_SelectDeselect
 * 描述  ：Selects od Deselects the corresponding card
 * 输入  ：-addr 选择卡的地址
 * 输出  ：-SD_Error SD卡错误代码
 *         成功时则为 SD_OK
 * 调用  ：外部调用
 */
SD_Error SD_SelectDeselect(uint32_t addr)
{
  SD_Error errorstatus = SD_OK;

  /* Send CMD7 SDIO_SEL_DESEL_CARD */
  /* CMD7: SELECT/DESELECT_CARD -----------------------------------------------------*/
  /* 发送 CMD7 选择被寻址的SD卡 */
  /* 参数: - [31:16]: 相对卡地址RCA
           - [15:0]: 填充位(‘0’) */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument =  addr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEL_DESEL_CARD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SEL_DESEL_CARD);

  return(errorstatus);
}

/**
  * 功能描述  从指定的地址开始，从卡中读出一个数据块
  * 输入参数  readbuff: 指向读缓冲区的指针
  * 输入参数  ReadAddr: 读数据块起始地址
  * 输入参数  BlockSize: SD卡数据块长度
  * 返回值  SD_Error: SD卡错误代码
  */
SD_Error SD_ReadBlock(uint32_t addr, uint32_t *readbuff, uint16_t BlockSize)
{
  SD_Error errorstatus = SD_OK;
  uint32_t count = 0, *tempbuff = readbuff;
  uint8_t power = 0;

  if (NULL == readbuff)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;

   /* 关闭SDIO数据通道状态机(DPSM) */
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_1b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Disable;
  SDIO_DataConfig(&SDIO_DataInitStructure);
  SDIO_DMACmd(DISABLE);					/* 关闭SDIO DMA请求 */
  /* 确认卡未上锁 */
  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }
  /* 对于标准容量SD卡，写数据块长度由CMD16指定，而对于高容量SD卡，写数据块长度固定为512B */
  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    addr /= 512;
  }
  if ((BlockSize > 0) && (BlockSize <= 2048) && ((BlockSize & (BlockSize - 1)) == 0))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);

     /* CMD16: SET_BLOCKLEN ----------------------------------------------------------*/
    /* 发送 CMD16 设置卡的数据块长度 */
    /* 参数: - [31:0]: 数据块长度 */
    /* 响应类型: R1 */
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }
   /* 初始化并使能SDIO数据通道状态机(DPSM) */
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
  /* SDIO_DataBlockSize用于设置主机数据块长度DBCKSIZE位(SDIO_DCTRL[7:4]) */
  SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) power << 4;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);

  TotalNumberOfBytes = BlockSize;
  StopCondition = 0;
  DestBuffer = readbuff;

  /* CMD17: READ_SINGLE_BLOCK -------------------------------------------------------*/
  /* 发送 CMD17 开始读一个数据块 */
  /* 参数: - [31:0]: 数据地址 */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)addr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_READ_SINGLE_BLOCK;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  //SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_Pend;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_READ_SINGLE_BLOCK);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
  /* 单数据块传输结束后，不需要发送停止传输命令(因为我们发送的命令就是要求卡发送一个数据块) */
  /* 数据传输模式为中断模式 -------------------------------------------------------------*/
  /* In case of single block transfer, no need of stop transfer at all.*/
  if (DeviceMode == SD_POLLING_MODE)
  {
     /* 当下列标志都不为1时，继续接收数据 */
    while (!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL 
						| SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND 
						| SDIO_FLAG_STBITERR)))
    {
	/* 如果接收FIFO半满：FIFO中至少还有8个字，则从接收FIFO中读出8个字*/
      if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)
      {
        for (count = 0; count < 8; count++)
        {
		 /* 从接收FIFO中读出1个字 */
          *(tempbuff + count) = SDIO_ReadData();
        }
        tempbuff += 8;
      }
    }
	 /* 数据超时 */
    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
    {
      SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
      errorstatus = SD_DATA_TIMEOUT;
      return(errorstatus);
    }
	/* 已发送/接收数据块，但是CRC校验失败 */
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
    {
      SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
      errorstatus = SD_DATA_CRC_FAIL;
      return(errorstatus);
    }
	/* 接收FIFO上溢错误 */
    else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
    {
      SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
      errorstatus = SD_RX_OVERRUN;
      return(errorstatus);
    }
	/* 在宽总线模式下，在所有数据线上都没有检测到起始位 */
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
    {
      SDIO_ClearFlag(SDIO_FLAG_STBITERR);
      errorstatus = SD_START_BIT_ERR;
      return(errorstatus);
    }
	 /* 如果接收FIFO中还有未读出的有效数据，则将其读出 */
    while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
    {
      *tempbuff = SDIO_ReadData();	  /* 从接收FIFO中读出1个字 */
      tempbuff++;
    }

    /* 清除所有静态标志 */
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);
  } 
  /* 数据传输模式为中断模式 -------------------------*/
  else if (DeviceMode == SD_INTERRUPT_MODE)
  {	 
  /* 使能下列中断 */
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_RXFIFOHF | SDIO_IT_STBITERR, ENABLE);
    while ((TransferEnd == 0) && (TransferError == SD_OK))
    {} /* 等待数据传输结束 */
    if (TransferError != SD_OK)
    {
      return(TransferError);
    }
  }
   /* 数据传输模式为DMA模式 ------------------------*/
  else if (DeviceMode == SD_DMA_MODE)
  {
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
    SDIO_DMACmd(ENABLE);
    DMA_RxConfiguration(readbuff, BlockSize);
    while (DMA_GetFlagStatus(DMA2_FLAG_TC4) == RESET)
    {}
  }
  return(errorstatus);
}

/**
  * 功能描述  从指定的地址开始，从卡中读多个数据块
  * 输入参数  readbuff: 指向接收缓冲区的指针
  * 输入参数  ReadAddr: 读数据块起始地址
  * 输入参数  BlockSize: SD卡数据块长度
  * 输入参数  NumberOfBlocks: 数据块数目
  * 返回值  SD_Error: SD卡错误状态
  */
SD_Error SD_ReadMultiBlocks(uint32_t addr, uint32_t *readbuff, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  SD_Error errorstatus = SD_OK;
  uint32_t count = 0, *tempbuff = readbuff;
  uint8_t power = 0;

  if (NULL == readbuff)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;

  /* 关闭SDIO数据通道状态机(DPSM) */
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_1b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Disable;
  SDIO_DataConfig(&SDIO_DataInitStructure);
  SDIO_DMACmd(DISABLE);				/* 关闭SDIO DMA请求 */
   /* 确认卡未上锁 */
  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }
  /* 对于标准容量SD卡，写数据块长度由CMD16指定，而对于高容量SD卡，写数据块长度固定为512B */
  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    addr /= 512;
  }
  /* 分别设置卡和主机的数据块长度 */
  if ((BlockSize > 0) && (BlockSize <= 2048) && (0 == (BlockSize & (BlockSize - 1))))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);

    /* CMD16: SET_BLOCKLEN ----------------------------------------------------------*/
    /* 发送 CMD16 设置卡的数据块长度 */
    /* 参数: - [31:0]: 数据块长度 */
    /* 响应类型: R1 */
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  if (NumberOfBlocks > 1)
  {
    /* Common to all modes */
    if (NumberOfBlocks * BlockSize > SD_MAX_DATA_LENGTH)
    {
      errorstatus = SD_INVALID_PARAMETER;
      return(errorstatus);
    }

    TotalNumberOfBytes = NumberOfBlocks * BlockSize;
    StopCondition = 1;
    DestBuffer = readbuff;
	 /* 初始化并使能SDIO数据通道状态机(DPSM) */
    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
	 /* SDIO_DataBlockSize用于设置主机数据块长度DBCKSIZE位(SDIO_DCTRL[7:4]) */
    SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) power << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    /* CMD18: READ_MULTIPLE_BLOCK ---------------------------------------------------*/
    /* 发送 CMD18 开始读多个数据块 */
    /* 参数: - [31:0]: 数据地址 */
    /* 响应类型: R1 */
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)addr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_READ_MULT_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_READ_MULT_BLOCK);

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
	 /* 数据传输模式为轮询模式 ------------------------------*/
    if (DeviceMode == SD_POLLING_MODE)
    {
       /* 当下列标志都不为 1 时，继续接收数据 */
      while (!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DATAEND | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_STBITERR)))
      {
         /* 如果接收FIFO半满：FIFO中至少还有8个字，则从接收FIFO中读出 SD_HALFFIFO (8)个字*/
		if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)
        {
          for (count = 0; count < SD_HALFFIFO; count++)
          {
            *(tempbuff + count) = SDIO_ReadData(); /* 从接收FIFO中读出1个字 */
          }
          tempbuff += SD_HALFFIFO;
        }
      }
	   /* 数据超时 */
      if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return(errorstatus);
      }
	   /* 已发送/接收数据块，但是CRC校验失败 */
      else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        errorstatus = SD_DATA_CRC_FAIL;
        return(errorstatus);
      }
	  /* 接收FIFO上溢错误 */
      else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
        errorstatus = SD_RX_OVERRUN;
        return(errorstatus);
      }
	  /* 在宽总线模式下，在所有数据线上都没有检测到起始位 */
      else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        errorstatus = SD_START_BIT_ERR;
        return(errorstatus);
      }
	   /* 如果接收FIFO中还有未读出的有效数据，则将其读出 */
      while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
      {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
      }
	  /* 数据传输结束后(已接收到NumberOfBlocks * BlockSize个字节)，发送停止传输命令(CMD12) */
      /* 我们发送的命令只是要求卡发送多个数据块，而卡不知道究竟要发送几个数据块 */
      if (SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET)
      {
        /* In Case Of SD-CARD Send Command STOP_TRANSMISSION */
        if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType))
        {
          /* CMD12: STOP_TRANSMISSION ------------------------------------------------*/
          /* 发送 CMD12 强制卡停止数据传输 */
          /* 参数: - [31:0]: 填充位(‘0’) */
          /* 响应类型: R1b(短) */
          SDIO_CmdInitStructure.SDIO_Argument = 0x0;
          SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_STOP_TRANSMISSION;
          SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
          SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
          SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
          SDIO_SendCommand(&SDIO_CmdInitStructure);

          errorstatus = CmdResp1Error(SDIO_STOP_TRANSMISSION);

          if (errorstatus != SD_OK)
          {
            return(errorstatus);
          }
        }
      }
      /* Clear all the static flags */
      SDIO_ClearFlag(SDIO_STATIC_FLAGS);
    }
    else if (DeviceMode == SD_INTERRUPT_MODE)
    {
      SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_RXFIFOHF | SDIO_IT_STBITERR, ENABLE);
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
	/* 数据传输模式为DMA模式 ------------------------------*/
    else if (DeviceMode == SD_DMA_MODE)
    {
      SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
      SDIO_DMACmd(ENABLE);
      DMA_RxConfiguration(readbuff, (NumberOfBlocks * BlockSize));
      while (DMA_GetFlagStatus(DMA2_FLAG_TC4) == RESET)
      {}
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
  }
  return(errorstatus);
}

/**
  * 功能描述  从指定的地址开始，向卡中写入一个数据块
  * 输入参数  writebuff: 指向写缓冲区的指针
  * 输入参数  WriteAddr: 写数据块起始地址
  * 输入参数  BlockSize: SD卡数据块长度
  * 返回值  SD_Error: SD卡错误代码
  */
SD_Error SD_WriteBlock(uint32_t addr, uint32_t *writebuff, uint16_t BlockSize)
{
  SD_Error errorstatus = SD_OK;
  uint8_t  power = 0, cardstate = 0;
  uint32_t timeout = 0, bytestransferred = 0;
  uint32_t cardstatus = 0, count = 0, restwords = 0;
  uint32_t *tempbuff = writebuff;

  if (writebuff == NULL)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;
  /* 关闭SDIO数据通道状态机(DPSM) */
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_1b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Disable;
  SDIO_DataConfig(&SDIO_DataInitStructure);
  /* 关闭SDIO DMA请求 */
  SDIO_DMACmd(DISABLE);
  /* 确认卡未上锁 */
  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }
   /* 对于标准容量SD卡，写数据块长度由CMD16指定，而对于高容量SD卡，写数据块长度固定为512B */
  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    addr /= 512;
  }
  /* 分别设置卡和主机的数据块长度 */ 
  /* Set the block size, both on controller and card */
  if ((BlockSize > 0) && (BlockSize <= 2048) && ((BlockSize & (BlockSize - 1)) == 0))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);
	/* CMD16: SET_BLOCKLEN ----------------------------------------------------------*/
    /* 发送 CMD16 设置卡的数据块长度 */
    /* 参数: - [31:0]: 数据块长度 */
    /* 响应类型: R1 */
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }
  /* CMD13: SEND_STATUS -------------------------------------------------------------*/
  /* 发送 CMD13 要求卡发送状态寄存器(这里指卡状态，如果该命令在紧随CMD55之后发送，则为SD状态) */
  /* 参数: - [31:16]: 相对卡地址RCA
           - [15:0]: 填充位(‘0’) */
  /* 响应类型: R1 */
  /* Wait till card is ready for data Added */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) (RCA << 16);
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SEND_STATUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  cardstatus = SDIO_GetResponse(SDIO_RESP1);

  timeout = SD_DATATIMEOUT;
  
  /* 等待直到卡准备好接收数据(卡状态位READY_FOR_DATA(卡状态位9) = 1)或数据超时 */
  while (((cardstatus & 0x00000100) == 0) && (timeout > 0))
  {
    timeout--;
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) (RCA << 16);
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SEND_STATUS);

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
    cardstatus = SDIO_GetResponse(SDIO_RESP1);
  }

  if (timeout == 0)
  {
    return(SD_ERROR);
  }
  /* CMD24: WRITE_SINGLE_BLOCK ------------------------------------------------------*/
  /* 发送 CMD24 开始写一个数据块 */
  /* 参数: - [31:0]: 数据地址 */
  /* 响应类型: R1 */
  /* Send CMD24 WRITE_SINGLE_BLOCK */
  SDIO_CmdInitStructure.SDIO_Argument = addr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_WRITE_SINGLE_BLOCK;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_WRITE_SINGLE_BLOCK);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  TotalNumberOfBytes = BlockSize;
  StopCondition = 0;
  SrcBuffer = writebuff;
  /* 初始化并使能SDIO数据通道状态机(DPSM) */
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
  /* SDIO_DataBlockSize用于设置主机数据块长度DBCKSIZE位(SDIO_DCTRL[7:4]) */
  SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) power << 4;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);
  /* 单数据块传输结束后，不需要发送停止传输命令 */
  /* 数据传输模式为轮询模式 -------------------------------------------------------------*/
  /* In case of single data block transfer no need of stop command at all */
  if (DeviceMode == SD_POLLING_MODE)
  {
  /* 当下列标志都不为 1 时，继续发送数据 */
    while (!(SDIO->STA & (SDIO_FLAG_DBCKEND | SDIO_FLAG_TXUNDERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_STBITERR)))
    {
	/* 如果发送FIFO半空：FIFO中至少还可以写入8个字 */
      if (SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)
      {
	  /* 如果剩下的待发送的数据字节小于32个字节(8个字)，则只发送剩下的字 */
        if ((TotalNumberOfBytes - bytestransferred) < 32)
        {
          restwords = ((TotalNumberOfBytes - bytestransferred) % 4 == 0) ? ((TotalNumberOfBytes - bytestransferred) / 4) : (( TotalNumberOfBytes -  bytestransferred) / 4 + 1);

          for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
          {
             /* 向发送FIFO中写入1个字 */
			SDIO_WriteData(*tempbuff);
          }
        }
		 /* 如果剩下的待发送的数据字节大于或等于32个字节(8个字)，则向发送FIFO中写入8个字 */
        else
        {
          for (count = 0; count < 8; count++)
          {	 
		  /* 向发送FIFO中写入1个字 */
            SDIO_WriteData(*(tempbuff + count));
          }
          tempbuff += 8;
          bytestransferred += 32;
        }
      }
    }
	 /* 数据超时 */
    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
    {
      SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
      errorstatus = SD_DATA_TIMEOUT;
      return(errorstatus);
    }
	/* 已发送/接收数据块，但是CRC校验失败 */
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
    {
      SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
      errorstatus = SD_DATA_CRC_FAIL;
      return(errorstatus);
    }
	/* 发送FIFO下溢错误 */
    else if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET)
    {
      SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
      errorstatus = SD_TX_UNDERRUN;
      return(errorstatus);
    }
	/* 在宽总线模式下，在所有数据线上都没有检测到起始位 */
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
    {
      SDIO_ClearFlag(SDIO_FLAG_STBITERR);
      errorstatus = SD_START_BIT_ERR;
      return(errorstatus);
    }
  }
   /* 数据传输模式为中断模式 -------------------------*/
  else if (DeviceMode == SD_INTERRUPT_MODE)
  {
    /* 使能下列中断 */
	SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_FLAG_TXFIFOHE | SDIO_IT_TXUNDERR | SDIO_IT_STBITERR, ENABLE);
     /* 等待数据传输结束 */
	while ((TransferEnd == 0) && (TransferError == SD_OK))
    {}
    if (TransferError != SD_OK)
    {
      return(TransferError);
    }
  }
  /* 数据传输模式为DMA模式 ---------------------------*/
  else if (DeviceMode == SD_DMA_MODE)
  {
    /* 使能下列中断 */
	SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_TXUNDERR | SDIO_IT_STBITERR, ENABLE);
    /* 配置并使能SDIO DMA发送请求 */
	DMA_TxConfiguration(writebuff, BlockSize);
    SDIO_DMACmd(ENABLE);
    /* 等待数据传输结束 */
	while (DMA_GetFlagStatus(DMA2_FLAG_TC4) == RESET)
    {}
    while ((TransferEnd == 0) && (TransferError == SD_OK))
    {}
    if (TransferError != SD_OK)
    {
      return(TransferError);
    }
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /* Wait till the card is in programming state */
  errorstatus = IsCardProgramming(&cardstate);

  while ((errorstatus == SD_OK) && ((cardstate == SD_CARD_PROGRAMMING) || (cardstate == SD_CARD_RECEIVING)))
  {
    errorstatus = IsCardProgramming(&cardstate);
  }

  return(errorstatus);
}

/**
  * 功能描述  从指定的地址开始，向卡中写入多个数据块
  * 输入参数  WriteAddr: 写数据块起始地址
  * 输入参数  writebuff: 指向写缓冲区的指针
  * 输入参数  BlockSize: SD卡数据块长度
  * 输入参数  NumberOfBlocks: 块数目
  * 返回值  SD_Error: SD卡错误代码
  */
SD_Error SD_WriteMultiBlocks(uint32_t addr, uint32_t *writebuff, uint16_t BlockSize, uint32_t NumberOfBlocks)
{							     
 
  SD_Error errorstatus = SD_OK;
  uint8_t  power = 0, cardstate = 0;
  uint32_t bytestransferred = 0;
  uint32_t count = 0, restwords = 0;
  uint32_t *tempbuff = writebuff;

  if (writebuff == NULL)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;
   // 关闭SDIO数据通道状态机(DPSM) //
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_1b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Disable;
  SDIO_DataConfig(&SDIO_DataInitStructure);
  SDIO_DMACmd(DISABLE); // 关闭SDIO DMA请求 //
   // 确认卡未上锁 //
  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }
   // 对于标准容量SD卡，写数据块长度由CMD16指定，而对于高容量SD卡，写数据块长度固定为512B //
  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    addr /= 512;
  }
  
  // 分别设置卡和主机的数据块长度 // 
  if ((BlockSize > 0) && (BlockSize <= 2048) && ((BlockSize & (BlockSize - 1)) == 0))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);
	// CMD16: SET_BLOCKLEN ----------------------------------------------------------//
    // 发送 CMD16 设置卡的数据块长度 //
    // 参数: - [31:0]: 数据块长度 //
    // 响应类型: R1 //
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  // CMD13: SEND_STATUS -------------------------------------------------------------//
  /// 发送 CMD13 要求卡发送状态寄存器(这里指卡状态，如果该命令在紧随CMD55之后发送，则为SD状态) //
  // 参数: - [31:16]: 相对卡地址RCA
   //        - [15:0]: 填充位(‘0’) //
  // 响应类型: R1 //
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) (RCA << 16);
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SEND_STATUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  if (NumberOfBlocks > 1)
  {
    // Common to all modes //
    if (NumberOfBlocks * BlockSize > SD_MAX_DATA_LENGTH)
    {
      errorstatus = SD_INVALID_PARAMETER;
      return(errorstatus);
    }
	// 设置预擦除写数据块数目(ACMD23)，为加快多块写操作速度 //
    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
    {
      // CMD55: APP_CMD //
      // 必须在发送 ACMD23 之前发送 CMD55 通知卡，下面一条命令是应用特定命令//
      // 参数: - [31:16]: 相对卡地址RCA 
      //         - [15:0]: 保留('0') //
      // 响应类型: R1 //
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) (RCA << 16);
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(SDIO_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
       // ACMD23: SET_WR_BLK_ERASE_COUNT ----------------------------------------------//
      // 发送 ACMD23 设置多块写命令前的预擦除写数据块数目 //
      // 参数: - [31:23]: 填充位('0') 
       //        - [22:0]: 预擦除写数据块数目 //
      // 响应类型: R1 //
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)NumberOfBlocks;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCK_COUNT;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_SET_BLOCK_COUNT);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
    }

    // CMD23: SET_WR_BLK_ERASE_COUNT -------------------------//
    // 发送 CMD25 开始写多个多块 //
    // 参数: - [31:0]: 数据地址 //
    // 响应类型: R1//
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)addr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_WRITE_MULT_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_WRITE_MULT_BLOCK);

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
	 // 初始化并使能SDIO数据通道状态机(DPSM) //
    TotalNumberOfBytes = NumberOfBlocks * BlockSize;
    StopCondition = 1;
    SrcBuffer = writebuff;

    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
	// SDIO_DataBlockSize用于设置主机数据块长度DBCKSIZE位(SDIO_DCTRL[7:4]) //
    SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) power << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);
	//数据传输模式为轮询模式 -------------------------------------------//
    if (DeviceMode == SD_POLLING_MODE)
    {
	// 当下列标志都不为1时，继续发送数据 //
      while (!(SDIO->STA & (SDIO_FLAG_TXUNDERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DATAEND | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_STBITERR)))
      {
        // 如果剩下的待发送的数据字节大于或等于SD_HALFFIFOBYTES(32)个字节(8个字)，
		//	则发送SD_HALFFIFO(8)个字 //
		if (SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)
        {
          if (!((TotalNumberOfBytes - bytestransferred) < SD_HALFFIFOBYTES))
          {
            for (count = 0; count < SD_HALFFIFO; count++)
            {
              SDIO_WriteData(*(tempbuff + count));  // 向发送FIFO中写入1个字//
            }
            tempbuff += SD_HALFFIFO;
            bytestransferred += SD_HALFFIFOBYTES;
          }
		   // 如果剩下的待发送的数据字节小于SD_HALFFIFOBYTES(32)个字节(8个字)，则只发送剩下的字 //
          else
          {
            restwords = ((TotalNumberOfBytes - bytestransferred) % 4 == 0) ? ((TotalNumberOfBytes - bytestransferred) / 4) :
                        ((TotalNumberOfBytes - bytestransferred) / 4 + 1);

            for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
            {
              SDIO_WriteData(*tempbuff);
            }
          }
        }
      }
	   // 数据超时 //
      if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return(errorstatus);
      }
	   // 已发送/接收数据块，但是CRC校验失败 //
      else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        errorstatus = SD_DATA_CRC_FAIL;
        return(errorstatus);
      }
	   // 发送FIFO下溢错误 //
      else if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
        errorstatus = SD_TX_UNDERRUN;
        return(errorstatus);
      }
	   // 在宽总线模式下，在所有数据线上都没有检测到起始位 //
      else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        errorstatus = SD_START_BIT_ERR;
        return(errorstatus);
      }
	   // 数据传输结束后(已发送NumberOfBlocks * BlockSize个字节)，发送停止传输命令(CMD12) //
      if (SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET)
      {
       if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
        {
          //CMD12: STOP_TRANSMISSION ------------------------------------------------//
          // 发送 CMD12 强制卡停止数据传输 //
          // 参数: - [31:0]: 填充位(‘0’) //
          // 响应类型: R1b(短) //
          SDIO_CmdInitStructure.SDIO_Argument = 0x0;
          SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_STOP_TRANSMISSION;
          SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
          SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
          SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
          SDIO_SendCommand(&SDIO_CmdInitStructure);


          errorstatus = CmdResp1Error(SDIO_STOP_TRANSMISSION);

          if (errorstatus != SD_OK)
          {
            return(errorstatus);
          }
        }
      }
    }
	 //数据传输模式为中断模式 ---------------------------------///
    else if (DeviceMode == SD_INTERRUPT_MODE)
    {
      SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_TXFIFOHE | SDIO_IT_TXUNDERR | SDIO_IT_STBITERR, ENABLE);
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
	// 数据传输模式为DMA模式 ---------------------------------//
    else if (DeviceMode == SD_DMA_MODE)
    {
      SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_TXUNDERR | SDIO_IT_STBITERR, ENABLE);
      SDIO_DMACmd(ENABLE);
      DMA_TxConfiguration(writebuff, (NumberOfBlocks * BlockSize));
      while (DMA_GetFlagStatus(DMA2_FLAG_TC4) == RESET)
      {}
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
  }
  // Clear all the static flags //
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  // Wait till the card is in programming state //
  errorstatus = IsCardProgramming(&cardstate);

  while ((errorstatus == SD_OK) && ((cardstate == SD_CARD_PROGRAMMING) || (cardstate == SD_CARD_RECEIVING)))
  {
    errorstatus = IsCardProgramming(&cardstate);
  }

  return(errorstatus);
}

/**
  * 功能描述  返回SD卡主机当前的数据传输状态
  * 输入参数  无
  * 返回值  SDTransferState: 数据传输状态
  *     该值可以是: 
  *         SD_TRANSFER_OK: 没有数据在传输
  *         SD_TRANSFER_BUSY: 数据正在传输
  */
SDTransferState SD_GetTransferState(void)
{
  if (SDIO->STA & (SDIO_FLAG_TXACT | SDIO_FLAG_RXACT))
  {
    return(SD_TRANSFER_IN_PROGRESS);
  }
  else
  {
    return(SD_NO_TRANSFER);
  }
}

/**
  * 功能描述  终止正在进行的数据传输
  * 输入参数  无
  * 返回值  SD_Error: SD卡错误代码
  */
SD_Error SD_StopTransfer(void)
{
  SD_Error errorstatus = SD_OK;

  /* CMD12: STOP_TRANSMISSION -------------------------------------------------------*/
  /* 发送 CMD12 强制卡停止数据传输 */
  /* 参数: - [31:0]: 填充位(‘0’) */
  /* 响应类型: R1b(短) */
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_STOP_TRANSMISSION;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_STOP_TRANSMISSION);

  return(errorstatus);
}

/**
  * 功能描述  擦除给定卡指定的存储区
  * 输入参数  startaddr: 起始地址
  * 输入参数  endaddr: 结束地址
  * 返回值  SD_Error: SD卡错误代码
  */
SD_Error SD_Erase(uint32_t startaddr, uint32_t endaddr)
{
  SD_Error errorstatus = SD_OK;
  uint32_t delay = 0;
  __IO uint32_t maxdelay = 0;
  uint8_t cardstate = 0;

  /* Check if the card coomnd class supports erase command */
  /* 检查卡是否支持擦出命令类(class 5)，即CCC(CSD[95:84])D5位是否为1 */
  /* 如果不支持，则返回错误代码 */
  if (((CSD_Tab[1] >> 20) & SD_CCCC_ERASE) == 0)
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }
/*有见过不是72000而是120000的****************************************************11111111111111111111111111111111111111111111111111111111111111***/
  maxdelay = 72000 / ((SDIO->CLKCR & 0xFF) + 2);
   /* 如果卡已被上锁 */
  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }
   /* 在标准容量SD存储卡中，地址是以字节为单位的，
   而在高容量SD存储卡中，地址是以块(512字节)为单位的 */
  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    startaddr /= 512;
    endaddr /= 512;
  }
   /* 设置待擦除连续块的起始地址和结束地址 */
  /* According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
  if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || 
  													(SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {
    /* Send CMD32 SD_ERASE_GRP_START with argument as addr  */
	 /* CMD32: ERASE_WR_BLK_START ----------------------------------------------------*/
    /* 发送 CMD32 设置待擦除连续块的起始地址 */
    /* 参数: - [31:0]: 数据地址 */
    /* 响应类型: R1 */
    SDIO_CmdInitStructure.SDIO_Argument = startaddr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_ERASE_GRP_START;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SD_ERASE_GRP_START);
    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }

    /* Send CMD33 SD_ERASE_GRP_END with argument as addr  */
	/* CMD33: ERASE_WR_BLK_END ------------------------------------------------------*/
    /* 发送 CMD33 设置待擦除连续块的起始地址 */
    /* 参数: - [31:0]: 数据地址 */
    /* 响应类型: R1 */
    SDIO_CmdInitStructure.SDIO_Argument = endaddr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_ERASE_GRP_END;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SD_ERASE_GRP_END);
    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
  }

  /* Send CMD38 ERASE */
   /* CMD38: ERASE -------------------------------------------------------------------*/
  /* 发送 CMD38 擦除之前CMD32和CMD33选择的连续块 */
  /* 参数: - [31:0]: 填充位(‘0’) */
  /* 响应类型: R1b */
  SDIO_CmdInitStructure.SDIO_Argument = 0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_ERASE;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_ERASE);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  for (delay = 0; delay < maxdelay; delay++)
  {}
   /* 等待直到擦除操作完成 */
  /* Wait till the card is in programming state */
  errorstatus = IsCardProgramming(&cardstate);

  while ((errorstatus == SD_OK) && ((SD_CARD_PROGRAMMING == cardstate) || (SD_CARD_RECEIVING == cardstate)))
  {
    errorstatus = IsCardProgramming(&cardstate);
  }

  return(errorstatus);
}

/**
  * 功能描述  返回当前的卡状态(卡状态寄存器)
  * 输入参数  pcardstatus: 指向SD卡状态数据的指针
  * 返回值  SD_Error: SD卡错误代码
  */
SD_Error SD_SendStatus(uint32_t *pcardstatus)
{
  SD_Error errorstatus = SD_OK;

  if (pcardstatus == NULL)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }
   /* CMD13: SEND_STATUS -------------------------------------------------------------*/
  /* 发送 CMD13 要求卡发送状态寄存器(这里指卡状态，如果该命令在紧随CMD55之后发送，则为SD状态) */
  /* 参数: - [31:16]: 相对卡地址RCA
            - [15:0]: 填充位(‘0’) */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);


  errorstatus = CmdResp1Error(SDIO_SEND_STATUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  *pcardstatus = SDIO_GetResponse(SDIO_RESP1);

  return(errorstatus);
}

/**
  * 功能描述  返回当前的SD状态(SD状态寄存器，512b，64B)
  * 输入参数  psdstatus: 指向SD状态数据的指针
  * 返回值  SD_Error: SD卡错误代码
  */
SD_Error SD_SendSDStatus(uint32_t *psdstatus)
{
  SD_Error errorstatus = SD_OK;
  uint32_t count = 0;
  /* 确认SD卡未上锁 */
  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

   /* 由于SD状态是通过数据线发送的，因此要设置卡和主机的数据块长度 */
  /* CMD16: SET_BLOCKLEN ----------------------------------------------------------*/
  /* 发送 CMD16 设置卡的数据块长度 */
  /* 参数: - [31:0]: 数据块长度 */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument = 64;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /* CMD55: APP_CMD ---------------------------------*/
  /* 必须在发送 ACMD13 之前发送 CMD55 通知卡，下面一条命令是应用特定命令 */
  /* 参数: - [31:16]: 相对卡地址RCA 
            - [15:0]: 保留('0') */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SDIO_APP_CMD);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
   /* 主机的数据块长度在初始化DPSM时设置 */
  /* 初始化并使能SDIO数据通道状态机(DPSM) */
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 64;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_64b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);

  /* ACMD13: SD_STATUS --------------------------------------------------------------*/
  /* 发送 ACMD13 要求卡发送状态寄存器(这里指SD状态，因为该命令是在紧随CMD55之后发送的) */
  /* 参数: - [31:0]: 填充位(‘0’) */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument = 0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_APP_STAUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SDIO_SD_APP_STAUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
   /* 当下列标志都不为1时，继续接收数据 */
  while (!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
  {	/* 如果接收FIFO半满：FIFO中至少还有8个字，则从接收FIFO中读出8个字 */
    if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)
    {
      for (count = 0; count < 8; count++)
      {
        *(psdstatus + count) = SDIO_ReadData();
      }
      psdstatus += 8;
    }
  }
   /* 数据超时 */ 
  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  /* 已接收数据块，但CRC校验错误 */
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
   /* 接收FIFO上溢错误 */
  else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  /* 在宽总线模式下，在所有数据线上都没有检测到起始位 */
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }
  /* 接收FIFO中还有未读出的有效数据 */
  while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
  {
    *psdstatus = SDIO_ReadData();
    psdstatus++;
  }

  /* Clear all the static status flags*/
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);
  /* SD状态的数据包格式属于宽幅数据，先接收的是高字节 */
  psdstatus -= 16;
  for (count = 0; count < 16; count++)
  {
    psdstatus[count] = ((psdstatus[count] & SD_0TO7BITS) << 24) |((psdstatus[count] & SD_8TO15BITS) << 8) |
                       ((psdstatus[count] & SD_16TO23BITS) >> 8) |((psdstatus[count] & SD_24TO31BITS) >> 24);
  }
  return(errorstatus);
}

/**
  * 功能描述  处理SD卡数据传输中断请求
  * 输入参数  无
  * 返回值  SD_Error: SD卡错误代码
  */
SD_Error SD_ProcessIRQSrc(void)
{
  uint32_t count = 0, restwords = 0;
  /* 如果数据传输模式为中断模式，发送/接收数据按下面if语句进行处理 */
  if (DeviceMode == SD_INTERRUPT_MODE)
  {
    if (SDIO_GetITStatus(SDIO_IT_RXFIFOHF) != RESET)
    {
       /* 如果接收FIFO半满：FIFO中至少还有8个字，则从接收FIFO中读出 SD_HALFFIFO (8)个字*/
	  for (count = 0; count < SD_HALFFIFO; count++)
      {
        *(DestBuffer + count) = SDIO_ReadData();
      }
      DestBuffer += SD_HALFFIFO;
      NumberOfBytes += SD_HALFFIFOBYTES;
    }
	/* 如果发送FIFO半空：FIFO中至少还可以写入8个字 */
    else if (SDIO_GetITStatus(SDIO_IT_TXFIFOHE) != RESET)
    {
	 /* 如果剩下的待发送的数据字节小于SD_HALFFIFOBYTES(32)个字节(8个字)，则只发送剩下的字 */
      if ((TotalNumberOfBytes - NumberOfBytes) < SD_HALFFIFOBYTES)
      {
        restwords = ((TotalNumberOfBytes - NumberOfBytes) %  4 == 0) ?
                    ((TotalNumberOfBytes - NumberOfBytes) / 4) :
                    ((TotalNumberOfBytes - NumberOfBytes) / 4 + 1);

        for (count = 0; count < restwords;  count++, SrcBuffer++, NumberOfBytes += 4)
        {
          SDIO_WriteData(*SrcBuffer);
        }
      }
	   /* 如果剩下的待发送的数据字节大于或等于SD_HALFFIFOBYTES(32)个字节(8个字)，则发送SD_HALFFIFO(8)个字 */
      else
      {
        for (count = 0; count < SD_HALFFIFO; count++)
        {
          SDIO_WriteData(*(SrcBuffer + count));
        }

        SrcBuffer += SD_HALFFIFO;
        NumberOfBytes += SD_HALFFIFOBYTES;
      }
    }
  }
  /* 数据传输结束 */
  if (SDIO_GetITStatus(SDIO_IT_DATAEND) != RESET)
  {
  /* 如果数据传输模式是中断模式，则还要检查接收FIFO中是否还有未读出的有效数据 */
    if (DeviceMode != SD_DMA_MODE)
    {
	/* 如果接收FIFO中还有未读出的有效数据，则将其读出 */
	  while ((SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)  &&  (NumberOfBytes < TotalNumberOfBytes))
      {
        *DestBuffer = SDIO_ReadData();
        DestBuffer++;
        NumberOfBytes += 4;
      }
    }
	/* 停止数据传输 */
    if (StopCondition == 1)
    {
      TransferError = SD_StopTransfer();
    }
    else
    {
      TransferError = SD_OK;
    }
    SDIO_ClearITPendingBit(SDIO_IT_DATAEND);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    TransferEnd = 1;
    NumberOfBytes = 0;
    return(TransferError);
  }
   /* 已发送/接收数据块，但是CRC校验失败 */
  if (SDIO_GetITStatus(SDIO_IT_DCRCFAIL) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_DCRCFAIL);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_DATA_CRC_FAIL;
    return(SD_DATA_CRC_FAIL);
  }
  /* 数据超时 */
  if (SDIO_GetITStatus(SDIO_IT_DTIMEOUT) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_DTIMEOUT);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_DATA_TIMEOUT;
    return(SD_DATA_TIMEOUT);
  }
  /* 接收FIFO上溢错误 */
  if (SDIO_GetITStatus(SDIO_IT_RXOVERR) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_RXOVERR);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_RX_OVERRUN;
    return(SD_RX_OVERRUN);
  }
  /* 发送FIFO下溢错误 */
  if (SDIO_GetITStatus(SDIO_IT_TXUNDERR) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_TXUNDERR);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_TX_UNDERRUN;
    return(SD_TX_UNDERRUN);
  }
   /* 在宽总线模式下，在所有数据线上都没有检测到起始位 */
  if (SDIO_GetITStatus(SDIO_IT_STBITERR) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_STBITERR);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_START_BIT_ERR;
    return(SD_START_BIT_ERR);
  }

  return(SD_OK);
}

/**
  * @brief  Checks for error conditions for CMD0.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdError(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t timeout;

  timeout = SDIO_CMD0TIMEOUT; /* 10000 */

  while ((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET))
  {
    timeout--;
  }

  if (timeout == 0)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    return(errorstatus);
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

/*******************************************************
* @brief  检查响应R7的错误条件
* @param  None
* @retval SD_Error: SD Card Error code.
******************************************************/
static SD_Error CmdResp7Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t timeout = SDIO_CMD0TIMEOUT;

  status = SDIO->STA;
 
  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0))
  {
    timeout--;
    status = SDIO->STA;
  }

  if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
  {
    /*命令响应超时，不符合SD卡2.0标准或不支持设置的电压
	范围（2.7-3.6），返回命令响应超时错误*/
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
	/*已收到命令R7,并且CRC校验成功，符合SD卡2.0标准，返回SD_OK */
  if (status & SDIO_FLAG_CMDREND)							  
  {    
    errorstatus = SD_OK;
    SDIO_ClearFlag(SDIO_FLAG_CMDREND);
    return(errorstatus);
  }
  /*已收到命令响应R7，但是CRC校验失败，符合SD卡2.0标准返回SD_OK*/
  return(errorstatus);
}

/**
  * @brief  检查响应R1的错误条件
  * @param  cmd: 已发送命令的索引.
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp1Error(uint8_t cmd)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t response_r1;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }
  /*命令响应超时*/
  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /*已收到命令响R1，但校验失败*/
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }
  /*已收到的命令响应R1，并且校验成功*/
  /* 检查收到的响应是不是该命令的，如果不是，返回非法命令错误 */
  if (SDIO_GetCommandResponse() != cmd)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /*清除所有的静态标志 */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /*从SDIO_RESP1寄存器中读取响应R1，并解析 */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);
  /*如果卡状态寄存器的各位（不考虑CARD_IS_LOCDED,CURRENT_STATE,APP_CMD
  以及保留位）都为等于1，则返回SD_OK*/
  if ((response_r1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
  {
    return(errorstatus);
  }

  if (response_r1 & SD_OCR_ADDR_OUT_OF_RANGE)
  {
    return(SD_ADDR_OUT_OF_RANGE);
  }

  if (response_r1 & SD_OCR_ADDR_MISALIGNED)
  {
    return(SD_ADDR_MISALIGNED);
  }

  if (response_r1 & SD_OCR_BLOCK_LEN_ERR)
  {
    return(SD_BLOCK_LEN_ERR);
  }

  if (response_r1 & SD_OCR_ERASE_SEQ_ERR)
  {
    return(SD_ERASE_SEQ_ERR);
  }

  if (response_r1 & SD_OCR_BAD_ERASE_PARAM)
  {
    return(SD_BAD_ERASE_PARAM);
  }

  if (response_r1 & SD_OCR_WRITE_PROT_VIOLATION)
  {
    return(SD_WRITE_PROT_VIOLATION);
  }

  if (response_r1 & SD_OCR_LOCK_UNLOCK_FAILED)
  {
    return(SD_LOCK_UNLOCK_FAILED);
  }

  if (response_r1 & SD_OCR_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  if (response_r1 & SD_OCR_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & SD_OCR_CARD_ECC_FAILED)
  {
    return(SD_CARD_ECC_FAILED);
  }

  if (response_r1 & SD_OCR_CC_ERROR)
  {
    return(SD_CC_ERROR);
  }

  if (response_r1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & SD_OCR_STREAM_READ_UNDERRUN)
  {
    return(SD_STREAM_READ_UNDERRUN);
  }

  if (response_r1 & SD_OCR_STREAM_WRITE_OVERRUN)
  {
    return(SD_STREAM_WRITE_OVERRUN);
  }

  if (response_r1 & SD_OCR_CID_CSD_OVERWRIETE)
  {
    return(SD_CID_CSD_OVERWRITE);
  }

  if (response_r1 & SD_OCR_WP_ERASE_SKIP)
  {
    return(SD_WP_ERASE_SKIP);
  }

  if (response_r1 & SD_OCR_CARD_ECC_DISABLED)
  {
    return(SD_CARD_ECC_DISABLED);
  }

  if (response_r1 & SD_OCR_ERASE_RESET)
  {
    return(SD_ERASE_RESET);
  }

  if (response_r1 & SD_OCR_AKE_SEQ_ERROR)
  {
    return(SD_AKE_SEQ_ERROR);
  }
  /*返回SD_OK*/
  return(errorstatus);
}

/**
  * @brief 检查响应R3（OCR）的错误条件
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp3Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }
  /*命令响应超时*/
  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /* 清除所有静态标志 */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);
  /*已收到命令响应R3，无论如何CRC校验是否成功，都返回SD_OK*/
  return(errorstatus);
}

/**
  * @brief 检查命令响应R2（CID或CSD寄存器）的错误条件
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp2Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }
  /*命令响应超时*/
  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /*已收到命令响应R2，但是CRC校验失败*/
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

/**
  * @brief  检查命令响应R6（RCA寄存器）的错误条件
  * @param  cmd: 已发送的命令索引.
  * @param  prca: 指向相对卡地址（RCA）的指针. 
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t response_r1;

  status = SDIO->STA;
  
  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }
  /*命令响应超时*/
  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /*已收到命令响应R6，但是CRC校验失败*/
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /* Check response received is of desired command */
  /*已收到命令响应R6，并且CRC校验成功*/
  /*检查收到的响应是不是该命令的，如果不是，返回非法命令错误*/
  if (SDIO_GetCommandResponse() != cmd)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /* We have received response, retrieve it.  */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);
  /* 如果响应R6中没有发生普通或未知错误(卡状态位19:ERROR)、非法命令错误(卡状态位22:ILLEGAL_COMMAND)
     以及CRC校验错误(卡状态位23:COM_CEC_ERROR)，则计算出RCA并返回SD_OK。*/
  if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
  {	 
  	/* 相对卡地址RCA位于RCA寄存器的高16位[31:16] */
    *prca = (uint16_t) (response_r1 >> 16);
    return(errorstatus);
  }
   /* 普通或未知错误(卡状态位19:ERROR) */
  if (response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }
   /* 非法命令错误(卡状态位22:ILLEGAL_COMMAND) */
  if (response_r1 & SD_R6_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }
  /* CRC校验错误(卡状态位23:COM_CEC_ERROR) */
  if (response_r1 & SD_R6_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }
  /* 返回SD_OK */
  return(errorstatus);
}

/**
  * 功能描述  使能或失能SDIO宽总线模式
  * 输入参数  NewState: SDIO宽总线模式的新状态
  *    该参数可以是: ENABLE 或 DISABLE.
  * 返回值  SD_Error: SD卡错误代码
  */
static SD_Error SDEnWideBus(FunctionalState NewState)
{
  SD_Error errorstatus = SD_OK;

  uint32_t scr[2] = {0, 0};
  /* 确认卡未被上锁 */
  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  /* Get SCR Register */
   /* 获得 SCR 寄存器 */
  errorstatus = FindSCR(RCA, scr);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /* If wide bus operation to be enabled */
  /* 使能宽总线操作 */
  if (NewState == ENABLE)
  {
    /* If requested card supports wide bus operation */
	/* 如果卡支持宽总线操作，即SD_BUS_WIDTHS(SCR[51:48]) = 0100b */
    if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /* Send CMD55 APP_CMD with argument as card's RCA.*/
	  /* CMD55: APP_CMD ------------------------------------------------------------*/
      /* 必须在发送 ACMD6 之前发送 CMD55 通知卡，下面一条命令是应用特定命令 */
      /* 参数: - [31:16]: RCA 
               - [15:0]: 保留('0') */
      /* 响应类型: R1 */
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /* Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
	  /* ACMD6: SET_BUS_WIDTH ------------------------------------------------------*/
      /* 发送 ACMD6 设置总线宽度 */
      /* 参数: - [31:2]: 填充位('0') 
                - [1:0]: 总线宽度(‘00’=1位，‘10’=4位) */
      /* 响应类型: R1 */
      SDIO_CmdInitStructure.SDIO_Argument = 0x2;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }   
  /* If wide bus operation to be disabled */
  /* 失能宽总线操作 */
  else
  {
    /* If requested card supports 1 bit mode operation */
	 /* 如果卡支持1位总线操作，即SD_BUS_WIDTHS(SCR[51:48]) = 0001b */
    if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /* Send CMD55 APP_CMD with argument as card's RCA.*/
	  /* CMD55: APP_CMD ------------------------------------------------------------*/
      /* 必须在发送 ACMD6 之前发送 CMD55 通知卡，下面一条命令是应用特定命令 */
      /* 参数: - [31:16]: RCA 
               - [15:0]: 保留('0') */
      /* 响应类型: R1 */
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(SDIO_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /* Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
	  /* ACMD6: SET_BUS_WIDTH ------------------------------------------------------*/
      /* 发送 ACMD6 设置总线宽度 */
      /* 参数: - [31:2]: 填充位('0') 
              - [1:0]: 总线宽度(‘00’=1位，‘10’=4位) */
      /* 响应类型: R1 */
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }
}

/**
  * 功能描述  返回SD卡状态CURRENT_STATE(用于检查SD卡是否处于编程状态)
  * 输入参数  pstatus: 指向 CURRENT_STATE 的指针
  * 返回值  SD_Error: SD卡错误代码
  */
static SD_Error IsCardProgramming(uint8_t *pstatus)
{
  SD_Error errorstatus = SD_OK;
  __IO uint32_t respR1 = 0, status = 0;
  /* CMD13: SEND_STATUS -------------------------------------------------------------*/
  /* 发送 CMD13 要求卡发送状态寄存器(这里指卡状态，如果该命令在紧随CMD55之后发送，则为SD状态) */
  /* 参数: - [31:16]: 相对卡地址RCA
            - [15:0]: 填充位(‘0’) */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  status = SDIO->STA;
  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }
  /* 命令响应超时 */
  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /* 已接收到响应，但CRC校验失败 */
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }
  /* 已接收到响应，并且CRC校验成功 -----------------------------------------------------*/
  /* 检查是不是该命令的响应 */
  status = (uint32_t)SDIO_GetCommandResponse();

  /* Check response received is of desired command */
  if (status != SDIO_SEND_STATUS)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }
  /* 清除所有静态标志 */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /* 从SDIO_RESP1寄存器中读取响应R1 */
  /* We have received response, retrieve it for analysis  */
  respR1 = SDIO_GetResponse(SDIO_RESP1);

  /* Find out card status */
  /* 解析 CURRENT_STATE(卡状态位[12:9]) */
  *pstatus = (uint8_t) ((respR1 >> 9) & 0x0000000F);

  if ((respR1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
  {
    return(errorstatus);
  }

  if (respR1 & SD_OCR_ADDR_OUT_OF_RANGE)
  {
    return(SD_ADDR_OUT_OF_RANGE);
  }

  if (respR1 & SD_OCR_ADDR_MISALIGNED)
  {
    return(SD_ADDR_MISALIGNED);
  }

  if (respR1 & SD_OCR_BLOCK_LEN_ERR)
  {
    return(SD_BLOCK_LEN_ERR);
  }

  if (respR1 & SD_OCR_ERASE_SEQ_ERR)
  {
    return(SD_ERASE_SEQ_ERR);
  }

  if (respR1 & SD_OCR_BAD_ERASE_PARAM)
  {
    return(SD_BAD_ERASE_PARAM);
  }

  if (respR1 & SD_OCR_WRITE_PROT_VIOLATION)
  {
    return(SD_WRITE_PROT_VIOLATION);
  }

  if (respR1 & SD_OCR_LOCK_UNLOCK_FAILED)
  {
    return(SD_LOCK_UNLOCK_FAILED);
  }

  if (respR1 & SD_OCR_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  if (respR1 & SD_OCR_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (respR1 & SD_OCR_CARD_ECC_FAILED)
  {
    return(SD_CARD_ECC_FAILED);
  }

  if (respR1 & SD_OCR_CC_ERROR)
  {
    return(SD_CC_ERROR);
  }

  if (respR1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (respR1 & SD_OCR_STREAM_READ_UNDERRUN)
  {
    return(SD_STREAM_READ_UNDERRUN);
  }

  if (respR1 & SD_OCR_STREAM_WRITE_OVERRUN)
  {
    return(SD_STREAM_WRITE_OVERRUN);
  }

  if (respR1 & SD_OCR_CID_CSD_OVERWRIETE)
  {
    return(SD_CID_CSD_OVERWRITE);
  }

  if (respR1 & SD_OCR_WP_ERASE_SKIP)
  {
    return(SD_WP_ERASE_SKIP);
  }

  if (respR1 & SD_OCR_CARD_ECC_DISABLED)
  {
    return(SD_CARD_ECC_DISABLED);
  }

  if (respR1 & SD_OCR_ERASE_RESET)
  {
    return(SD_ERASE_RESET);
  }

  if (respR1 & SD_OCR_AKE_SEQ_ERROR)
  {
    return(SD_AKE_SEQ_ERROR);
  }

  return(errorstatus);
}

/**
  * 功能描述  返回SCR寄存器(64b，8B)
  * 输入参数  rca: 被选择的卡的RCA
  * 输入参数  pscr: 指向SCR数据的指针
  * 返回值  SD_Error: SD卡错误代码
  */
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr)
{
  uint32_t index = 0;
  SD_Error errorstatus = SD_OK;
  uint32_t tempscr[2] = {0, 0};

  /* Set Block Size To 8 Bytes */
  /* Send CMD55 APP_CMD with argument as card's RCA */
  /* 由于SCR是通过数据线发送的，因此要设置卡和主机的数据块长度 */
  /* CMD16: SET_BLOCKLEN -------------------------------------------------------*/
  /* 发送 CMD16 设置卡的数据块长度 */
  /* 参数: - [31:0]: 数据块长度 */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)8;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /* Send CMD55 APP_CMD with argument as card's RCA */
  /* CMD55: APP_CMD ------------------------------------------------------------*/
  /* 必须在发送 ACMD51 之前发送 CMD55 通知卡，下面一条命令是应用特定命令 */
  /* 参数: - [31:16]: 相对卡地址RCA
           - [15:0]: 保留('0') */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_APP_CMD);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
  /* 主机的数据块长度在初始化DPSM时设置 */
  /* 初始化并使能SDIO数据通道状态机(DPSM) */
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 8;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);


  /* Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
  /* ACMD51: SEND_SCR ----------------------------------------------------------*/
  /* 发送 ACMD51 要求卡发送SCR寄存器*/
  /* 参数: - [31:0]: 填充位('0') */
  /* 响应类型: R1 */
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_APP_SEND_SCR;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SD_APP_SEND_SCR);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  while (!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
  {
    /* 如果接收FIFO中还有未读出的有效数据，则将其读出 */
	if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
    {
	  /* 从接收FIFO中读出1个字 */
      *(tempscr + index) = SDIO_ReadData();
      index++;
    }
  }

  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }

  /* 清除所有静态标志 */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);
   /* SCR寄存器的数据包格式属于宽幅数据，先接收的是高字节 */
  *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

  *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

  return(errorstatus);
}

/**
  * 功能描述  将字节数换算成2的次幂，并返回
  * 输入参数  NumberOfBytes: 字节数
  * 返回值  无
  */
static uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes)
{
  uint8_t count = 0;

  while (NumberOfBytes != 1)
  {
    NumberOfBytes >>= 1;
    count++;
  }
  return(count);
}



/**
  * @brief  Configures the SDIO Corresponding GPIO Ports
  * @param  None
  * @retval : None
  */
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOC and GPIOD Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

  /* Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Tx request.
  * @param  BufferSRC: pointer to the source buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
static void DMA_TxConfiguration(uint32_t *BufferSRC, uint32_t BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

  /* DMA2 Channel4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);

  /* DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferSRC;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /* DMA2 Channel4 enable */
  DMA_Cmd(DMA2_Channel4, ENABLE);
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Rx request.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
static void DMA_RxConfiguration(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

  /* DMA2 Channel4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);

  /* DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /* DMA2 Channel4 enable */
  DMA_Cmd(DMA2_Channel4, ENABLE);
}


SD_Error Test_SDCard(void)
{
	/* 宏定义 */

	
	SD_CardInfo qq;
	SD_Error Status = SD_OK;
	Status = SD_Init();   
  	if (Status == SD_OK)
  	{		  	
  	 	//----------------- Read CSD/CID MSD registers ------------------	 
   		//Usart1_PutStr( " \r\n SD_Init is ok " );
	 	Status = SD_GetCardInfo(&qq);	 	   
  	}								
  	if (Status == SD_OK)
  	{																	 
	  	//Usart1_PutStr("\r\n SD_GetCardInfo is ok ");
    	Status = SD_SelectDeselect((uint32_t) (qq.RCA << 16));  
  	}
	 
  	if (Status == SD_OK)
  	{
  		//Usart1_PutStr(" \r\n SD_SelectDeselect is ok  ");
		// SDIO_BusWide_4b 会进入死循环,至今还未解决			   
    	Status = SD_EnableWideBusOperation(SDIO_BusWide_4b); 		  
  	}	 
	
  	if (Status == SD_OK)
  	{ 
	    Usart1_PutStr(" \r\n SDIO_BusWide_4b is ok "); 
   	    Status = SD_SetDeviceMode(SD_DMA_MODE);	
	    //Usart1_PutStr(" \r\n 好消息：MicroSD卡读写测试成功 \r\n ");// 
  	}  
	return Status;
}




/*
 * 函数名：SD_NVIC_Configuration
 * 描述  ：SDIO 优先级配置为最高优先级。
 * 输入  ：无
 * 输出  ：无
 */
void SD_NVIC_Configuration (void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}






