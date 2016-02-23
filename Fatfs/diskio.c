/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "diskio.h"
#include "sdio.h"


/*-----------------------------------------------------------------------*/
/* Correspondence between physical drive number and physical drive.      */

//#define ATA		0
//#define MMC		1
//#define USB		2

extern SD_Error SD_USER_Init(void);
extern SD_CardInfo SDCardInfo;
#define SECTOR_SIZE 512U
/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
	SD_Error Status;

	switch ( drv )	
	{
		case 0 :
			Status = Test_SDCard(); /* SD_USER_Init() ����������û���д */
			if ( Status == SD_OK )	 /* SD ����ʼ���ɹ�,��һ���ǳ���Ҫ */
				return 0;
			else
			return STA_NOINIT;

		case 1 :
			return STA_NOINIT;

		case 2 :
			return STA_NOINIT;
	}
	return STA_NOINIT;

/* ����ע�͵��Ĳ���ΪFATFS�Դ��� */
//	DSTATUS stat;
//	int result;
//
//	switch (drv) {
//	case ATA :
//		//result = ATA_disk_initialize();
//		// translate the reslut code here
//
//		return stat;
//
//	case MMC :
//		result = MMC_disk_initialize();
//		// translate the reslut code here
//
//		return stat;
//
//	case USB :
//		result = USB_disk_initialize();
//		// translate the reslut code here
//
//		return stat;
//	}
//	return STA_NOINIT;	
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status   ��ȡ��ǰ���̵�״̬                                                 */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{
	switch (drv) /* �û��Լ����Ӧ�ô��� */
	{
	  case 0 :
		
	  /* translate the reslut code here	*/

	    return 0;

	  case 1 :
	
	  /* translate the reslut code here	*/

	    return 0;

	  case 2 :
	
	  /* translate the reslut code here	*/

	    return 0;

	  default:

        break;
	}
	return STA_NOINIT;

/* ����ע�͵��Ĳ���ΪFATFS�Դ��� */
//	DSTATUS stat;
//	int result;
//
//	switch (drv) {
//	case ATA :
//		//result = ATA_disk_status();
//		// translate the reslut code here
//
//		return stat;
//
//	case MMC :
//		result = MMC_disk_status();
//		// translate the reslut code here
//
//		return stat;
//
//	case USB :
//		result = USB_disk_status();
//		// translate the reslut code here
//
//		return stat;
//	}
//	return STA_NOINIT;
	
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)    �Ӵ����������ж�ȡ����                                                    */

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{	
	if ( count == 1 )		/* 1��sector�Ķ����� */
  {	
  	SD_ReadBlock( sector << 9, (u32 *)(&buff[0]), SECTOR_SIZE );
		//SD_ReadBlock( sector << 9, (u32 *)(&buff[0]), SDCardInfo.CardBlockSize );
		
	}
	else        			 /* ���sector�Ķ����� */
	{
		SD_ReadMultiBlocks( sector << 9, (u32 *)(&buff[0]), SECTOR_SIZE, count );
		
	}
	return RES_OK;		 

/* ����ע�͵��Ĳ���ΪFATFS�Դ��� */
//	DRESULT res;
//	int result;
//
//	switch (drv) {
//	case ATA :
//		//result = ATA_disk_read(buff, sector, count);
//		// translate the reslut code here
//
//		return res;
//
//	case MMC :
//		result = MMC_disk_read(buff, sector, count);
//		// translate the reslut code here
//
//		return res;
//
//	case USB :
//		result = USB_disk_read(buff, sector, count);
//		// translate the reslut code here
//
//		return res;
//	}
//	return RES_PARERR;	
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)        �������������д������                                               */

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{	
	if ( count == 1 )		/* 1��sector��д���� */
  {		
		SD_WriteBlock(sector << 9 ,(u32 *)(&buff[0]),SECTOR_SIZE);
	}
	else							 /* ���sector��д���� */
  {		
    SD_WriteMultiBlocks(sector << 9 ,(u32 *)(&buff[0]),SECTOR_SIZE,count);
	}        
  return RES_OK;	

/* ����ע�͵��Ĳ���ΪFATFS�Դ��� */
//	DRESULT res;
//	int result;
//
//	switch (drv) {
//	//case ATA :
//		result = ATA_disk_write(buff, sector, count);
//		// translate the reslut code here
//
//		return res;
//
//	case MMC :
//		result = MMC_disk_write(buff, sector, count);
//		// translate the reslut code here
//
//		return res;
//
//	case USB :
//		result = USB_disk_write(buff, sector, count);
//		// translate the reslut code here
//
//		return res;
//	}
//	return RES_PARERR; 	
}
#endif /* _READONLY 		�����豸�ض��Ĺ����Լ����̶�д�������������  */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	return RES_OK;	// 0
/* ����ע�͵��Ĳ���ΪFATFS�Դ��� */
//	DRESULT res;
//	int result;
//
//	switch (drv) {
//	case ATA :
//		// pre-process here
//
//		//result = ATA_disk_ioctl(ctrl, buff);
//		// post-process here
//
//		return res;
//
//	case MMC :
//		// pre-process here
//
//		result = MMC_disk_ioctl(ctrl, buff);
//		// post-process here
//
//		return res;
//
//	case USB :
//		// pre-process here
//
//		result = USB_disk_ioctl(ctrl, buff);
//		// post-process here
//
//		return res;
//	}
//	return RES_PARERR;	
}

/* �õ��ļ�Calendar��ʽ�Ľ�������,��DWORD get_fattime (void) ��任 */							
/*-----------------------------------------------------------------------*/
/* User defined function to give a current time to fatfs module          */
/* 31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
/* 15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{   
	return 0;
}
