#include "stmflash.h"

u16 FlashBuffer[STM_SECTOR_SIZE / 2];   //�����2K�ֽ�




//��ȡָ����ַ��16λ����
u16  FlashReadHalfWord(u32 addr)
{
	return *(vu16 *)addr;
}


void  FlashRead(u32 addr,u16 *pbuffer,u16 num)   	
{
	u16 i;
	for(i = 0;i < num;i++)
	{
		*pbuffer++ = FlashReadHalfWord(addr);//��ȡ2���ֽ�.
		addr += 2;                           //ƫ��2���ֽ�.	
	}
}



void  FlashWriteNoCheck(u32 addr,u16 *pbuffer,u16 num)
{
	u16 i;
	for(i = 0;i < num;i++)
	{
		FLASH_ProgramHalfWord (addr,pbuffer[i]);
		addr += 2;	
	}
}



/******************************************************************************* 
 * ��������: FlashWrite(*)
 * ��������: ��������д
 * ��    ��: WLB
 * �������: u32 addr,u16 *pbuffer,u16 num
 * �������: 
 * �� �� ֵ: 
 * ����˵��: һҳ2K
 * �޸���ʷ: 
 *           1. 2015-1-31  WLB  Created
 *******************************************************************************/
void  FlashWrite(u32 addr,u16 *pbuffer,u16 num)
{
#if 1

    u16 num_sector =  (addr - STM32_FLASH_BASE) / STM_SECTOR_SIZE;	   //��ȡ�ڼ�ҳ

    FLASH_Unlock();
    FLASH_ErasePage (num_sector * STM_SECTOR_SIZE + STM32_FLASH_BASE);
    FlashWriteNoCheck(num_sector * STM_SECTOR_SIZE + STM32_FLASH_BASE, pbuffer, num / 2);
    FLASH_Lock();             //����
#else
    u16  num_sector;  //�ڵڼ�ҳ
	u16  offset_sector;  //��һҳ��ƫ����
	u16  num_last;    //ʣ�µİ���
	u16  i = 0;
    
	num_sector =  (addr - STM32_FLASH_BASE) / STM_SECTOR_SIZE;	   //��ȡ�ڼ�ҳ
	offset_sector =  ((addr - STM32_FLASH_BASE) % STM_SECTOR_SIZE) / 2;  //��ȡƫ������16���ֽ�Ϊ��λ
	num_last = 	STM_SECTOR_SIZE / 2 - offset_sector;                     //���ʣ�µİ���
	FLASH_Unlock();                              //����
	if(num <= num_last)
        num_last = num;		 //����һ��д��

    for(;;)										 //��ʼ��ͣ��д
	{
		//FlashRead(num_sector * STM_SECTOR_SIZE + STM32_FLASH_BASE, FlashBuffer, STM_SECTOR_SIZE / 2);  //����һҳ
		//for(i = 0;i < num_last;i++)
		//{
		//	if(FlashBuffer[offset_sector + i] != 0xffff)
        //       break;	
		//}

		if(i < num_last)						//��Ҫ����
		{
			FLASH_ErasePage (num_sector * STM_SECTOR_SIZE + STM32_FLASH_BASE);                      //����
			for(i = 0;i < num_last;i++)
			{
				FlashBuffer[offset_sector + i] = pbuffer[i];	               //��Ҫд���������������
			}
			FlashWriteNoCheck(num_sector * STM_SECTOR_SIZE + STM32_FLASH_BASE, FlashBuffer,STM_SECTOR_SIZE / 2);               //д��������		
		}
        else								    //����Ҫ����
		{
			FlashWriteNoCheck(addr,pbuffer,num_last);	//ֱ�ӽ�Ҫд�������д��flash
		}

		if(num_last == num)
            break;                     //д��������
		else                                            //û��д�����
		{
			num_sector += 1;    //ҳ��һ
			offset_sector = 0;  //ƫ��Ϊ0
			pbuffer += num_last;
			addr += num_last * 2;
			num -= num_last;    //����Ҫд�������
			if(num > (STM_SECTOR_SIZE / 2))
                num_last = STM_SECTOR_SIZE / 2;
			else
                num_last = num;   //����num_last		
		}		
	}
	FLASH_Lock();             //����
#endif
}













