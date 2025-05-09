#include "bsp_flash.h"

#define FMC_FLASH_BASE      0x08000000   // FLASH����ʼ��ַ
#define FMC_FLASH_END       0x08080000   // FLASH�Ľ�����ַ

#define FLASH_WAITETIME     50000        //FLASH�ȴ���ʱʱ��

//FLASH ��������ʼ��ַ
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//����0��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//����1��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//����2��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//����3��ʼ��ַ, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//����4��ʼ��ַ, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//����5��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//����6��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//����7��ʼ��ַ, 128 Kbytes  
//   stm32f407vetx   512k  ֻ��8������
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//����8��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//����9��ʼ��ַ, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//����10��ʼ��ַ,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//����11��ʼ��ַ,128 Kbytes 

//��ȡָ����ַ����(32λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
    return *(uint32_t*)faddr; 
}

//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
static uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
    if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
    else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
    else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
    else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
    else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
    else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
    else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
    else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
    else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
    else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
    else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10;   
    return FLASH_SECTOR_11;	
}

/**
 *@���ܣ����ڲ�Flashд������
 *@����1��WriteAddress������Ҫд���Ŀ���ַ��ƫ�Ƶ�ַ��
 *@����2��*data�� д��������׵�ַ
 *@����3��word�����ٸ���
 */
void WriteFlashData(uint32_t WriteAddress, uint32_t *data, uint32_t word)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    uint32_t SectorError=0;
    uint32_t addrx=0;
    uint32_t endaddr=0;
    
    if( (WriteAddress < FMC_FLASH_BASE) || ( WriteAddress + word*4 >= FMC_FLASH_END) || (word*4 <= 0) )
    return;

	__disable_irq();  // �ر�ȫ���ж�
	
    HAL_FLASH_Unlock();              //����
    addrx = WriteAddress;            //д�����ʼ��ַ
    endaddr = WriteAddress+word*4;   //д��Ľ�����ַ


        while(addrx<endaddr)  //ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
        {
             if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
            {   
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;       //�������ͣ��������� 
                FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx);   //Ҫ����������
                FlashEraseInit.NbSectors=1;                             //һ��ֻ����һ������
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //��ѹ��Χ��VCC=2.7~3.6V֮��!!
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
                {
                    break;//����������
                }
                }else addrx+=1;
                FLASH_WaitForLastOperation(FLASH_WAITETIME);
				//�ȴ��ϴβ������
        }
    
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME);            //�ȴ��ϴβ������
    if(FlashStatus==HAL_OK)
    {
         while(WriteAddress<endaddr)//д����
         {
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddress,*data)!=HAL_OK)//д������
            { 
                break;	//д���쳣
            }
            WriteAddress+=4;
            data++;
        }  
    }
    HAL_FLASH_Lock();           //����
	__enable_irq();  // �ָ�ȫ���ж�
}
