#ifndef __BSP_FLASH_H__
#define __BSP_FLASH_H__

#include "main.h"  
#include "stm32f4xx_hal.h"

void WriteFlashData(uint32_t WriteAddress, uint32_t *data, uint32_t word);
uint32_t STMFLASH_ReadWord(uint32_t faddr);

#endif

