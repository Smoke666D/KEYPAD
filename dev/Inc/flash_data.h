/*
 * flash_data.h
 *
 *  Created on: Dec 1, 2021
 *      Author: igor.dymov
 */


#include "main.h"

#define FLASH_DATA_ADR 0x800fC00
#define CODE_ADR      0x00
#define BITRATE_ADR   0x01
#define NODE_ID_ADR   0x02
#define NMT_STATE_ADR 0x03

#define REG_SIZE      4U

#define VALID_CODE    0x4D
#define FLASH_SIZE 	0x08007FFFU
#define APP_ADDRESS    	0x08008000U
/* USER CODE BEGIN EXPORTED_MACRO */
#define GET_SECTOR( adr )	 ( ( adr < 0x08003FFFU )?FLASH_SECTOR_0:( ( adr < 0x08007FFFU )?FLASH_SECTOR_1:( ( adr < 0x0800BFFFU )?FLASH_SECTOR_2:( ( adr < 0x0800FFFFU )?FLASH_SECTOR_3:( ( adr < 0x0801FFFFU )?FLASH_SECTOR_4:( ( adr < 0x0803FFFFU )?FLASH_SECTOR_5:( ( adr < 0x0805FFFFU )?FLASH_SECTOR_6:( ( adr < 0x0807FFFFU )?FLASH_SECTOR_7:( ( adr < 0x0809FFFFU )?FLASH_SECTOR_8:( ( adr < 0x080BFFFFU )?FLASH_SECTOR_9:( ( adr < 0x080DFFFFU )?FLASH_SECTOR_10:FLASH_SECTOR_11 ) ) ) ) ) ) ) ) ) ) )
/* USER CODE END EXPORTED_MACRO */

uint16_t vGetBitrate();
uint8_t vGetNodeId();
void vFDSetBitrate(uint8_t bitrate);
void vFDSetNodeID(uint8_t nodeid);
