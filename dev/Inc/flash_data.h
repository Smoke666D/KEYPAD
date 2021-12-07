/*
 * flash_data.h
 *
 *  Created on: Dec 1, 2021
 *      Author: igor.dymov
 */


#include "main.h"

#define FLASH_DATA_ADR 0x800fC00
#define CODE_ADR       		0x00
#define BITRATE_ADR    		0x01
#define NODE_ID_ADR    		0x02
#define NMT_STATE_ADR  	    0x03
#define DEF_LED_BRIGTH_ADR  0x04
#define DEF_BL_BRIGTH_ADR   0x05
#define DEF_BL_COLOR_ADR    0x06
#define NMT_START_MESSAGE   0x07

#define REG_SIZE      		8U

#define VALID_CODE    0x25
#define FLASH_SIZE 	0x08007FFFU
#define APP_ADDRESS    	0x08008000U


uint16_t vGetBitrate();
uint8_t vGetNodeId();
uint16_t vFDGetNMTState();
uint8_t vFDGetRegState(uint8_t adr);
void vFDSetRegState(uint8_t adr, uint8_t state);
uint8_t vFDGetStartMessage();
