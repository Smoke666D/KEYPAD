/*
 * flash_data.h
 *
 *  Created on: Dec 1, 2021
 *      Author: igor.dymov
 */


#include "main.h"

#define BITRATE_ADR 0x00
#define NODE_ID_ADR 0x01

uint16_t vGetBitrate();
uint8_t vGetNodeId();
