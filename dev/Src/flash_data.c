/*
 * flash_data.c
 *
 *  Created on: Dec 1, 2021
 *      Author: igor.dymov
 */


#include "flash_data.h"

static uint8_t FisrtStart = 1;
static uint8_t SettingsREG[]={0x04,0x12};


uint16_t vGetBitrate()
{
	uint16_t data = 0;
	switch(SettingsREG[BITRATE_ADR])
	{
		case 0x00:
			data = 1000;
			break;
		case 0x02:
			data = 500;
			break;
		case 0x03:
			data = 250;
			break;
		case 0x04:
			data = 125;
			break;
		case 0x06:
			data =50;
			break;
		case 0x07:
			data = 20;
			break;
		default:
			data = 125;
			break;
	}
    return data;
}

uint8_t vGetNodeId()
{

	return SettingsREG[NODE_ID_ADR];
}


/*

uint16_t vFDGetRegister(uint8_t reg_adr)
{
	uint8_t data=04h;
	switch (reg_adr)
	{
		case NODE_ID:
			data = 0x012;
			break;
		case BOUND_RATE:
			switch(data)
			{
				case 0x00:
					data = 1000;
					break;
				case 0x02:
					data = 500;
					break;
				case 0x03:
					data = 250;
					break;
				case 0x04:
					data = 125;
					break;
				case 0x06:
					data =50;
					break;
				case 0x07:
					data = 20;
					break;
				default:
					data = 125;
					break;
			}
			break;

	}

	return data;


}

void vFDSetRegister()
{

}*/
