/*
 * process.c
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */

#include "process.h"
#include "CANopen.h"
#include "OD.h"
#include "CO_driver_ST32F103.h"
#include "led.h"



static QueueHandle_t     pKeyboard        = NULL;
static KeyEvent          TempEvent        = { 0U };

static StaticQueue_t     xLedQueue;
static QueueHandle_t     pLedQueue;

uint8_t LedBuffer[ 16U * sizeof( xLEDEvent ) ] = { 0U };

/* Variables used for triggering TPDO, see simulation in app_programRt(). */
OD_extension_t OD_LED_data_extension = {
    .object = NULL,
    .read =  OD_readOriginal,
    .write = OD_writeLed
};

/* Variables used for triggering TPDO, see simulation in app_programRt(). */
OD_extension_t OD_KEY_extension = {
    .object = NULL,
    .read = OD_readOriginal,
    .write = NULL
};

uint8_t *OD_KEY_flagsPDO = NULL;
uint8_t *OD_LED_data_flagsPDO = NULL;

void vProceesInit( void)
{
	pLedQueue  = xQueueCreateStatic( 16U, sizeof( xLEDEvent ), LedBuffer, &xLedQueue );
	pKeyboard = pGetKeyboardQueue();
	OD_extension_init(OD_ENTRY_H2000_digitalInputModuleKeysStates,
	                      &OD_KEY_extension);


	OD_extension_init(OD_ENTRY_H2001_digitalOutputModuleLED_ON,
		                      &OD_LED_data_extension);
	OD_KEY_flagsPDO = OD_getFlagsPDO(OD_ENTRY_H2000_digitalInputModuleKeysStates);
	//OD_LED_data_flagsPDO = OD_getFlagsPDO(OD_ENTRY_H2000_digitalInputModuleKeysStates);
}

void vProcessTask( void * argument )
{
   uint8_t key_mask;
	for(;;)
	{
		//Обработка событий от клавиатуры
	    xQueueReceive( pKeyboard, &TempEvent,portMAX_DELAY );
		switch (TempEvent.KeyCode)
		{
			   case kl1_key:
				   key_mask = K1;
				   break;
			   case kl2_key:
				   key_mask = K2;
			   	   break;
			   case kl3_key:
				   key_mask = K3;
			   	   break;
			   case kl4_key:
				   key_mask = K4;
 			   	   break;
			   case kl5_key:
				   key_mask = K5;
   			   	   break;
			   case kl6_key:
				   key_mask = K6;
			  	   break;
			   case kl7_key:
				   key_mask = K7;
			   	   break;
			   case kl8_key:
				   key_mask = K8;
			   	   break;
			   default:
				   key_mask = 0U;
				   break;
		}
		if ( TempEvent.Status == MAKECODE ) {
		   OD_RAM.x2000_digitalInputModuleKeysStates[0] |= key_mask;
  		 }
		 else {
		   OD_RAM.x2000_digitalInputModuleKeysStates[0] &= ~key_mask;
		 }
		 OD_requestTPDO(OD_KEY_flagsPDO,1);
	}
}


void vLedProcess(void *argument)
{
	static xLEDEvent	 xLedEvent;
	for(;;)
	{

		xQueueReceive(pLedQueue, &xLedEvent,portMAX_DELAY );
		switch (xLedEvent.command)
		{
			case SET_LED_ON_RED:
				    SetLedOn(RED_COLOR ,xLedEvent.data);
					break;
			case SET_LED_ON_GREEN:
				    SetLedOn(GREEN_COLOR ,xLedEvent.data);
					break;
			case SET_LED_ON_BLUE:
				    SetLedOn(BLUE_COLOR ,xLedEvent.data);
					break;
			case SET_LED_BLINK_RED:
				    SetLedBlink(RED_COLOR ,xLedEvent.data);
				    break;
			case SET_LED_BLINK_GREEN:
				    SetLedBlink(GREEN_COLOR ,xLedEvent.data);
					break;
			case SET_LED_BLINK_BLUE:
				    SetLedBlink(BLUE_COLOR ,xLedEvent.data);
					break;
			default:
				break;
		}
	}
}



