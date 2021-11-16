/*
 * process.c
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */

#include "process.h"
#include "CANopen.h"
#include "OD.h"




static QueueHandle_t     pKeyboard        = NULL;
static KeyEvent          TempEvent        = { 0U };


/* Local variables */
static CO_t* CO;
static uint32_t co_heap_used;
static CO_NMT_reset_cmd_t reset = CO_RESET_NOT;


void vCanOpenInit(void *argument)
{


}


void vCanOpen_Init(void *argument)
{

	for(;;)
	{


	}

}



void vCanSendMessage(uint8_t Adress, uint8_t *pMessage);

void vProceesInit( void)
{
	pKeyboard = pGetKeyboardQueue();

}

void vProcessTask( void * argument )
{

	for(;;)
	{
		// osDelay(1);
		//Обработка событий от клавиатуры
		if ( xQueueReceive( pKeyboard, &TempEvent, 0U ) == pdPASS )
		{
			switch (TempEvent.KeyCode)
			{
			   case kl1_key:
				   if ( TempEvent.Status == MAKECODE )
				   {
					   OD_RAM.x2000_digitalInputModuleKeysStates_sub0 |= K1;

				   }
				   else
					   OD_RAM.x2000_digitalInputModuleKeysStates_sub0 &= ~K1;
				   break;
			   case kl2_key:
			   	   if ( TempEvent.Status == MAKECODE )
			   		    OD_RAM.x2000_digitalInputModuleKeysStates_sub0 |= K2;
			 	   else
			 		    OD_RAM.x2000_digitalInputModuleKeysStates_sub0 &= ~K2;
			   	   break;
			   case kl3_key:
			        if ( TempEvent.Status == MAKECODE )
			        	OD_RAM.x2000_digitalInputModuleKeysStates_sub0 |= K3;
			   	   else
			   		    OD_RAM.x2000_digitalInputModuleKeysStates_sub0 &= ~K3;
			   	   break;
			   case kl4_key:
			   	   if ( TempEvent.Status == MAKECODE )
			   		    OD_RAM.x2000_digitalInputModuleKeysStates_sub0 |= K4;
			 	   else
			 		    OD_RAM.x2000_digitalInputModuleKeysStates_sub0 &= ~K4;
 			   	   break;
			   case kl5_key:
			   	   if ( TempEvent.Status == MAKECODE )
			   		   OD_RAM.x2000_digitalInputModuleKeysStates_sub0 |= K5;
			 	   else
			 		   OD_RAM.x2000_digitalInputModuleKeysStates_sub0 &= ~K5;
   			   	   break;
			   case kl6_key:
			       if ( TempEvent.Status == MAKECODE )
			    	   OD_RAM.x2000_digitalInputModuleKeysStates_sub0 |= K6;
			   	   else
			   		   OD_RAM.x2000_digitalInputModuleKeysStates_sub0 &= ~K6;
			  	   break;
			   case kl7_key:
				   if ( TempEvent.Status == MAKECODE )
					   OD_RAM.x2000_digitalInputModuleKeysStates_sub0 |= K7;
			 	   else
			 		   OD_RAM.x2000_digitalInputModuleKeysStates_sub0  &= ~K7;
			   	   break;
			   case kl8_key:
			   	   if ( TempEvent.Status == MAKECODE )
			   		  OD_RAM.x2000_digitalInputModuleKeysStates_sub0 |= K8;
			 	   else
			 		  OD_RAM.x2000_digitalInputModuleKeysStates_sub0 &= ~K8;
			   	   break;
			   default:
				   break;

			}

		}


	}
}

void vCanSendMessage(uint8_t Adress, uint8_t *pMessage)
{

	switch (Adress)
	{
		case keys_state_message:
			//HAL_CAN_AddTxMessage();
			break;

	}
}




