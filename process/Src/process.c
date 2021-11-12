/*
 * process.c
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */

#include "process.h"


static uint8_t REGISTER[REGISTER_COUNT] = {0,0,0,0,0,0,0};


static QueueHandle_t     pKeyboard        = NULL;
static KeyEvent          TempEvent        = { 0U };


void vCanSendMessage(uint8_t Adress, uint8_t *pMessage);

void vProceesInit( void)
{
	pKeyboard = pGetKeyboardQueue();

}

void vProcessTask( void * argument )
{

	for(;;)
	{
		//Обработка событий от клавиатуры
		if ( xQueueReceive( pKeyboard, &TempEvent, 0U ) == pdPASS )
		{
			switch (TempEvent.KeyCode)
			{
			   case kl1_key:
				   if ( TempEvent.Status == MAKECODE )
					   REGISTER[KEY_STATE_REGISTER] |= K1;
				   else
					   REGISTER[KEY_STATE_REGISTER] &= ~K1;
				   break;
			   case kl2_key:
			   	   if ( TempEvent.Status == MAKECODE )
		     	        REGISTER[KEY_STATE_REGISTER] |= K2;
			 	   else
		 	 		    REGISTER[KEY_STATE_REGISTER] &= ~K2;
			   	   break;
			   case kl3_key:
			        if ( TempEvent.Status == MAKECODE )
			   		    REGISTER[KEY_STATE_REGISTER] |= K3;
			   	   else
			  		    REGISTER[KEY_STATE_REGISTER] &= ~K3;
			   	   break;
			   case kl4_key:
			   	   if ( TempEvent.Status == MAKECODE )
			   	        REGISTER[KEY_STATE_REGISTER] |= K4;
			 	   else
		 	 		    REGISTER[KEY_STATE_REGISTER] &= ~K4;
 			   	   break;
			   case kl5_key:
			   	   if ( TempEvent.Status == MAKECODE )
			   	        REGISTER[KEY_STATE_REGISTER] |= K5;
			 	   else
   		 	 		    REGISTER[KEY_STATE_REGISTER] &= ~K5;
   			   	   break;
			   case kl6_key:
			       if ( TempEvent.Status == MAKECODE )
			   		    REGISTER[KEY_STATE_REGISTER] |= K6;
			   	   else
			 		    REGISTER[KEY_STATE_REGISTER] &= ~K6;
			  	   break;
			   case kl7_key:
				   if ( TempEvent.Status == MAKECODE )
			   	        REGISTER[KEY_STATE_REGISTER] |= K7;
			 	   else
			 		    REGISTER[KEY_STATE_REGISTER] &= ~K7;
			   	   break;
			   case kl8_key:
			   	   if ( TempEvent.Status == MAKECODE )
			   	        REGISTER[KEY_STATE_REGISTER] |= K8;
			 	   else
			 		    REGISTER[KEY_STATE_REGISTER] &= ~K8;
			   	   break;
			   default:
				   break;

			}

		}
		vCanSendMessage(keys_state_message,&REGISTER[KEY_STATE_REGISTER]);

	}
}

void vCanSendMessage(uint8_t Adress, uint8_t *pMessage)
{

	switch (Adress)
	{
		case keys_state_message:
			HAL_CAN_AddTxMessage();
			break;

	}
}




