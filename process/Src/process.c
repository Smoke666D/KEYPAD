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



static QueueHandle_t     pKeyboard        = NULL;
static KeyEvent          TempEvent        = { 0U };



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

			co_drv_mutex_lock();
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
			co_drv_mutex_unlock();
		}


	}
}






