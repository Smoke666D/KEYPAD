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
#include "CO_ODinterface.h"

#define REGISTER_COUNT   10U
#define REGISTER_KL		 0U

static uint8_t LocalRegiste[REGISTER_COUNT] ={0,0,0,0,0,0,0,0};
static QueueHandle_t     pKeyboard        = NULL;
static KeyEvent          TempEvent        = { 0U };


/* Variables used for triggering TPDO, see simulation in app_programRt(). */
OD_extension_t OD_variableInt32_extension = {
    .object = NULL,
    .read = OD_readOriginal,
    .write = OD_writeOriginal
};
uint8_t *OD_variableInt32_flagsPDO = NULL;

void vProceesInit( void)
{
	pKeyboard = pGetKeyboardQueue();
	OD_extension_init(OD_ENTRY_H2000_digitalInputModuleKeysStates,
	                      &OD_variableInt32_extension);
	OD_variableInt32_flagsPDO = OD_getFlagsPDO(OD_ENTRY_H2000_digitalInputModuleKeysStates);
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
				   if ( TempEvent.Status == MAKECODE ) {
					   LocalRegiste[REGISTER_KL] |= K1;
				   }
				   else {
					   LocalRegiste[REGISTER_KL] &= ~K1;
				   }
				   break;
			   case kl2_key:
			   	   if ( TempEvent.Status == MAKECODE ) {
			   		    LocalRegiste[REGISTER_KL] |= K2;
			   	   }
			 	   else {
			 		  LocalRegiste[REGISTER_KL] &= ~K2;
			 	   }
			   	   break;
			   case kl3_key:
			        if ( TempEvent.Status == MAKECODE ) {
			        	LocalRegiste[REGISTER_KL] |= K3;
			        }
			   	   else {
			   		   LocalRegiste[REGISTER_KL] &= ~K3;
			   	   }
			   	   break;
			   case kl4_key:
			   	   if ( TempEvent.Status == MAKECODE ){
			   		   LocalRegiste[REGISTER_KL] |= K4;
			   	   }
			 	   else {
			 		  LocalRegiste[REGISTER_KL] &= ~K4;
			 	   }
 			   	   break;
			   case kl5_key:
			   	   if ( TempEvent.Status == MAKECODE ) {
			   		LocalRegiste[REGISTER_KL] |= K5;
					}
			 	   else {
			 		  LocalRegiste[REGISTER_KL] &= ~K5;
					}
   			   	   break;
			   case kl6_key:
			       if ( TempEvent.Status == MAKECODE ) {
			    	   LocalRegiste[REGISTER_KL] |= K6;
					}
			   	   else {
			   		   LocalRegiste[REGISTER_KL] &= ~K6;
				   }
			  	   break;
			   case kl7_key:
				   if ( TempEvent.Status == MAKECODE ) {
					   LocalRegiste[REGISTER_KL] |= K7;
				   }
			 	   else {
			 		  LocalRegiste[REGISTER_KL]  &= ~K7;
					}
			   	   break;
			   case kl8_key:
			   	   if ( TempEvent.Status == MAKECODE ) {
			   		LocalRegiste[REGISTER_KL] |= K8;
			   	   }
			 	   else {
			 		  LocalRegiste[REGISTER_KL] &= ~K8;
				   }
			   	   break;
			   default:
				   break;

			}

			if  (LocalRegiste[REGISTER_KL] != OD_RAM.x2000_digitalInputModuleKeysStates[0])
			{
				OD_RAM.x2000_digitalInputModuleKeysStates[0] = LocalRegiste[REGISTER_KL];
				OD_requestTPDO(OD_variableInt32_flagsPDO,1);
			}
		}


	}
}






