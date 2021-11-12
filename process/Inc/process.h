/*
 * process.h
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */

#ifndef INC_PROCESS_H_
#define INC_PROCESS_H_

#include "main.h"
#include "keyboard.h"


#define REGISTER_COUNT  10

#define KEY_STATE_REGISTER  1U
#define K1   0x01
#define K2   0x02
#define K3   0x04
#define K4   0x08
#define K5   0x10
#define K6   0x20
#define K7   0x40
#define K8   0x80

#define keys_state_message 180U

void vProcessTask( void * argument );
void vProceesInit( void );

#endif /* INC_PROCESS_H_ */
