/*
 * process.h
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */

#ifndef INC_PROCESS_H_
#define INC_PROCESS_H_

#include "main.h"
#include "CO_ODinterface.h"
#include "keyboard.h"


#define NMT_CONTROL                     (CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION)
#define FIRST_HB_TIME                   500
#define SDO_SRV_TIMEOUT_TIME            1000
#define SDO_CLI_TIMEOUT_TIME            500
#define SDO_CLI_BLOCK                   false
#define OD_STATUS_BITS                  NULL




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
void vCanOpen_Init(void *argument);
ODR_t OD_writeLED(OD_stream_t *stream, const void *buf,
                       OD_size_t count, OD_size_t *countWritten);
#endif /* INC_PROCESS_H_ */
