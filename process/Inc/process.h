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

#define SET_LED_ON_RED		0x01
#define SET_LED_ON_GREEN	0x02
#define SET_LED_ON_BLUE 	0x03
#define SET_LED_BLINK_RED	0x04
#define SET_LED_BLINK_GREEN	0x05
#define SET_LED_BLINK_BLUE 	0x06

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


typedef struct __packed
{
  unsigned char command;
  unsigned char data;
} xLEDEvent;

void vProcessTask( void * argument );
void vProceesInit( void );
void vCanOpen_Init(void *argument);
void vLedProcess(void *argument);
ODR_t OD_writeLED(OD_stream_t *stream, const void *buf,
                       OD_size_t count, OD_size_t *countWritten);
#endif /* INC_PROCESS_H_ */
