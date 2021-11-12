/*
 * keyboard.h
 *
 *  Created on: 25 февр. 2020 г.
 *      Author: igor.dymov
 */

#ifndef INC_KEYBOARD_H_
#define INC_KEYBOARD_H_
/*----------------------- Includes -------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

/*------------------------ Define --------------------------------------*/
/* Константа определяющая количесвто клавиш, которые обрабатываем драйвер */
#define KEYBOARD_COUNT     8U
#define KEY_READY	         0x0001U
#define KEY_ON_MESSAGE     0x0001U
#define KEY_OFF_MESSAGE    0x0002U
#define KEY_ON_STATE       ( GPIO_PIN_RESET )
#define KEY_OFF_STATE      ( GPIO_PIN_SET )
#define KEY_OFF            0x00U
#define KEY_ON    	       0X01U
#define KEY_ON_REPEAT      0x02U
#define KEY_PEREOD         15U
#define SWITCHONDELAY      30U
#define DefaultDelay       600U
#define DefaultRepeatRate  400U
#define BRAKECODE          0x40U
#define MAKECODE           0x80U
#define LINE               4U
#define ROW                5U
#define up_key             0x01U
#define down_key           0x02U
#define stop_key           0x04U
#define start_key          0x08U
#define auto_key           0x10U
#define time_out           0x20U

#define USE_KEY_TIME_OUT       0

#define  KEY_TIME_OUT      1000U
/*---------------------------- Structures --------------------------------------*/
typedef struct __packed
{
  GPIO_TypeDef* KeyPort;
  uint16_t      KeyPin;
} xKeyPortStruct;
typedef struct __packed
{
  unsigned int  KeyCode;
  unsigned char Status;
} KeyEvent;
typedef struct __packed
{
  unsigned int  sKeyCode;
  unsigned char cStatus;
} xKeyEvent;
/*----------------------------- Functions ------------------------------------*/
void          vKeyboardTask( void  * argument );//  __attribute__((optimize("-O3")));
void          vSetupKeyboard( void );
void          vKeyboardInit( uint32_t Message );
QueueHandle_t pGetKeyboardQueue( void );
/*----------------------------------------------------------------------------*/
#endif /* INC_KEYBOARD_H_ */
