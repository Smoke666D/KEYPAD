/*
 * led.h
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */

#ifndef INC_LED_H_
#define INC_LED_H_


#include "main.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "flash_data.h"

#define FLAT_VERSION

#define  MAX_BRIGTH 0x3F
#define OFF 0x00

#ifdef FLAT_VERSION
	#define RED_COLOR 	1U
	#define GREEN_COLOR 2U
	#define BLUE_COLOR  3U
#else
	#define RED_COLOR 	3U
	#define GREEN_COLOR 2U
	#define BLUE_COLOR  1U
#endif

#define RED		     0x01
#define GREEN	     0x02
#define BLUE	     0x03
#define YELLOW       0x04
#define CYAN		 0x05
#define VIOLET		 0x06
#define WHITE		 0x07
#define AMBER        0x08
#define YELLOW_GREEN 0x09


#define SET_LED_ON_RED		0x01
#define SET_LED_ON_GREEN	0x02
#define SET_LED_ON_BLUE 	0x03
#define SET_LED_BLINK_RED	0x04
#define SET_LED_BLINK_GREEN	0x05
#define SET_LED_BLINK_BLUE 	0x06

#define DISABLE    0x00
#define FULL_SHOW  0x01
#define FLASH_SHOW 0x02


typedef struct __packed
{
  unsigned char command;
  unsigned char data;
} xLEDEvent;
void vLedDriverStart(void);
void SetBackLigthColor(uint8_t color);
void SetLedBrigth(uint8_t brigth);
void SetLedOn(uint8_t Color,uint8_t State);
void SetLedBlink(uint8_t Color,uint8_t State);
void SetBrigth(uint8_t brigth);
#ifdef  FLAT_VERSION
void vLedInit(TIM_HandleTypeDef * htim, TIM_HandleTypeDef * dtim, SemaphoreHandle_t temp, SPI_HandleTypeDef* spi, TIM_HandleTypeDef * btim );
#else
void vLedInit(TIM_HandleTypeDef * htim, TIM_HandleTypeDef * dtim, SemaphoreHandle_t temp, SPI_HandleTypeDef* spi );
#endif

void vLedProcess(void *argument);
void SetBackLigth(uint8_t brigth);
void vSTPDealyInterrupt();
void vLatch( void );
void vSPTuSDealy(uint16_t Delay);
void StartLEDShow(uint8_t show_type);
#endif /* INC_LED_H_ */
