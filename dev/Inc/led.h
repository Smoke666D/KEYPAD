/*
 * led.h
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */

#ifndef INC_LED_H_
#define INC_LED_H_


#include "main.h"

#define  MAX_BRIGTH 0x3F
#define OFF 0x00
#define RED_COLOR 	1U
#define GREEN_COLOR 2U
#define BLUE_COLOR  3U

#define RED		     0x01
#define GREEN	     0x02
#define BLUE	     0x03
#define YELLOW       0x04
#define CYAN		 0x05
#define VIOLET		 0x06
#define WHITE		 0x07
#define AMBER        0x08
#define YELLOW_GREEN 0x09

void SetLedOn(uint8_t Color,uint8_t State);
void SetLedBlink(uint8_t Color,uint8_t State);
void SetBrigth(uint8_t brigth);
void vLedInit(TIM_HandleTypeDef * htim);
#endif /* INC_LED_H_ */
