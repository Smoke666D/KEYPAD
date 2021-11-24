/*
 * led.c
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */
#include "led.h"

static uint8_t LED_ON[3] = {0,0,0};
static uint8_t LED_BLINK[3] = {0,0,0};
static uint8_t LedBlinkState = RESET;
static uint8_t backligch_brigth = OFF;
static uint8_t blink_count = 0;
static uint8_t RegBusyFlag = RESET;
static uint8_t led_brigth = OFF;
void DrvLedSetState(uint8_t * state);




void SetLedOn(uint8_t Color,uint8_t State)
{
	if ((Color >=RED_COLOR) && (Color <=BLUE_COLOR)) {
		RegBusyFlag = SET;
		LED_ON[Color-1] = State;
		RegBusyFlag = RESET;
	}
	if ((backligch_brigth == OFF) && (LedBlinkState == OFF))
	{
		DrvLedSetState(&LED_ON[0]);
	}

}
void SetLedBlink(uint8_t Color,uint8_t State)
{
	if ((Color >=RED_COLOR) && (Color <=BLUE_COLOR)) {
		RegBusyFlag = SET;
		LED_BLINK[Color-1] = State;
		RegBusyFlag = RESET;
	}
}

void SetBackLigth(uint8_t brigth)
{
	backligch_brigth = brigth;
	if (backligch_brigth ! = OFF)
	{
		uint8_t brigth_color[3];
		backligch_brigth = brigth;
		SetBrigth(backligch_brigth );
		DrvLedSetState(&brigth_color[0]);
	}
	else
	{
		SetBrigth(led_brigth);
		DrvLedSetState(&LED_ON[0]);
	}

}


void DrvLedSetState(uint8_t * state)
{

}

void SetBrigth(uint8_t brigth)
{


}

uint8_t leds[3];
void BlinkProcess()
{
	if ((RegBusyFlag == RESET) && (LedBlinkState = SET) && (backligch_brigth == OFF))
	{
		switch (blink_count)
		{
		  case 0:
			 for (uint8_t i= 0;i<3;i++)
			 {
			 	leds[i] = LED_ON[i] & ~(LED_BLINK[i] & LED_ON[i]);
			 }
			 DrvLedSetState(&leds[0]);
			 blink_count = 1;
			 break;
		  case 1:
			 for (uint8_t i= 0;i<3;i++)
			 {
				 leds[i] = LED_ON[i] | (LED_BLINK[i] & LED_ON[i]);
			  }
			  DrvLedSetState(&leds[0]);
			  blink_count = 0;
			  break;
		}
	}
}
