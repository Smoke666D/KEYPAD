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
static uint8_t backligth_color = 0;
static uint8_t blink_count = 0;
static uint8_t RegBusyFlag = RESET;
static uint8_t led_brigth = OFF;
void DrvLedSetState(uint8_t * state);
TIM_HandleTypeDef * pwmtim;

void vLedInit(TIM_HandleTypeDef * htim)
{
	pwmtim = htim;
}


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
	if (backligch_brigth != OFF)
	{
		uint8_t brigth_color[3];
		backligch_brigth = brigth;
		SetBrigth(backligch_brigth );
		switch (backligth_color)
		{
		case  RED:
			brigth_color[0]=0xFF;
			brigth_color[1]=0x00;
			brigth_color[2]=0x00;
			break;
		case GREEN:
			brigth_color[0]=0x00;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			break;
		case BLUE:
			brigth_color[0]=0x00;
			brigth_color[1]=0x00;
			brigth_color[2]=0xFF;
			break;
		case YELLOW:
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			break;
		case CYAN:
			brigth_color[0]=0xFF;
			brigth_color[1]=0x00;
			brigth_color[2]=0xFF;
			break;
		case VIOLET:
			brigth_color[0]=0x00;
			brigth_color[1]=0xFF;
			brigth_color[2]=0xFF;
			break;
		case WHITE:
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0xFF;
			break;
		case  AMBER:
			break;
		case YELLOW_GREEN:
			break;
		}
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

	TIM_OC_InitTypeDef sConfigOC = {0};
	HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_3);
	if (brigth <= MAX_BRIGTH)
	{

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = (brigth/MAX_BRIGTH)*1000;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(pwmtim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
			Error_Handler();
		}
		if (HAL_TIM_PWM_ConfigChannel(pwmtim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		{
			Error_Handler();
		}
		if (HAL_TIM_PWM_ConfigChannel(pwmtim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_3);
	}

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
