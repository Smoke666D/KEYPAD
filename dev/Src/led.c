/*
 * led.c
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */
#include "led.h"




uint8_t LED_ON[3] 		=         { 0x00 , 0x00 , 0x00 };
uint8_t LED_BLINK[3] 	=         { 0x00 , 0x00 , 0x00 };

static uint8_t backligch_brigth = OFF;
static uint8_t backligth_color 	= 0U;
static uint8_t blink_count 		= 0U;
static uint8_t RegBusyFlag 		= RESET;
static uint8_t led_brigth 		= OFF;
static uint8_t led_show_enable  = OFF;
static uint8_t startup_backligth =0x3F;
static SPI_HandleTypeDef* LEDSpi         = NULL;
#ifdef  FLAT_VERSION
static uint8_t brigth_color[3]=  { 0x00 , 0x00 , 0x00 };
static TIM_HandleTypeDef * backpwmtim		 = NULL;
#else
enum KEY_FSM
{
	BACKLIGTH,
	LED,
};
enum KEY_FSM KEYPAD_STATE = BACKLIGTH;
#endif
static TIM_HandleTypeDef * pwmtim		 = NULL;
static TIM_HandleTypeDef * delaytim      = NULL;
static SemaphoreHandle_t  xSemaphore = NULL;
void SetBackBrigth(uint8_t brigth);
void BackBrigthON();
void BackBrigthOFF();
uint8_t vSTPErrorDetection();
void vSTPNormalMode();
HAL_StatusTypeDef SPI_Transmit_DMA (uint8_t *pData, uint16_t size );
void DrvLedSetState(uint8_t * state);

static uint16_t us_delay=0;
static uint16_t us_counter = 0;




void DrvLedSetState(uint8_t * state)
{
	uint8_t buf[3];
	buf[0]=state[2];
	buf[1]=state[1];
	buf[2]=state[0];
	HAL_SPI_Transmit(LEDSpi,&buf[0],3,100);
	vLatch();
}


void vSTPDealyInterrupt()
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	us_counter++;
	if  (us_counter>= us_delay)
	{
	   us_counter = 0;
	   xHigherPriorityTaskWoken = pdFALSE;
	   xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );
	   portEND_SWITCHING_ISR( xHigherPriorityTaskWoken )
	}
}


void vSPTuSDealy(uint16_t Delay)
{
	us_delay = Delay;
	HAL_TIM_Base_Start_IT(delaytim);
	xSemaphoreTake( xSemaphore, 1 );
	HAL_TIM_Base_Stop_IT(delaytim);
}

void vLatch()
{
	//osDelay(1);//vSPTuSDealy(1);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	//osDelay(1);//vSPTuSDealy(1);
	vSPTuSDealy(100);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	//osDelay(1);//vSPTuSDealy(1);
	vSPTuSDealy(100);
}

uint8_t vSTPErrorDetection()
{
	 uint8_t res = 1;
	// Входим в режим Detectiom

	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET); //OE High
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);  //LE low
	 for (uint8_t i=0;i<(5+8*3);i++)
	 {

		 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
		 vSPTuSDealy(1);
		 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		 switch (i)
		 {
		    case 0:
		    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET); //OE Low
		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		    	break;
		    case 1:
		    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET); //OE High
		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		    	break;
		    case 2:
		    	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);  //LE High
		    	break;
		    case 3:
		    	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);  //LE low
		    	break;
		    case 4:
		    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		    	break;
		    default:
		    	break;
		 }
	 }
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	 vLatch();
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET); //OE Low
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	 vSPTuSDealy(1);
	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	 vSPTuSDealy(1);
	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET); //OE Low
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	 uint32_t buf=0;
	 for (uint8_t i =0;i<(3*8);i++)
	 {
		 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
		 vSPTuSDealy(1);
		 if ( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_SET )
         {
			 buf=buf<<1;
			 buf |= 0x0001;
         }
		 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		 vSPTuSDealy(1);
	 }
	 if (buf == 0x0FFFFFF) {
		 res = 0;
	 }
	 return res;
}


void vSTPNormalMode()
{
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET); //OE High
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);  //LE low
	 for (uint8_t i=0;i<5;i++)
	 {

	    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	    vSPTuSDealy(1);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		switch (i)
		{
		    case 1:
		    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET); //OE Low
		    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		    	break;
		    case 2:
		    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET); //OE High
		    	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		    	break;
		    default:
		    	break;
		 }
	 }
}

#ifdef  FLAT_VERSION
void vLedInit(TIM_HandleTypeDef * htim, TIM_HandleTypeDef * dtim, SemaphoreHandle_t temp, SPI_HandleTypeDef* spi, TIM_HandleTypeDef * btim )
#else
void vLedInit(TIM_HandleTypeDef * htim, TIM_HandleTypeDef * dtim, SemaphoreHandle_t temp, SPI_HandleTypeDef* spi )
#endif
{
#ifdef  FLAT_VERSION
    backpwmtim = btim;
#endif
	pwmtim = htim;
	delaytim = dtim;
	LEDSpi = spi;
	xSemaphore = temp;
	backligth_color  = vFDGetRegState(DEF_BL_COLOR_ADR);
	led_brigth 		 = vFDGetRegState(DEF_LED_BRIGTH_ADR);
	backligch_brigth = vFDGetRegState(DEF_BL_BRIGTH_ADR);
	led_show_enable = vFDGetRegState(LED_SHOW_ADRRES);


}


void vLedDriverStart(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//Выключем SPI и переиницилизируем порты на GPIO
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Speed =  GPIO_SPEED_FREQ_HIGH ;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Speed =  GPIO_SPEED_FREQ_HIGH ;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	if (vSTPErrorDetection() == 0)
	{
		vSTPNormalMode();
	}

	SetBackLigth(0);
	DrvLedSetState(&LED_ON[0]);
	HAL_TIM_MspPostInit(pwmtim);

	if (led_show_enable!=DISABLE)
    {
	 	StartLEDShow(led_show_enable);
		led_show_enable = DISABLE;

    }

}

void SetLedOn(uint8_t Color,uint8_t State)
{
	KEYPAD_STATE = LED;
	if ((Color <=RED_COLOR) && (Color >=BLUE_COLOR)) {
		RegBusyFlag = SET;
		LED_ON[Color-1] = State;
		RegBusyFlag = RESET;
	}
	DrvLedSetState(&LED_ON[0]);

}
void SetLedBlink(uint8_t Color,uint8_t State)
{
	KEYPAD_STATE = LED;
	if ((Color <=RED_COLOR) && (Color >=BLUE_COLOR)) {
		RegBusyFlag = SET;
		LED_BLINK[Color-1] = State;
		RegBusyFlag = RESET;
	}
}

void SetLedBrigth(uint8_t brigth)
{
	led_brigth = brigth;
	KEYPAD_STATE = LED;
	SetBrigth(led_brigth);

}

void SetBackLigthColor(uint8_t color)
{
	backligth_color = color;
	SetBackLigth(backligch_brigth);

}



void SetBackLigth(uint8_t brigth)
{
#ifdef  FLAT_VERSION
	 switch (backligth_color)
	 {
	 		case  RED:
	 			brigth_color[0]=0x01;
	 			brigth_color[1]=0x00;
	 			brigth_color[2]=0x00;
	 			break;
	 		case GREEN:
	 		case YELLOW_GREEN:
	 			brigth_color[0]=0x00;
	 			brigth_color[1]=0x01;
	 			brigth_color[2]=0x00;
	 			break;
	 		case BLUE:
	 			brigth_color[0]=0x00;
	 			brigth_color[1]=0x00;
	 			brigth_color[2]=0x01;
	 			break;
	 		case YELLOW:
	 		case  AMBER:
	 			brigth_color[0]=0x01;
	 			brigth_color[1]=0x01;
	 			brigth_color[2]=0x00;
	 			break;
	 		case CYAN:
	 			brigth_color[0]=0x01;
	 			brigth_color[1]=0x00;
	 			brigth_color[2]=0x01;
	 			break;
	 		case VIOLET:
	 			brigth_color[0]=0x00;
	 			brigth_color[1]=0x01;
	 			brigth_color[2]=0x01;
	 			break;
	 		case WHITE:
	 			brigth_color[0]=0x01;
	 			brigth_color[1]=0x01;
	 			brigth_color[2]=0x01;
	 			break;
	 		}
	 SetBackBrigth(brigth );


#else

	KEYPAD_STATE = BACKLIGTH;
	SetBrigth(brigth );
	uint8_t brigth_color[3];
	switch (backligth_color)
	{
		case  RED:
			brigth_color[0]=0xFF;
			brigth_color[1]=0x00;
			brigth_color[2]=0x00;
			break;
		case GREEN:
		case YELLOW_GREEN:
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
		case  AMBER:
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
	}
	DrvLedSetState(&brigth_color[0]);
#endif
	backligch_brigth = brigth;
}

#ifdef  FLAT_VERSION

void SetBackBrigth(uint8_t brigth)
{

	TIM_OC_InitTypeDef sConfigOC = {0};
	BackBrigthOFF();
	if (brigth <= MAX_BRIGTH)
	{
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = (uint32_t)( ( (float)(MAX_BRIGTH - brigth_color[0]*brigth)/MAX_BRIGTH )*backpwmtim->Init.Period);
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(backpwmtim, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
			Error_Handler();
		}
		sConfigOC.Pulse = (uint32_t)( ( (float)(MAX_BRIGTH-brigth_color[1]*brigth)/MAX_BRIGTH )*backpwmtim->Init.Period);
		if (HAL_TIM_PWM_ConfigChannel(backpwmtim, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		{
			Error_Handler();
		}
		sConfigOC.Pulse = (uint32_t)( ( (float)(MAX_BRIGTH-brigth_color[2]*brigth)/MAX_BRIGTH )*backpwmtim->Init.Period);
		if (HAL_TIM_PWM_ConfigChannel(backpwmtim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		{
			Error_Handler();
		}
	}
	BackBrigthON();

}
void BackBrigthON()
{
	HAL_TIM_PWM_Start(backpwmtim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(backpwmtim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(backpwmtim,TIM_CHANNEL_3);

}
void BackBrigthOFF()
{
	HAL_TIM_PWM_Stop(backpwmtim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(backpwmtim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(backpwmtim,TIM_CHANNEL_3);
}


#endif

void SetBrigth(uint8_t brigth)
{

	TIM_OC_InitTypeDef sConfigOC = {0};

	if (brigth <= MAX_BRIGTH)
	{
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = (uint32_t)( ( (float)(brigth)/MAX_BRIGTH )*pwmtim->Init.Period);
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW ;//TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(pwmtim, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(pwmtim, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(pwmtim, &sConfigOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_3);
	}

}

uint8_t leds[3];

void BlinkProcess()
{
	if ( (RegBusyFlag == RESET) && ( LED_BLINK[0] || LED_BLINK[1] || LED_BLINK[2] ) && (KEYPAD_STATE == LED) )
	{
		switch (blink_count)
		{
		  case 0:
			 for (uint8_t i= 0;i<3;i++)
			 {
			 	leds[i] = LED_ON[i] & (~LED_BLINK[i]);
			 }
			 DrvLedSetState(&leds[0]);
			 blink_count = 1;
			 break;
		  case 1:
			 for (uint8_t i= 0;i<3;i++)
			 {
				 leds[i] = LED_ON[i] | LED_BLINK[i];
			  }
			  DrvLedSetState(&leds[0]);
			  blink_count = 0;
			  break;
		}
	}
}


void StartLEDShow(uint8_t show_type)
{
		switch (show_type)
		{
			case FULL_SHOW:
				for (uint8_t i=1;i<=0x3F;i=i+2)
				{
					SetBackLigth(i);
					vTaskDelay(50);
				}
				for (uint8_t i = 0x3F;i>0;)
				{
					if (i>2)
					{
						i=i-2;
						SetBackLigth(i);
						vTaskDelay(50);
					}
					else
					{
						SetBackLigth(0);
						break;
					}
			    }
				break;
			case FLASH_SHOW:
				SetBackLigth(startup_backligth);
				vTaskDelay(500);
				SetBackLigth(0);
				break;
			default:
				break;

		}


}




