/*
 * led.c
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */
#include "led.h"




uint8_t LED_ON[3] 		=         { 0x00 , 0x00 , 0x00 };
uint8_t LED_OFF[3] 		=         { 0x00 , 0x00 , 0x00 };
uint8_t LED_BLINK[3] 	=         { 0x00 , 0x00 , 0x00 };

static uint8_t backligch_brigth = 0x1F;
static uint8_t backligth_color 	= 0U;
static uint8_t blink_count 		= 0U;
//static uint8_t RegBusyFlag 		= RESET;
static uint8_t led_brigth 		= 0x3F;
static uint8_t led_show_enable  = OFF;
static uint8_t startup_backligth =0x3F;
static SPI_HandleTypeDef* LEDSpi         = NULL;



#ifdef  FLAT_VERSION
static uint8_t brigth_color[3]=  { 0x00 , 0x00 , 0x00 };
static TIM_HandleTypeDef * backpwmtim		 = NULL;
#endif
enum KEY_FSM
{
	BACKLIGTH,
	LED,
};
enum KEY_FSM KEYPAD_STATE = BACKLIGTH;
static TIM_HandleTypeDef * pwmtim		 = NULL;
void SetBackBrigth(uint8_t brigth);
void BackBrigthON();
void BackBrigthOFF();
uint8_t vSTPErrorDetection();
void vSTPNormalMode();
HAL_StatusTypeDef SPI_Transmit_DMA (uint8_t *pData, uint16_t size );
void DrvLedSetState(uint8_t * state);


static uint8_t tbuf[3]={0,0,0};
void DrvLedSetState(uint8_t * state)
{
	uint8_t buf[3];
	buf[0]=state[2];
	buf[1]=state[1];
	buf[2]=state[0];
	if ( (tbuf[0] != buf[0]) || (tbuf[1] != buf[1]) || (tbuf[2] != buf[2]) )
	{
		HAL_SPI_Transmit(LEDSpi,&buf[0],3,100);
		vLatch();
		tbuf[0] = buf[0];
		tbuf[1] = buf[1];
		tbuf[2] = buf[2];
	}
	return;
}



void vLatch()
{
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	for (uint8_t i =0;i< 10;i++)
	{
	}
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	return;
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
		 vTaskDelay(1);
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
	 vTaskDelay(1);
	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	 vTaskDelay(1);
	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET); //OE Low
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	 uint32_t buf=0;
	 for (uint8_t i =0;i<(3*8);i++)
	 {
		 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
		 vTaskDelay(1);
		 if ( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) == GPIO_PIN_SET )
         {
			 buf=buf<<1;
			 buf |= 0x0001;
         }
		 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		 vTaskDelay(1);
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
	    vTaskDelay(1);
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


void vLedInit(TIM_HandleTypeDef * htim,  EventGroupHandle_t temp, SPI_HandleTypeDef* spi )
{
	pwmtim = htim;
	LEDSpi = spi;
	backligth_color  = vFDGetRegState(DEF_BL_COLOR_ADR);
	led_brigth 		 = vFDGetRegState(DEF_LED_BRIGTH_ADR);
	backligch_brigth = vFDGetRegState(DEF_BL_BRIGTH_ADR);
	led_show_enable = vFDGetRegState(LED_SHOW_ADRRES);
    SetBrigth(0x3F);
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


/*
 * Функция включения светодиод
 */
void SetLedOn(uint8_t Color,uint8_t State)
{
	if ((Color <=RED_COLOR) && (Color >=BLUE_COLOR))
	{
		RegBusyFlag = SET;
		LED_ON[Color-1] = State;
		RegBusyFlag = RESET;
	}
	return 0;
}

void SetLedBlink(uint8_t Color,uint8_t State)
{

	//DrvLedSetState(&LED_OFF[0]);
	//if (KEYPAD_STATE != LED)
	//{
	//	KEYPAD_STATE = LED;
//		SetLedBrigth(led_brigth);
//	}
	if ((Color <=RED_COLOR) && (Color >=BLUE_COLOR))
	{
		RegBusyFlag = SET;
		LED_BLINK[Color-1] = State;
		RegBusyFlag = RESET;
	}
}

void SetLedBrigth(uint8_t brigth)
{
	led_brigth = brigth;
	//if (KEYPAD_STATE == LED)
//	{
//		SetBrigth(led_brigth);
//	}

}

void SetBackLigthColor(uint8_t color)
{
	backligth_color = color;
	SetBackLigth(backligch_brigth);

}

unsigned int red_period, green_period, blue_period;

static uint8_t brigth_color[3] = {0,0,0};

void SetBackLigth(uint8_t brigth)
{

//	KEYPAD_STATE = BACKLIGTH;
	//SetBrigth(brigth );

	switch (backligth_color)
	{
		case  RED:
			red_period = 0;
			brigth_color[0]=0xFF;
			brigth_color[1]=0x00;
			brigth_color[2]=0x00;
			break;
		case GREEN:
			green_period = 300;
			brigth_color[0]=0x00;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			break;
		case BLUE:
			blue_period = 200;
			brigth_color[0]=0x00;
			brigth_color[1]=0x00;
			brigth_color[2]=0xFF;
			break;
		case YELLOW:
			red_period = 0;
			green_period = 300;
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			break;
		case YELLOW_GREEN:
			red_period = 500;
			green_period = 300;
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			break;
		case  AMBER:
			red_period = 0;
			green_period = 700;
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			break;
		case VIOLET:
			red_period = 0;
			green_period = 300;
			blue_period = 200;
			brigth_color[0]=0xFF;
			brigth_color[1]=0x00;
			brigth_color[2]=0xFF;
			break;
		case CYAN:
			red_period = 0;
			green_period = 300;
			blue_period = 200;
			brigth_color[0]=0x00;
			brigth_color[1]=0xFF;
			brigth_color[2]=0xFF;
			break;
		case WHITE:
			red_period = 0;
			green_period = 300;
			blue_period = 200;
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0xFF;
			break;
	}
	//DrvLedSetState(&brigth_color[0]);
	backligch_brigth = brigth;
	//SetBrigth(brigth );
}




/*
 * Функция установки якрости.
 *
 */


void SetBrigth(uint8_t brigth)
{

	TIM_OC_InitTypeDef sConfigOC = {0};

	if (brigth <= MAX_BRIGTH)
	{

		//if (KEYPAD_STATE != BACKLIGTH)
		//{
		//	red_period = 0;
		//	green_period = 300;
		//	blue_period = 200;
		//}
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW ;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

		/*
		 * Яркость устанвливается для каждого цвета отдельно. Возможно задавать индивидуалное соотношение яркостей цветов для получения
		 * дополнительных переходных цветов, например  AMBER и YELLOW_GREEN
		 */
		HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_1);
		sConfigOC.Pulse = (uint32_t)( ( (float)(brigth)/MAX_BRIGTH )*(pwmtim->Init.Period- red_period));
		HAL_TIM_PWM_ConfigChannel(pwmtim, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_2);
		sConfigOC.Pulse = (uint32_t)( ( (float)(brigth)/MAX_BRIGTH )*(pwmtim->Init.Period-green_period));
		HAL_TIM_PWM_ConfigChannel(pwmtim, &sConfigOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_3);
		sConfigOC.Pulse = (uint32_t)( ( (float)(brigth)/MAX_BRIGTH )*(pwmtim->Init.Period-blue_period));
		HAL_TIM_PWM_ConfigChannel(pwmtim, &sConfigOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_3);
	}

}

uint8_t leds[3];

void BlinkProcess()
{
	if ( (RegBusyFlag == RESET) &&  (( LED_BLINK[0] || LED_BLINK[1] || LED_BLINK[2] ) && (KEYPAD_STATE == LED) /*|| (blink_count == 1)*/) )
	{
		switch (blink_count)
		{
		  case 1:
			 for (uint8_t i= 0;i<3;i++)
			 {
			 	leds[i] = LED_ON[i] & (~LED_BLINK[i]);
			 }
			 DrvLedSetState(&leds[0]);
			 blink_count = 0;
			 break;
		  case 0:
			 for (uint8_t i= 0;i<3;i++)
			 {
				 leds[i] = LED_ON[i] | LED_BLINK[i];
			  }
			  DrvLedSetState(&leds[0]);
			  blink_count = 1;
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
		backligch_brigth = vFDGetRegState(DEF_BL_BRIGTH_ADR);
}

static uint8_t led_brigth_counter = 0;
static uint8_t data[3]={0,0,0};
static uint8_t old_data[3]={0,0,0};
void LedProcees()
{

	led_brigth_counter++;
    if (led_brigth_counter>MAX_BRIGTH*2)
    	led_brigth_counter = 0;
     old_data[0]=data[0];
     old_data[1]=data[2];
     old_data[2]=data[3];
     data[0] =0;
     data[1] =0;
     data[2] =0;
	 if (led_brigth_counter <= backligch_brigth*2)
	 {
		 data[0]=(brigth_color[0] & (~LED_ON[0]));
	 	 data[1]=(brigth_color[1] & (~LED_ON[1]));
	 	 data[2]=(brigth_color[2] & (~LED_ON[2]));
 	 }
	 if (led_brigth_counter <= led_brigth*2)
	 {
	 	data[0]|=LED_ON[0];
	 	data[1]|=LED_ON[1];
	 	data[2]|=LED_ON[2];

	 }
     if ((old_data[0]!=data[0]) || (old_data[1]!=data[1]) || (old_data[2]!=data[2]))
     {
    	 DrvLedSetState(&data[0]);
     }
}


