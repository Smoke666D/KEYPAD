/*
 * led.c
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */
#include "led.h"
#include "math.h"

#define MAX_BRIGTH_COUNTER MAX_BRIGTH*3

uint8_t LED_ON[3] 		=         { 0x00 , 0x00 , 0x00 };
uint8_t LED_OFF[3] 		=         { 0x00 , 0x00 , 0x00 };
uint8_t LED_BLINK[3] 	=         { 0x00 , 0x00 , 0x00 };

static uint16_t backligch_brigth = 0x1F;
static uint8_t backligth_color 	= 0U;
static uint16_t led_brigth 		= 0x3F;
static uint8_t led_show_enable  = OFF;
static uint8_t startup_backligth =0x3F;
static SPI_HandleTypeDef* LEDSpi         = NULL;
static uint8_t color_div =1;


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



static void DrvLedSetState(uint8_t * state)
{
	HAL_SPI_Transmit(LEDSpi,&state[0],3,100);
	vLatch();
	return;
}

uint16_t calcBrigt(uint8_t pbr)
{
  uint16_t br = pbr;
  uint16_t res;
  if ( pbr > MAX_BRIGTH ) br = MAX_BRIGTH;
  res = sin((float)br*(3.14/2.0)/MAX_BRIGTH)*(MAX_BRIGTH_COUNTER);
  return res;
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
	 return;
}


void vLedInit(TIM_HandleTypeDef * htim,  EventGroupHandle_t temp, SPI_HandleTypeDef* spi )
{
	pwmtim = htim;
	LEDSpi = spi;
	backligth_color  = vFDGetRegState(DEF_BL_COLOR_ADR);
	led_brigth 		 = calcBrigt(vFDGetRegState(DEF_LED_BRIGTH_ADR));
	backligch_brigth = calcBrigt(vFDGetRegState(DEF_BL_BRIGTH_ADR));
	led_show_enable = vFDGetRegState(LED_SHOW_ADRRES);
    SetBrigth(0x3F);
    return;
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
	return;
}


/*
 * Функция включения светодиод
 */
void SetLedOn(uint8_t Color,uint8_t State)
{
	//if ((Color >=RED_COLOR) && (Color <=BLUE_COLOR))
	{
		LED_ON[Color-1] = State;
	}
	return;
}

void SetLedBlink(uint8_t Color,uint8_t State)
{
	//if ((Color >=RED_COLOR) && (Color <=BLUE_COLOR))
	{
		LED_BLINK[Color-1] = State;
	}
	return;
}






void SetLedBrigth(uint8_t brigth)
{
	led_brigth = calcBrigt(brigth);
	return;
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
	switch (backligth_color)
	{
		case  RED:
			brigth_color[0]=0xFF;
			brigth_color[1]=0x00;
			brigth_color[2]=0x00;
			color_div =1;
			break;
		case GREEN:
			brigth_color[0]=0x00;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			color_div =1;
			break;
		case BLUE:
			brigth_color[0]=0x00;
			brigth_color[1]=0x00;
			brigth_color[2]=0xFF;
			color_div =1;
			break;
		case YELLOW:
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			color_div =2;
			break;
		case YELLOW_GREEN:
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			color_div =2;
			break;
		case  AMBER:
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0x00;
			color_div =2;
			break;
		case VIOLET:
			brigth_color[0]=0xFF;
			brigth_color[1]=0x00;
			brigth_color[2]=0xFF;
			color_div =2;
			break;
		case CYAN:
			brigth_color[0]=0x00;
			brigth_color[1]=0xFF;
			brigth_color[2]=0xFF;
			color_div =2;
			break;
		case WHITE:
			brigth_color[0]=0xFF;
			brigth_color[1]=0xFF;
			brigth_color[2]=0xFF;
			color_div =2;
			break;
	}
	backligch_brigth =calcBrigt( brigth);
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


	 	red_period = 100;
		green_period = 0;
		blue_period =0;

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

static uint16_t led_brigth_counter = 0;
static uint8_t data[3]={0,0,0};
static uint8_t temp_led=0;

void LedProcees()
{
	led_brigth_counter++;
     if (led_brigth_counter>MAX_BRIGTH_COUNTER)
     {
    	led_brigth_counter = 0;

     }
     data[0] =0;
     data[1] =0;
     data[2] =0;
     temp_led = ~(LED_ON[0]  | LED_ON[1] | LED_ON[2] | LED_BLINK[0]| LED_BLINK[1]| LED_BLINK[2]);
	 if (led_brigth_counter < backligch_brigth/color_div)
	 {
		 data[2]=brigth_color[0] & temp_led;
	 	 data[1]=brigth_color[1] & temp_led;
	 	 data[0]=brigth_color[2] & temp_led;
 	 }
	 if (led_brigth_counter <= led_brigth)
	 {
	 	data[2]|=LED_ON[0];
	 	data[1]|=LED_ON[1];
	 	data[0]|=LED_ON[2];

	 }
     DrvLedSetState(&data[0]);
}


