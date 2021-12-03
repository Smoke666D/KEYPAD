/*
 * led.c
 *
 *  Created on: Nov 12, 2021
 *      Author: igor.dymov
 */
#include "led.h"

static uint8_t LED_ON[3] 		= { 0U , 0U , 0U };
static uint8_t LED_BLINK[3] 	= { 0U , 0U , 0U };
static uint8_t LedBlinkState 	= RESET;
static uint8_t backligch_brigth = OFF;
static uint8_t backligth_color 	= 0U;
static uint8_t blink_count 		= 0U;
static uint8_t RegBusyFlag 		= RESET;
static uint8_t led_brigth 		= OFF;
static SPI_HandleTypeDef* LEDSpi           = NULL;

void DrvLedSetState(uint8_t * state);
static void SPI_REINIT(void);
TIM_HandleTypeDef * pwmtim;


void vSTPInit()
{

	// Входим в режим Detectiom

	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET); //OE High
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);  //LE low
	 for (uint8_t i=0;i<5;i++)
	 {

		 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
		 osDelay(1);
		 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		 switch (i)
		 {
		    case 1:
		    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET); //OE Low
		    	break;
		    case 2:
		    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET); //OE High
		    	break;
		    case 3:
		    	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);  //LE High
		    	break;
		    case 4:
		    	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);  //LE low
		    	break;
		    default:
		    	break;
		 }
	 }
}


void vLedInit(TIM_HandleTypeDef * htim,  SPI_HandleTypeDef* spi )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	pwmtim = htim;
	LEDSpi = spi;
	backligth_color  = vFDGetRegState(DEF_BL_COLOR_ADR);
	led_brigth 		 = vFDGetRegState(DEF_LED_BRIGTH_ADR);
	backligch_brigth = vFDGetRegState(DEF_BL_BRIGTH_ADR);
	//Выключем SPI и переиницилизируем порты на GPIO
	HAL_SPI_MspDeInit(LEDSpi);
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	vSTPInit();
	SPI_REINIT();
	HAL_TIM_MspPostInit(pwmtim);
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

void SetLedBrigth(uint8_t brigth)
{
	led_brigth = brigth;
	if (backligch_brigth == OFF)
	{
		SetBackLigth(led_brigth);
	}
}

void SetBackLigthColor(uint8_t color)
{
	backligth_color = color;
	SetBackLigth(backligch_brigth);
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


static HAL_StatusTypeDef SPI_WaitFlagStateUntilTimeout1(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus State,
                                                       uint32_t Timeout, uint32_t Tickstart)
{
  while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) != State)
  {
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) >= Timeout) || (Timeout == 0U))
      {
        /* Disable the SPI and reset the CRC: the CRC value should be cleared
        on both master and slave sides in order to resynchronize the master
        and slave for their respective CRC calculation */

        /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
        __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

        if ((hspi->Init.Mode == SPI_MODE_MASTER) && ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
                                                     || (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY)))
        {
          /* Disable SPI peripheral */
          __HAL_SPI_DISABLE(hspi);
        }

        /* Reset CRC Calculation */
        if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
        {
          SPI_RESET_CRC(hspi);
        }

        hspi->State = HAL_SPI_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(hspi);

        return ( HAL_TIMEOUT );
      }
    }
  }
  return ( HAL_OK );
}



static void SPI_DMATransmit ( DMA_HandleTypeDef *hdma )
{
 /* SPI_HandleTypeDef *hspi     = ( SPI_HandleTypeDef* )( ( ( DMA_HandleTypeDef* ) hdma )->Parent ); /* Derogation MISRAC2012-Rule-11.5 */
  uint32_t          tickstart = 0U;
 //tickstart = HAL_GetTick();                          /* Init tickstart for timeout management*/
 // __HAL_SPI_DISABLE_IT( hspi, SPI_IT_ERR );           /* Disable ERR interrupt */
//  CLEAR_BIT( hspi->Instance->CR2, SPI_CR2_TXDMAEN );  /* Disable Tx DMA Request */
  /* Check the end of the transaction */
 // if ( SPI_WaitFlagStateUntilTimeout1( hspi, SPI_FLAG_BSY, RESET, SPI_DEFAULT_TIMEOUT, tickstart ) != HAL_OK )
 // {
 //   SET_BIT( hspi->ErrorCode, HAL_SPI_ERROR_FLAG );
//  }
 // hspi->TxXferCount = 0U;
 // hspi->State       = HAL_SPI_STATE_READY;
 // return;
}







static void SPI_REINIT(void)
{
 LEDSpi->Instance = SPI2;
 LEDSpi->Init.Mode = SPI_MODE_MASTER;
 LEDSpi->Init.Direction = SPI_DIRECTION_2LINES;
 LEDSpi->Init.DataSize = SPI_DATASIZE_8BIT;
 LEDSpi->Init.CLKPolarity = SPI_POLARITY_HIGH;
 LEDSpi->Init.CLKPhase = SPI_PHASE_1EDGE;
 LEDSpi->Init.NSS = SPI_NSS_SOFT;
 LEDSpi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
 LEDSpi->Init.FirstBit = SPI_FIRSTBIT_MSB;
 LEDSpi->Init.TIMode = SPI_TIMODE_DISABLE;
 LEDSpi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
 LEDSpi->Init.CRCPolynomial = 10;
 if (HAL_SPI_Init(LEDSpi) != HAL_OK)
 {
   Error_Handler();
 }
 LEDSpi->hdmatx->XferHalfCpltCallback = NULL;            /* Set the SPI TxDMA Half transfer complete callback */
 LEDSpi->hdmatx->XferCpltCallback     = SPI_DMATransmit; /* Set the SPI TxDMA transfer complete callback */
 LEDSpi->hdmatx->XferErrorCallback    = NULL;            /* Set the DMA error callback */
 LEDSpi->hdmatx->XferAbortCallback    = NULL;            /* Set the DMA AbortCpltCallback */
 /* Init field not used in handle to zero */
 LEDSpi->pRxBuffPtr  = ( uint8_t* )NULL;
 LEDSpi->TxISR       = NULL;
 LEDSpi->RxISR       = NULL;
 LEDSpi->RxXferSize  = 0U;
 LEDSpi->RxXferCount = 0U;
}
