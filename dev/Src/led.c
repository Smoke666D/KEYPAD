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
static SPI_HandleTypeDef* LEDSpi         = NULL;
static TIM_HandleTypeDef * pwmtim		 = NULL;
static TIM_HandleTypeDef * delaytim      = NULL;
static SemaphoreHandle_t  xSemaphore = NULL;
static uint8_t flag= 0;
static void BrigthON();
static void BrigthOFF();
void StartLEDShow();
uint8_t vSTPErrorDetection();
void vSTPNormalMode();
HAL_StatusTypeDef SPI_Transmit_DMA (uint8_t *pData, uint16_t size );
void DrvLedSetState(uint8_t * state);
static void SPI_REINIT(void);

static uint16_t us_delay=0;
static uint16_t us_counter = 0;




void DrvLedSetState(uint8_t * state)
{
	uint8_t buf[3];
	BrigthOFF();
	buf[0]=state[2];
	buf[1]=state[1];
	buf[2]=state[0];
	 HAL_SPI_Transmit(LEDSpi,&buf[0],3,100);
	//SPI_Transmit_DMA( state, 24U );
	vLatch();
	BrigthON();
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
	osDelay(1);//vSPTuSDealy(1);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	osDelay(1);//vSPTuSDealy(1);vSPTuSDealy(1);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	osDelay(1);//vSPTuSDealy(1);vSPTuSDealy(1);
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

void vLedInit(TIM_HandleTypeDef * htim, TIM_HandleTypeDef * dtim, SemaphoreHandle_t temp, SPI_HandleTypeDef* spi )
{

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
//	SPI_REINIT();
	HAL_TIM_MspPostInit(pwmtim);
	if (led_show_enable)
    {
		led_show_enable = 0;
		StartLEDShow();
    }
	SetLedBrigth(0x3F);
	DrvLedSetState(&LED_ON[0]);

}

void SetLedOn(uint8_t Color,uint8_t State)
{
	if ((Color <=RED_COLOR) && (Color >=BLUE_COLOR)) {
		RegBusyFlag = SET;
		LED_ON[Color-1] = State;
		RegBusyFlag = RESET;
	}
	if (backligch_brigth == OFF)
	{
		DrvLedSetState(&LED_ON[0]);
	}

}
void SetLedBlink(uint8_t Color,uint8_t State)
{
	if ((Color <=RED_COLOR) && (Color >=BLUE_COLOR)) {
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
		SetBrigth(led_brigth);
	}
}

void SetBackLigthColor(uint8_t color)
{
	backligth_color = color;
	SetBackLigth(backligch_brigth);
}




void SetBackLigth(uint8_t brigth)
{

	if (backligch_brigth == OFF)
	{
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
	}
	backligch_brigth = brigth;
	SetBrigth(backligch_brigth );

}




void SetBrigth(uint8_t brigth)
{

	TIM_OC_InitTypeDef sConfigOC = {0};
	BrigthOFF();
	if (brigth <= MAX_BRIGTH)
	{


		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = (uint32_t)( ( (float)(MAX_BRIGTH-brigth)/MAX_BRIGTH )*pwmtim->Init.Period);
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
	}
	BrigthON();
}

void BrigthON()
{
	HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(pwmtim,TIM_CHANNEL_3);

}
void BrigthOFF()
{
	HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(pwmtim,TIM_CHANNEL_3);
}


uint8_t leds[3];

void BlinkProcess()
{
	if ( (RegBusyFlag == RESET) && ( LED_BLINK[0] || LED_BLINK[1] || LED_BLINK[2] ) && (backligch_brigth == OFF) )
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
  SPI_HandleTypeDef *hspi     = ( SPI_HandleTypeDef* )( ( ( DMA_HandleTypeDef* ) hdma )->Parent ); /* Derogation MISRAC2012-Rule-11.5 */
  uint32_t          tickstart = 0U;
 tickstart = HAL_GetTick();                          /* Init tickstart for timeout management*/
  __HAL_SPI_DISABLE_IT( hspi, SPI_IT_ERR );           /* Disable ERR interrupt */
  CLEAR_BIT( hspi->Instance->CR2, SPI_CR2_TXDMAEN );  /* Disable Tx DMA Request */
  /* Check the end of the transaction */
  if ( SPI_WaitFlagStateUntilTimeout1( hspi, SPI_FLAG_BSY, RESET, 100U , tickstart ) != HAL_OK )
  {
    SET_BIT( hspi->ErrorCode, HAL_SPI_ERROR_FLAG );
  }
  hspi->TxXferCount = 0U;
  hspi->State       = HAL_SPI_STATE_READY;
  return;
}




HAL_StatusTypeDef SPI_Transmit_DMA ( uint8_t *pData, uint16_t size )
{
  HAL_StatusTypeDef errorcode = HAL_OK;
  /* Process Locked */
  __HAL_LOCK( LEDSpi );
  if ( LEDSpi->State != HAL_SPI_STATE_READY )
  {
    errorcode = HAL_BUSY;
    goto error;
  }
  /* Set the transaction information */
  LEDSpi->State       = HAL_SPI_STATE_BUSY_TX;
  LEDSpi->ErrorCode   = HAL_SPI_ERROR_NONE;
  LEDSpi->pTxBuffPtr  = ( uint8_t* )pData;
  LEDSpi->TxXferSize  = size;
  LEDSpi->TxXferCount = size;

  /* Enable the Tx DMA Stream/Channel */
  if ( HAL_OK != HAL_DMA_Start_IT( LEDSpi->hdmatx,
				   (uint32_t)LEDSpi->pTxBuffPtr,
				   (uint32_t)&LEDSpi->Instance->DR,
				   LEDSpi->TxXferCount ) )
  {
    /* Update SPI error code */
    SET_BIT( LEDSpi->ErrorCode, HAL_SPI_ERROR_DMA );
    errorcode   = HAL_ERROR;
    LEDSpi->State = HAL_SPI_STATE_READY;
    goto error;
  }
  /* Check if the SPI is already enabled */
  if ( ( LEDSpi->Instance->CR1 & SPI_CR1_SPE ) != SPI_CR1_SPE )
  {
    __HAL_SPI_ENABLE( LEDSpi );                      /* Enable SPI peripheral */
  }
  __HAL_SPI_ENABLE_IT( LEDSpi, ( SPI_IT_ERR ) );     /* Enable the SPI Error Interrupt Bit */
  SET_BIT( LEDSpi->Instance->CR2, SPI_CR2_TXDMAEN ); /* Enable Tx DMA Request */

error :
  /* Process Unlocked */
  __HAL_UNLOCK( LEDSpi);
  return errorcode;
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
 LEDSpi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
 if ( LEDSpi->Init.Direction == SPI_DIRECTION_1LINE )
 {
   SPI_1LINE_TX( LEDSpi );
 }
}



void StartLEDShow()
{

		SetBackLigth(0);
		for (uint8_t i=0;i<=0x3F;i++)
		{

			SetBackLigth(i);
			osDelay(30);
		}
		for (uint8_t i=0x3F;i>0;i=i-1)
		{
			SetBackLigth(i);
			osDelay(30);
		}
		SetBackLigth(0);

}




