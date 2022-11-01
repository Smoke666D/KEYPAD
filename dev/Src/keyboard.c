/*
 * keyboard.c
 *
 *  Created on: 25 февр. 2020 г.
 *      Author: igor.dymov
 */
/*----------------------- Includes ------------------------------------------------------------------*/
#include "keyboard.h"

/*----------------------- Structures ----------------------------------------------------------------*/
static xKeyPortStruct xKeyPortMass[KEYBOARD_COUNT]={
		{ GPIOA, KL1_Pin },
		{ GPIOB, KL2_Pin },
		{ GPIOB, KL3_Pin },
		{ GPIOB, KL4_Pin },
		{ GPIOB, KL5_Pin },
		{ GPIOB, KL6_Pin },
		{ GPIOB, KL7_Pin },
		{ GPIOB, KL8_Pin },
};
static StaticQueue_t      xKeyboardQueue;
static QueueHandle_t      pKeyboardQueue;
static EventGroupHandle_t pxKeyStatusFLag;
/*----------------------- Variables -----------------------------------------------------------------*/
static unsigned char STATUS[KEYBOARD_COUNT]                     = { 0U };
static unsigned int  COUNTERS[KEYBOARD_COUNT]                   = { 0U };
static unsigned char CODES[KEYBOARD_COUNT]                      = { kl1_key, kl2_key, kl3_key, kl4_key ,kl5_key, kl6_key,kl7_key, kl8_key };


uint8_t              KeyboardBuffer[ 16U * sizeof( KeyEvent ) ] = { 0U };
/*---------------------------------------------------------------------------------------------------*/
void vSetupKeyboard( void )
{
  pxKeyStatusFLag = xEventGroupCreate();
  pKeyboardQueue  = xQueueCreateStatic( 16U, sizeof( KeyEvent ), KeyboardBuffer, &xKeyboardQueue );
  vKeyboardInit( KEY_ON_MESSAGE );
  return;
}
/*---------------------------------------------------------------------------------------------------*/
QueueHandle_t pGetKeyboardQueue( void )
{
  return pKeyboardQueue;
}
/*---------------------------------------------------------------------------------------------------*/
void vKeyboardInit(  uint32_t message )
{
  switch ( message )
  {
    case KEY_ON_MESSAGE:
      xQueueReset( pKeyboardQueue );
      xEventGroupSetBits( pxKeyStatusFLag, KEY_READY );
      break;
    case KEY_OFF_MESSAGE:
      break;
    default:
      xEventGroupClearBits( pxKeyStatusFLag, KEY_READY );
      xQueueReset( pKeyboardQueue );
      break;
  }
  return;
}
/*---------------------------------------------------------------------------------------------------*/
/*
 * Задача обработки клавиш
 * */
void vKeyboardTask( void * argument )
{
  KeyEvent      TEvent;
  GPIO_PinState TK[KEYBOARD_COUNT];
  uint8_t       i = 0U;
  //uint8_t delay
  for(;;)
  {

    vTaskDelay(vFDGetRegState(KEYBOARD_PERIOD_ADRRES) );
    for ( i=0U; i<KEYBOARD_COUNT; i++ )                                          /* Считываем текущее состояние портов клавиатуры */
    {
      TK[i]=  HAL_GPIO_ReadPin( xKeyPortMass[i].KeyPort, xKeyPortMass[i].KeyPin );
    }
    for ( i=0U; i<KEYBOARD_COUNT; i++ )                                          /* Анализируем клавиутру */
    {
      /*Если текущие состояние порта ВЫКЛ, а предидущие состояние было ВКЛ,
	  Фиксируем отжатие клавищи (BRAKECODE)*/
      if ( STATUS[i] && ( TK[i] == KEY_OFF_STATE ) )
      {
        STATUS[i]      = KEY_OFF; /*Состоянии клавиши ВЫКЛ*/
        COUNTERS[i]    = 0U;      /*Сбрасываем счетчик*/
        TEvent.KeyCode = CODES[i];
        TEvent.Status  = BRAKECODE;
        xQueueReset( pKeyboardQueue );
        xQueueSend( pKeyboardQueue, &TEvent, portMAX_DELAY );

      }
      else
      {
        /*Если текущие состояние потрта ВКЛ, а предидущие было ВЫКЛ
        //то запускаме счеткик нажатий*/
        if ( !STATUS[i] && ( TK[i] == KEY_ON_STATE ) )
        {
          COUNTERS[i]++;
          /*если счетчик превысил значение SWITCHONDELAY то фиксируем нажатие*/
          if ( COUNTERS[i] >= ( vFDGetRegState( KEYDOWN_DELAY_ADRRES) ) )
          {
            COUNTERS[i]    = 0U;
            STATUS[i]      = KEY_ON;
            TEvent.KeyCode = CODES[i];
            TEvent.Status  = MAKECODE;
            xQueueSend( pKeyboardQueue, &TEvent, portMAX_DELAY );

          }
        }
        else if ( STATUS[i] && ( TK[i] == KEY_ON_STATE ) )
        {
          COUNTERS[i]++;
          switch ( STATUS[i] )
          {
            case KEY_ON:
              if ( COUNTERS[i] >=  vFDGetRegState( KEYDOWN_HOLD_ADDRESS)  )
              {
                STATUS[i]      = KEY_ON_REPEAT;
                COUNTERS[i]    = 0U;
                TEvent.KeyCode = CODES[i];
                TEvent.Status  = MAKECODE;
                xQueueSend( pKeyboardQueue, &TEvent, portMAX_DELAY );

              }
              break;
            case KEY_ON_REPEAT:
              if ( COUNTERS[i] >= vFDGetRegState( REPEAT_TIME_ADDRESS ) )
              {
                COUNTERS[i]    = 0U;
                TEvent.KeyCode = CODES[i];
                TEvent.Status  = MAKECODE;
                xQueueSend( pKeyboardQueue, &TEvent, portMAX_DELAY );

              }
              break;
            default:
    	      break;
          }
        }
      }
    }
  }
  TEvent.KeyCode = kl1_key;
               TEvent.Status  = MAKECODE;
  xQueueSend( pKeyboardQueue, &TEvent, portMAX_DELAY );

  return;
}


