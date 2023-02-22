/*
 * flash_data.c
 *
 *  Created on: Dec 1, 2021
 *      Author: igor.dymov
 */


#include "flash_data.h"
#include "301/CO_NMT_Heartbeat.h"
#include "led.h"
#include "OD.h"

static uint8_t FisrtStart = 1;
uint8_t SettingsREG[REG_SIZE]={VALID_CODE, 0x00, 0x15, 0x01, 0x3F, 0x00, WHITE, 1 ,4, 2, 4, 3, 00, 44,00,55};

static uint8_t *MEM_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len);
static uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len);
static uint16_t MEM_If_Init_FS(void);
static uint16_t MEM_If_Erase_FS(void);
static uint16_t MEM_If_DeInit_FS(void);


void * cgetREGAdr(uint8_t adr)
{
	return ((void *)&SettingsREG[adr]);
}

void vFDWtiteReg(void)
{
	uint8_t * src = (uint8_t *) FLASH_DATA_ADR;
	MEM_If_Init_FS();
	MEM_If_Erase_FS();
	MEM_If_Write_FS(&SettingsREG[0], src, sizeof(SettingsREG));
	MEM_If_DeInit_FS();
}

void vFDInit( void )
{
	uint8_t * src =  (uint8_t *) FLASH_DATA_ADR;
	uint8_t  buff;
	if (FisrtStart)
	{
	   MEM_If_Read_FS(src, &buff, 1);
	   if (buff!= VALID_CODE)
	   {
		   vFDWtiteReg();
	   }
		MEM_If_Read_FS(src, &SettingsREG[0],  sizeof(SettingsREG));
		FisrtStart = 0;
		OD_set_value(OD_ENTRY_H2004_keyBoardParametr,0x01,&SettingsREG[KEYBOARD_PERIOD_ADRRES ], 1, true);
		OD_set_value(OD_ENTRY_H2004_keyBoardParametr,0x02,&SettingsREG[KEYBOARD_PERIOD_ADRRES +1 ], 1, true);
		OD_set_value(OD_ENTRY_H2004_keyBoardParametr,0x03,&SettingsREG[KEYBOARD_PERIOD_ADRRES +2 ], 1, true);
		OD_set_value(OD_ENTRY_H2004_keyBoardParametr,0x04,&SettingsREG[KEYBOARD_PERIOD_ADRRES +3 ], 1, true);
		OD_set_value(OD_ENTRY_H2005_PWM_Parametr,0x01,&SettingsREG[KEYBOARD_PERIOD_ADRRES +3 ], 1, true);
		OD_set_value(OD_ENTRY_H2005_PWM_Parametr,0x02,&SettingsREG[KEYBOARD_PERIOD_ADRRES +3 ], 1, true);
	}
}
/*
 *
 */
void vFDSetRegState(uint8_t adr, uint8_t state)
{
	SettingsREG[adr]= state;
	vFDWtiteReg();
}
/*
 *
 */
void vFDSetRegState16(uint8_t adr, uint16_t state)
{
	memmove(&SettingsREG[adr], &state, sizeof(state));
	vFDWtiteReg();
}

uint8_t vFDGetRegState(uint8_t adr)
{
	if (FisrtStart)
	{
	  vFDInit();
	}
	return SettingsREG[adr];
}

uint16_t vGetBitrate()
{
	uint16_t data = 0;
	 if (FisrtStart)
	 {
		  vFDInit();
	 }
	switch(SettingsREG[BITRATE_ADR])
	{
		case 0x00:
			data = 1000;
			break;
		case 0x02:
			data = 500;
			break;
		case 0x03:
			data = 250;
			break;
		case 0x04:
			data = 125;
			break;
		case 0x06:
			data =50;
			break;
		case 0x07:
			data = 20;
			break;
		default:
			data = 125;
			break;
	}
    return data;
}

uint16_t vFDGetNMTState()
{
	 uint16_t res = 0;
	 if (FisrtStart) {
	   vFDInit();
     }
	 if (SettingsREG[NMT_STATE_ADR] == 0x01) {
		 res = CO_NMT_STARTUP_TO_OPERATIONAL;
	 }
	 return res;
}

uint8_t vFDGetStartMessage()
{
	if (FisrtStart)
	  {
		  vFDInit();
	  }
	  return SettingsREG[NMT_START_MESSAGE];

}

uint8_t vGetNodeId()
{
  if (FisrtStart)
  {
	  vFDInit();
  }
  return SettingsREG[NODE_ID_ADR];
}



/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Memory initialization routine.
  * @retval USBD_OK if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Init_FS(void)
{

  HAL_StatusTypeDef flashStatus = HAL_ERROR;
  while ( flashStatus != HAL_OK )
  {
    flashStatus = HAL_FLASH_Unlock();
  }
  return HAL_OK;
  /* USER CODE END 0 */
}

/**
  * @brief  De-Initializes Memory
  * @retval USBD_OK if operation is successful, MAL_FAIL else
  */
uint16_t MEM_If_DeInit_FS(void)
{
  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef flashStatus = HAL_ERROR;
  while ( flashStatus != HAL_OK )
  {
    flashStatus = HAL_FLASH_Lock();
  }
  return HAL_OK;
  /* USER CODE END 1 */
}

/**
  * @brief  Erase sector.
  * @param  Add: Address of sector to be erased.
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Erase_FS()
{
  /* USER CODE BEGIN 2 */
  uint32_t               pageError = 0U;
  HAL_StatusTypeDef      status    = HAL_ERROR;
  FLASH_EraseInitTypeDef eraseInit;


    eraseInit.TypeErase    = FLASH_TYPEERASE_PAGES;
    eraseInit.Banks        = FLASH_BANK_1;
    eraseInit.PageAddress      = FLASH_DATA_ADR;
    eraseInit.NbPages    = 1U;
    status = HAL_FLASHEx_Erase( &eraseInit, &pageError );

  return status;
  /* USER CODE END 2 */
}

/**
  * @brief  Memory write routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be written (in bytes).
  * @retval USBD_OK if operation is successful, MAL_FAIL else.
  */
uint16_t MEM_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  /* USER CODE BEGIN 3 */
  uint32_t           i      = 0U;
  HAL_StatusTypeDef      status    = HAL_ERROR;

  for ( i=0U; i<Len; i+=4U )
  {
	if ( ( uint32_t )( dest + i ) > FLASH_SIZE )
	{
      if ( HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, ( uint32_t )( dest + i ), *( uint32_t* )( src + i ) ) == HAL_OK )
      {
        if ( *( uint32_t* )( src + i ) != *( uint32_t* )( dest + i ) )
        {
        	status = HAL_ERROR;
          break;
        }
        else
        {
        	status = HAL_OK;
        }
      }
      else
      {
    	  status = HAL_ERROR;
        break;
      }
	}
  }
  return status;
  /* USER CODE END 3 */
}

/**
  * @brief  Memory read routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the physical address where data should be read.
  */
uint8_t *MEM_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
  /* Return a valid address to avoid HardFault */
  /* USER CODE BEGIN 4 */

    uint32_t i    = 0U;
    uint8_t *psrc = src;

    for ( i=0U; i<Len; i++ )
    {
      dest[i] = *psrc++;
    }
    return ( uint8_t* )( dest );

  /* USER CODE END 4 */
}



