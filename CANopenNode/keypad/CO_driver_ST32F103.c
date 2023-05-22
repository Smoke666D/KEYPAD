/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "301/CO_driver.h"
#include "CO_driver_ST32F103.h"

/* CAN masks for identifiers */
#define CANID_MASK                              0x07FF  /*!< CAN standard ID mask */
#define FLAG_RTR                                0x8000  /*!< RTR flag, part of identifier */
/* Mutex for atomic access */
static osMutexId_t co_mutex;

/* Semaphore for main app thread synchronization */
osSemaphoreId_t co_drv_app_thread_sync_semaphore;

/* Semaphore for periodic thread synchronization */
osSemaphoreId_t co_drv_periodic_thread_sync_semaphore;
/* Local CAN module object */
static CO_CANmodule_t* CANModule_local = NULL;  /* Local instance of global CAN module */

uint8_t co_drv_create_os_objects(void) {
    /* Create new mutex for OS context */
    if (co_mutex == NULL) {
        const osMutexAttr_t attr = {
            .attr_bits = osMutexRecursive,
            .name = "co"
        };
        co_mutex = osMutexNew(&attr);
    }

    /* Semaphore for main app thread synchronization */
    if (co_drv_app_thread_sync_semaphore == NULL) {
        const osSemaphoreAttr_t attr = {
                .name = "co_app_thread_sync"
        };
        co_drv_app_thread_sync_semaphore = osSemaphoreNew(1, 1, &attr);
    }

    /* Semaphore for periodic thread synchronization */
    if (co_drv_periodic_thread_sync_semaphore == NULL) {
        const osSemaphoreAttr_t attr = {
                .name = "co_periodic_thread_sync"
        };
        co_drv_periodic_thread_sync_semaphore = osSemaphoreNew(1, 1, &attr);
    }

    return 1;
}

/**
 * \brief           Lock mutex or wait to be available
 * \return          `1` on success, `0` otherwise
 */
uint8_t
co_drv_mutex_lock(void) {
    return osMutexAcquire(co_mutex, osWaitForever) == osOK;
}

/**
 * \brief           Release previously locked mutex
 * \return          `1` on success, `0` otherwise
 */
uint8_t
co_drv_mutex_unlock(void) {
    return osMutexRelease(co_mutex) == osOK;
}

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
    /* Put CAN module in configuration mode */
	 if (CANptr != NULL) {
		 HAL_CAN_Stop(CANptr);
	 }
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */
	  if (CANmodule->CANptr != NULL && HAL_CAN_Start(CANmodule->CANptr) == HAL_OK) {
	        CANmodule->CANnormal = true;
	  }
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANptr,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    uint16_t i;
    CAN_HandleTypeDef * hcan = CANptr;
    CAN_FilterTypeDef  sFilterConfig;

    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    CANModule_local = CANmodule;
    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = (rxSize <= 32U) ? true : false;/* microcontroller dependent */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for(i=0U; i<txSize; i++){
        txArray[i].bufferFull = false;
    }


    /* Configure CAN module registers */

    /* Configure CAN module registers */
    hcan->Init.Mode = CAN_MODE_NORMAL;
    hcan->Init.TimeTriggeredMode = DISABLE;
    hcan->Init.AutoBusOff = ENABLE;
    hcan->Init.AutoWakeUp = DISABLE;
    hcan->Init.AutoRetransmission = DISABLE;
    hcan->Init.ReceiveFifoLocked = DISABLE;
    hcan->Init.TransmitFifoPriority = DISABLE;

    /* Configure CAN timing */

        switch (CANbitRate)
        {
            case 1000:  hcan->Init.Prescaler = 2;
                   break;
            case 500:  hcan->Init.Prescaler = 4;
                   break;
            case 250:  hcan->Init.Prescaler = 8;
                   break;
            default:
            case 125:  hcan->Init.Prescaler = 16;
                  break;
            case 100:  hcan->Init.Prescaler = 20;
                 break;
            case 50:  hcan->Init.Prescaler = 120;
                 break;
            case 20:  hcan->Init.Prescaler = 300;
                 break;
            case 10:  hcan->Init.Prescaler = 600;
                 break;
        }
        hcan->Init.SyncJumpWidth = CAN_SJW_4TQ;
        hcan->Init.TimeSeg1 = CAN_BS1_12TQ;
        hcan->Init.TimeSeg2 = CAN_BS2_5TQ;

        sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
        sFilterConfig.FilterBank =0;
        sFilterConfig.FilterFIFOAssignment =0;
        sFilterConfig.FilterIdLow		 = ( 0x180 | vGetNodeId() ) <<5;
        sFilterConfig.FilterIdHigh 		 = ( 0x200 | vGetNodeId() ) <<5;
        sFilterConfig.FilterMaskIdLow	 = ( 0x300 | vGetNodeId() ) <<5;
        sFilterConfig.FilterMaskIdHigh   = ( 0x400 | vGetNodeId() ) <<5;
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST ;
        sFilterConfig.FilterScale =CAN_FILTERSCALE_16BIT;
        sFilterConfig.SlaveStartFilterBank = 0;

        HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

       sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
       sFilterConfig.FilterBank =1;
       sFilterConfig.FilterFIFOAssignment =1;
       sFilterConfig.FilterIdLow = ( 0x500 | vGetNodeId() )<<5;
       sFilterConfig.FilterIdHigh =( 0x600 | vGetNodeId() )<<5;
       sFilterConfig.FilterMaskIdLow = 0;
       sFilterConfig.FilterMaskIdHigh =0;
       sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
       sFilterConfig.FilterScale =CAN_FILTERSCALE_16BIT;
       sFilterConfig.SlaveStartFilterBank = 0;

       HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

       if (HAL_CAN_Init(hcan) != HAL_OK)
       {
            Error_Handler();
       }

      if (HAL_CAN_ActivateNotification(hcan,
                  0
                  | CAN_IT_TX_MAILBOX_EMPTY
				  | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN
				  | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN
				  | CAN_IT_BUSOFF
				  ) != HAL_OK) {
              return CO_ERROR_ILLEGAL_ARGUMENT;
      }
      HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 6, 0);
      HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);

      HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 6, 0);
      HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);


      HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn , 6, 0);
      HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn );

      HAL_NVIC_SetPriority(CAN1_RX1_IRQn  , 6, 0);
      HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn  );

    return ( CO_ERROR_NO );
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
	if (CANmodule != NULL && CANmodule->CANptr != NULL) {
		    HAL_CAN_Stop(CANmodule->CANptr);
	    }
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (CANrx_callback!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        if(rtr){
            buffer->ident |= 0x0800U;
        }
        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){

        }
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
         * Microcontroller specific. */
        buffer->ident = ((uint32_t)ident & CANID_MASK)
                               | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}



HAL_StatusTypeDef CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[])
{
  HAL_CAN_StateTypeDef state = hcan->State;
  uint32_t tsr = READ_REG(hcan->Instance->TSR);

  if ((state == HAL_CAN_STATE_READY) || (state == HAL_CAN_STATE_LISTENING))
  {
    /* Check that all the Tx mailboxes are not full */
    if (((tsr & CAN_TSR_TME0) != 0U) ||  ((tsr & CAN_TSR_TME1) != 0U) || ((tsr & CAN_TSR_TME2) != 0U))
    {
      uint32_t transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
      if (transmitmailbox <= 2U)
      {
          	  hcan->Instance->sTxMailBox[transmitmailbox].TIR = ((pHeader->StdId << CAN_TI0R_STID_Pos) | pHeader->RTR);
      	  hcan->Instance->sTxMailBox[transmitmailbox].TDTR = (pHeader->DLC);
      	  if (pHeader->TransmitGlobalTime == ENABLE)
      	  {
    	  SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TDTR, CAN_TDT0R_TGT);
      	  }
      	  /* Set up the data field */
      	  WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDHR,
                ((uint32_t)aData[7] << CAN_TDH0R_DATA7_Pos) |
                ((uint32_t)aData[6] << CAN_TDH0R_DATA6_Pos) |
                ((uint32_t)aData[5] << CAN_TDH0R_DATA5_Pos) |
                ((uint32_t)aData[4] << CAN_TDH0R_DATA4_Pos));
      	  WRITE_REG(hcan->Instance->sTxMailBox[transmitmailbox].TDLR,
                ((uint32_t)aData[3] << CAN_TDL0R_DATA3_Pos) |
                ((uint32_t)aData[2] << CAN_TDL0R_DATA2_Pos) |
                ((uint32_t)aData[1] << CAN_TDL0R_DATA1_Pos) |
                ((uint32_t)aData[0] << CAN_TDL0R_DATA0_Pos));

      	  SET_BIT(hcan->Instance->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);
      	  return HAL_OK;
    	}
    }
  }
  return HAL_ERROR;
}


uint16_t CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan)
{
  uint16_t freelevel = 0U;

  if ((hcan->State == HAL_CAN_STATE_READY) || (hcan->State == HAL_CAN_STATE_LISTENING))
  {
    /* Check Tx Mailbox 0 status */
    if ((hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
    {
      freelevel++;
    }
    /* Check Tx Mailbox 1 status */
    if ((hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
    {
      freelevel++;
    }
    /* Check Tx Mailbox 2 status */
    if ((hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
    {
      freelevel++;
    }
  }
  return ( freelevel );
}


/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       CANmodule: CAN module instance
 * \param[in]       buffer: Pointer to buffer to transmit
 */

static  CAN_TxHeaderTypeDef pTXHeader ={0,0,CAN_ID_STD,0,0,DISABLE};

static uint32_t prv_send_can_message(CO_CANmodule_t* CANmodule, CO_CANtx_t *buffer) {

    /* Check if TX FIFO is ready to accept more messages */
    if (CAN_GetTxMailboxesFreeLevel(CANmodule->CANptr) > 0)
    {
        pTXHeader.DLC                = (uint32_t)buffer->DLC;
        pTXHeader.RTR                = (buffer->ident & FLAG_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
        pTXHeader.StdId              = buffer->ident & CANID_MASK;

        /* Now add message to FIFO. Should not fail */
        if (CAN_AddTxMessage(CANmodule->CANptr,  &pTXHeader, buffer->data) == HAL_OK)
        {
        	return 1U;
        }
    }
    return 0U;
}



/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    /* if CAN TX buffer is free, copy message to it */
    if (prv_send_can_message(CANmodule, buffer) ) {
          CANmodule->bufferInhibitFlag = buffer->syncFlag;
      } else {
          buffer->bufferFull = true;
          CANmodule->CANtxCount++;
      }
    CO_UNLOCK_CAN_SEND(CANmodule);
    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);


    if(tpdoDeleted != 0U){
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}


/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
    * different way to determine errors. */
static uint16_t rxErrors=0, txErrors=0, overflow=0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
    uint32_t err;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U) {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        }
        else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0) {
            /* CAN RX bus overflow */
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
}


/******************************************************************************/




 void CAN_SendMessage()
{
	CANModule_local->firstCANtxMessage = false;
	CANModule_local->bufferInhibitFlag = false;
    if(CANModule_local->CANtxCount > 0U)
    {
        uint16_t i;             /* index of transmitting message */
        CO_CANtx_t *buffer = &CANModule_local->txArray[0];
        CO_LOCK_CAN_SEND(CANModule_local);

        for (i = CANModule_local->txSize; i > 0U; --i, ++buffer) {
                   /* Try to send message */
                   if (buffer->bufferFull) {
                       if (prv_send_can_message(CANModule_local, buffer)) {
                           buffer->bufferFull = false;
                           CANModule_local->CANtxCount--;
                           CANModule_local->bufferInhibitFlag = buffer->syncFlag;
                       }
                   }
               }
        /* Clear counter if no more messages */
        if(i == 0U){
        	CANModule_local->CANtxCount = 0U;
        }
        CO_UNLOCK_CAN_SEND(CANModule_local);
    }
}

/*
 *
 */

HAL_StatusTypeDef CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo,  CO_CANrxMsg_t * pCANData)
{
  HAL_CAN_StateTypeDef state = hcan->State;

  if ((state == HAL_CAN_STATE_READY) ||
      (state == HAL_CAN_STATE_LISTENING))
  {
    /* Check the Rx FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Check that the Rx FIFO 0 is not empty */
      if ((hcan->Instance->RF0R & CAN_RF0R_FMP0) == 0U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

        return HAL_ERROR;
      }
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Check that the Rx FIFO 1 is not empty */
      if ((hcan->Instance->RF1R & CAN_RF1R_FMP1) == 0U)
      {
        /* Update error code */
        hcan->ErrorCode |= HAL_CAN_ERROR_PARAM;

        return HAL_ERROR;
      }
    }

    if ( ( CAN_RI0R_IDE & hcan->Instance->sFIFOMailBox[RxFifo].RIR ) == CAN_ID_STD)
    {

    	pCANData->ident = (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[RxFifo].RIR) >> CAN_TI0R_STID_Pos |
    		((CAN_RI0R_RTR & hcan->Instance->sFIFOMailBox[RxFifo].RIR) == CAN_RTR_REMOTE ? FLAG_RTR : 0x00);
    	pCANData->dlc = (CAN_RDT0R_DLC & hcan->Instance->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_DLC_Pos;
    	pCANData->data[0] = (uint8_t)((CAN_RDL0R_DATA0 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA0_Pos);
    	pCANData->data[1] = (uint8_t)((CAN_RDL0R_DATA1 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA1_Pos);
    	pCANData->data[2] = (uint8_t)((CAN_RDL0R_DATA2 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA2_Pos);
    	pCANData->data[3] = (uint8_t)((CAN_RDL0R_DATA3 & hcan->Instance->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA3_Pos);
    	pCANData->data[4] = (uint8_t)((CAN_RDH0R_DATA4 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA4_Pos);
    	pCANData->data[5] = (uint8_t)((CAN_RDH0R_DATA5 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA5_Pos);
    	pCANData->data[6] = (uint8_t)((CAN_RDH0R_DATA6 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA6_Pos);
    	pCANData->data[7] = (uint8_t)((CAN_RDH0R_DATA7 & hcan->Instance->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA7_Pos);
    }
    /* Release the FIFO */
    if (RxFifo == CAN_RX_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
      /* Release RX FIFO 0 */
      SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
      /* Release RX FIFO 1 */
      SET_BIT(hcan->Instance->RF1R, CAN_RF1R_RFOM1);
    }

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    /* Update error code */
    hcan->ErrorCode |= HAL_CAN_ERROR_NOT_INITIALIZED;

    return HAL_ERROR;
  }
}



 void  prv_read_can_received_msg(CAN_HandleTypeDef* can, uint32_t fifo) {

    static CO_CANrxMsg_t rcvMsg;
    if (CAN_GetRxMessage(can, fifo,  &rcvMsg) == HAL_OK)
    {
    	CO_CANrx_t * buffer = CANModule_local->rxArray;
    	    for (uint8_t index = CANModule_local->rxSize; index > 0U; --index, ++buffer)
    	    {
    	         if (((rcvMsg.ident ^ buffer->ident) & buffer->mask) == 0U)
    	         {
    	              if (buffer != NULL && buffer->CANrx_callback != NULL)
    	              {
    	                  buffer->CANrx_callback(buffer->object, (void*) &rcvMsg);
    	              }
    	              break;
    	          }
    	    }
    }
    return;
}
/*
 *
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	prv_read_can_received_msg(hcan, CAN_RX_FIFO0);
}
/*
 *
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	prv_read_can_received_msg(hcan, CAN_RX_FIFO1);
}


void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	if __HAL_CAN_GET_FLAG(hcan,CAN_FLAG_BOF )
	{
		vRestartNode();
	}
}

