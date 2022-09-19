/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
const int16_t CANTXDATALONG = 8;  //CAN发送数组长度
const int16_t CANRXDATALONG = 8;  //CAN接受数组长度
volatile uint8_t CanTxData[CANTXDATALONG]={0};  //CAN发送数组
volatile uint8_t CanRxData[CANRXDATALONG]={0};  //CAN接受数组
volatile uint8_t drive_State=0;       //驱动板状态码
volatile uint8_t drive_Temperature=0; //驱动板温度
volatile uint8_t drive_Power=0;       //驱动板功率负载
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	
	Filter_Init();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void Can_senddata(char *buf, int len)
{
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t mailBoxId = 0;
	HAL_StatusTypeDef statetypedef = 0;
	
	if(len>8) len=8;
	
	TxHeader.StdId = 0x001;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = len;
	TxHeader.TransmitGlobalTime = DISABLE;
	
	statetypedef = HAL_CAN_AddTxMessage(&hcan1,&TxHeader,buf,&mailBoxId);
	
	//return statetypedef;
}

void Filter_Init()
{
	CAN_FilterTypeDef sFilterCfg;
	sFilterCfg.FilterActivation = CAN_FILTER_ENABLE;
	sFilterCfg.FilterBank = 0;
	sFilterCfg.FilterMode = CAN_FILTERMODE_IDMASK; //????
	sFilterCfg.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterCfg.FilterFIFOAssignment = CAN_FilterFIFO0;
	
	//?????
	sFilterCfg.FilterMaskIdHigh = 0x0000;
	sFilterCfg.FilterMaskIdLow = 0x0000;
	sFilterCfg.FilterIdHigh = 0x0000;
	sFilterCfg.FilterIdLow =0x0000;
	HAL_CAN_ConfigFilter(&hcan1,&sFilterCfg);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
		
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanRxData) != HAL_OK)
	{
		Error_Handler();
	}	

  if (CanRxData[6]==even_parity(CanRxData, 0, 5))//校验成功读取数据
  {
    drive_State=CanRxData[1];       //驱动板状态码
    drive_Temperature=CanRxData[2]; //驱动板温度
    drive_Power=CanRxData[3];       //驱动板功率负载
  }
  
}

/* USER CODE END 1 */
