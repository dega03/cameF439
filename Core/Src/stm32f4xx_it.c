/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Swim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

extern uint32_t SWIMStatus;
extern uint32_t bufferRxDMA[1024];
extern uint32_t bufferTxDMA[256];
extern uint32_t bufferRxDMAPtr;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */
	//
	//   DMA1 stream 5 = Tim3 Ch2  -> Led green
	uint32_t PulseLen;
	//

  /* USER CODE END DMA1_Stream5_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */
	LL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);													//Debug
	LL_DMA_ClearFlag_TC5(DMA1);
	LL_DMA_ClearFlag_TC6(DMA1);
	LL_TIM_DisableCounter(SWIM_TIM);
	LL_TIM_DisableCounter(SWIM_CC_TIM);
	LL_TIM_DisableDMAReq_UPDATE(SWIM_TIM);
	//LL_DMA_DisableStream(DMA1, DMA_PWM);
	//LL_DMA_DisableStream(DMA1, DMA_InpCapt);  already disabled if here


	if ((SWIMStatus & SWIM_ReadCmdAnswrMsk ) == SWIM_ReadCmdAnswr) {
	  PulseLen = (SWIMStatus & ~SWIM_ReadCmdAnswrMsk) >> SWIM_ReadCmdAnswrShf;  //N Bytes left

	  if (PulseLen == 1) {
		  SWIMStatus = SWIM_Idle;
	  	  LL_TIM_DisableDMAReq_CC2(SWIM_CC_TIM);
	  } else {
		  SWIMStatus = SWIM_ReadCmdAnswr | (PulseLen -1 ) << SWIM_ReadCmdAnswrShf;
		  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 11); //ack + 0 + b7.. b0 + p
		  LL_DMA_SetMemoryAddress(DMA1, DMA_InpCapt, bufferRxDMAPtr);
		  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
		  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
		  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		  LL_TIM_EnableCounter(SWIM_CC_TIM);

	  }

	  bufferRxDMAPtr = bufferRxDMAPtr + 24;
	  LL_DMA_SetDataLength(DMA1, DMA_PWM, 2);
	  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)bufferTxDMA);
	  LL_DMA_EnableStream(DMA1, DMA_PWM);
	  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
	  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
	  LL_TIM_EnableCounter(SWIM_TIM);

	} else {
		if ((SWIMStatus & SWIM_CmdMask) == SWIM_Cmd5BitCmd)
			PulseLen = bufferRxDMA[2]>>16;
		else
			PulseLen = bufferRxDMA[5] & 0xffff;

		if (PulseLen > 10 && PulseLen < 50) {
			switch (SWIMStatus) {
			  case  SWIM_RST:   //Here if SWIM Reset
				SWIMStatus = SWIM_Idle;
				break;
			  case  SWIM_ReadCmd:   //Here if SWIM Read
				SWIMStatus = SWIM_ReadCmdAck;
				break;
			  case  SWIM_ReadCmdData:
				SWIMStatus = SWIM_ReadCmdDataAck;
				break;
			  case  SWIM_WriteCmd:   //Here if SWIM Write
				SWIMStatus = SWIM_WriteCmdAck;
				break;
			  case  SWIM_WriteCmdData:
				SWIMStatus = SWIM_WriteCmdDataAck;
				break;
			  default:   //Here for other states
				SWIMStatus = SWIM_Err | SWIMStatus;
				break;
			}
		} else
			SWIMStatus = SWIM_AckErr | SWIMStatus;
	}

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

	// Here for DMA interrupt TIM4_UP

  /* USER CODE END DMA1_Stream6_IRQn 0 */

  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */
	//Code gets here only at init sequence  -> Led RED
        LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);													//Debug
		LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		LL_TIM_EnableCounter(SWIM_CC_TIM);
		LL_DMA_DisableStream(DMA1, DMA_PWM);
		LL_DMA_ClearFlag_TC6(DMA1);
		SWIMStatus = SWIM_InitAck;
		LL_TIM_EnableIT_CC2(SWIM_CC_TIM);    //enable interrupt for capture compare
		LL_DMA_DisableIT_TC(DMA1, DMA_PWM);

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
	LL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);                                          //Debug

   //Here just for initial measure of pulse low after init sequence
	if(LL_TIM_IsActiveFlag_CC2(SWIM_CC_TIM)) {
		LL_TIM_ClearFlag_CC2(SWIM_CC_TIM); //After a period measurement
		if (SWIM_CC_TIM->CCR2 < 1550 && SWIM_CC_TIM->CCR2 > 1250) {  // 16us @ 88MHz +/- 10%
			LL_TIM_DisableIT_CC2(SWIM_CC_TIM);  //disable interrupt for capture compare
			if (SWIMStatus<SWIM_Idle) {
				LL_TIM_DisableCounter(SWIM_TIM);
				LL_TIM_SetCounterMode(SWIM_TIM,LL_TIM_COUNTERMODE_DOWN);
				LL_TIM_OC_SetCompareCH3(SWIM_TIM, 0xffff);   //To make sure it goes high
				LL_TIM_SetAutoReload(SWIM_TIM, 241 ); //242 = 22 ck * (88MHz/8MHz) then -1 !!!
			}
			SWIMStatus = SWIM_Idle;
		} else
			SWIMStatus = SWIM_InitErr;
	}
	if (LL_TIM_IsActiveFlag_CC4(SWIM_CC_TIM)) {  	// If here something went wrong with SWIM
		LL_TIM_ClearFlag_CC4(SWIM_CC_TIM);
		LL_TIM_DisableCounter(SWIM_TIM);
		LL_DMA_DisableStream(DMA1, DMA_InpCapt);
		LL_DMA_DisableStream(DMA1, DMA_PWM);
		SWIMStatus = SWIM_Timeout;
//		__asm__("BKPT");
	}
	LL_TIM_DisableDMAReq_CC2(SWIM_CC_TIM);
	LL_TIM_DisableCounter(SWIM_CC_TIM);
	
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles Ethernet global interrupt.
  */
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
