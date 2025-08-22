/*
 * Swim.c
 *
 *  Created on: Jun 29, 2020
 *      Author: Marco
 */

#include "Swim.h"

uint32_t bufferRxDMA[1024];
uint32_t bufferTxDMA[256]; // = { 0xABDF,0xABDF,0xABDF,0xABDF,0x55f055f0,0x55f055f0,0xffffffff,0xffffffff};
uint32_t SWIMStatus;
uint32_t bufferRxDMAPtr;

uint32_t * PrepareTxData (uint32_t DataIn,  uint32_t * BufferPtr ) {
	char Parity = 0;

	BufferPtr[0] = 0x0000001A;

	if ((DataIn & 0x80) == 0) //Bit 7
		BufferPtr[0] |= 0x001A0000;
	else {
		BufferPtr[0] |= 0x00D80000;
		Parity ^= 1;
	}
	if ((DataIn & 0x40) == 0) //Bit 6
		BufferPtr[1] = 0x0000001A;
	else {
		BufferPtr[1] = 0x000000D8;
		Parity ^= 1;
	}
	if ((DataIn & 0x20) == 0) //Bit 5
		BufferPtr[1] |= 0x001A0000;
	else {
		BufferPtr[1] |= 0x00D80000;
		Parity ^= 1;
	}
	if ((DataIn & 0x10) == 0) //Bit 4
		BufferPtr[2] = 0x0000001A;
	else {
		BufferPtr[2] = 0x000000D8;
		Parity ^= 1;
	}
	if ((DataIn & 0x08) == 0) //Bit 3
		BufferPtr[2] |= 0x001A0000;
	else {
		BufferPtr[2] |= 0x00D80000;
		Parity ^= 1;
	}
	if ((DataIn & 0x04) == 0) //Bit 2
		BufferPtr[3] = 0x0000001A;
	else {
		BufferPtr[3] = 0x000000D8;
		Parity ^= 1;
	}
	if ((DataIn & 0x02) == 0) //Bit 1
		BufferPtr[3] |= 0x001A0000;
	else {
		BufferPtr[3] |= 0x00D80000;
		Parity ^= 1;
	}
	if ((DataIn & 0x01) == 0) //Bit 0
		BufferPtr[4] = 0x0000001A;
	else {
		BufferPtr[4] = 0x000000D8;
		Parity ^= 1;
	}
	if (Parity == 0)           //Parity
		BufferPtr[4] |= 0x001A0000;
	else
		BufferPtr[4] |= 0x00D80000;
	BufferPtr[5] = 0xffffffff;  //Last 2 data set to FF to have line high for ACK

	BufferPtr += 6; //Update pointer for next
	return BufferPtr;

}

void SWIM_Read(uint32_t addr, uint8_t Ndata, uint16_t * DataOut) {

	  uint32_t * BufferPtr;
	  uint32_t * BufferPtr2;
	  uint32_t   NdataProcessed;
	  uint32_t   Parity, BuffDecodePtr;
	  uint16_t   CurrByte;

	  BufferPtr = bufferTxDMA;
	  memset(DataOut, 0xffff,Ndata * 2);

	  if (Ndata == 0 || SWIMStatus != SWIM_Idle)
		  return;

//	  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_ResetOutputPin(LD1_GPIO_Port, LD1_Pin);

	  LL_TIM_DisableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_DisableDMAReq_CC2(SWIM_CC_TIM);
	  LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);

	  SWIMStatus = SWIM_ReadCmd;
	  *BufferPtr++ = 0x001A001A;  //DC for 220 (0),  0x16 for 22 (1)  => 1A or D8
	  *BufferPtr++ = 0x00D8001A;
	  *BufferPtr++ = 0xffff00D8;  //6th bit is to make sure PWM stays high
	  *BufferPtr++ = 0xffffffff;  //7-8 th bits is to make sure PWM stays high
	  LL_DMA_SetDataLength(DMA1, DMA_PWM, 6);  //6th is to set FF to wait for ACK pulse, 7th to fill pre-buffer
	  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)bufferTxDMA);
	  LL_DMA_ClearFlag_TC6(DMA1);
	  LL_DMA_ClearFlag_FE6(DMA1); //Sometimes this error appears
	  LL_DMA_ClearFlag_TE6(DMA1); //Sometimes this error appears

	  LL_DMA_EnableStream(DMA1, DMA_PWM);

	  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 6); //0 + b2 + b1 + b0 + p + ack
	  LL_DMA_SetMemoryAddress(DMA1, DMA_InpCapt, (uint32_t)bufferRxDMA);

	  LL_DMA_ClearFlag_TC5(DMA1);                   //Clearing TC just in case
	  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
	  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
	  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
	  LL_TIM_EnableCounter(SWIM_CC_TIM);



	  //LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
	  //LL_TIM_CC_EnableChannel(SWIM_TIM, SWIM_CH);
	  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
	  //LL_TIM_OC_SetCompareCH3(SWIM_TIM, 42591);  //First low pulse, 16us
	  LL_TIM_EnableCounter(SWIM_TIM);

	  //Prepare next data
	  BufferPtr2 = PrepareTxData(Ndata,BufferPtr); //N

	  while(SWIMStatus == SWIM_ReadCmd)
		  ;
//		  if (LL_DMA_IsActiveFlag_FE2(DMA1) || LL_DMA_IsActiveFlag_TE2(DMA1))
//				  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
	  if (SWIMStatus == SWIM_ReadCmdAck) {  // Send N
		  SWIMStatus = SWIM_ReadCmdData;
		  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
		  LL_DMA_SetDataLength(DMA1, DMA_PWM, 11);  //11th is to set FF to wait for ACK pulse
		  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)BufferPtr);
		  BufferPtr = BufferPtr2;
		  //LL_DMA_ClearFlag_TC2(DMA1);
		  LL_DMA_EnableStream(DMA1, DMA_PWM);

		  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 11); //0 + b7.. b0 + p + ack
		  //LL_DMA_ClearFlag_TC3(DMA1);                   //Clearing TC just in case
		  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
		  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
		  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
		  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		  LL_TIM_EnableCounter(SWIM_CC_TIM);
		  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
		  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
		  LL_TIM_EnableCounter(SWIM_TIM);

		  BufferPtr2 = PrepareTxData(addr >> 16,BufferPtr); //E

	  }
	  while(SWIMStatus == SWIM_ReadCmdData)
		  ;
//		  if (LL_DMA_IsActiveFlag_FE2(DMA1) || LL_DMA_IsActiveFlag_TE2(DMA1))
//			  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);

	  if (SWIMStatus == SWIM_ReadCmdDataAck) {  // Send E
		  SWIMStatus = SWIM_ReadCmdData;
		  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
		  LL_DMA_SetDataLength(DMA1, DMA_PWM, 11);  //11th is to set FF to wait for ACK pulse
		  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)BufferPtr);
		  BufferPtr = BufferPtr2;
		  //LL_DMA_ClearFlag_TC2(DMA1);
		  LL_DMA_EnableStream(DMA1, DMA_PWM);

		  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 11); //0 + b7.. b0 + p + ack
		  //LL_DMA_ClearFlag_TC3(DMA1);                   //Clearing TC just in case
		  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
		  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
		  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
		  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		  LL_TIM_EnableCounter(SWIM_CC_TIM);
		  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
		  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
		  LL_TIM_EnableCounter(SWIM_TIM);

		  BufferPtr2 = PrepareTxData((addr >> 8) & 0xff ,BufferPtr); //H
	  }
	  while(SWIMStatus == SWIM_ReadCmdData)
		  ;
//		  if (LL_DMA_IsActiveFlag_FE2(DMA1) || LL_DMA_IsActiveFlag_TE2(DMA1))
//				  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);

	  if (SWIMStatus == SWIM_ReadCmdDataAck) {  //Send H
		  SWIMStatus = SWIM_ReadCmdData;
		  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
		  LL_DMA_SetDataLength(DMA1, DMA_PWM, 11);  //11th is to set FF to wait for ACK pulse
		  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)BufferPtr);
		  BufferPtr = BufferPtr2;
		  //LL_DMA_ClearFlag_TC2(DMA1);
		  LL_DMA_EnableStream(DMA1, DMA_PWM);

		  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 11); //0 + b7.. b0 + p + ack
		  //LL_DMA_ClearFlag_TC3(DMA1);                   //Clearing TC just in case
		  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
		  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
		  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
		  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		  LL_TIM_EnableCounter(SWIM_CC_TIM);
		  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
		  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
		  LL_TIM_EnableCounter(SWIM_TIM);

		  BufferPtr2 = PrepareTxData( addr & 0xff ,BufferPtr); //L
	  }
	  while(SWIMStatus == SWIM_ReadCmdData)
		  ;
//		  if (LL_DMA_IsActiveFlag_FE2(DMA1) || LL_DMA_IsActiveFlag_TE2(DMA1))
//				  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);

	  if (SWIMStatus == SWIM_ReadCmdDataAck) {  //Send L
		  SWIMStatus = SWIM_ReadCmdAnswr | (uint32_t)Ndata << SWIM_ReadCmdAnswrShf;
		  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
		  LL_DMA_SetDataLength(DMA1, DMA_PWM, 11);  //11th is to set FF to wait for ACK pulse
		  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)BufferPtr);
		  BufferPtr = BufferPtr2;
		  //LL_DMA_ClearFlag_TC2(DMA1);
		  LL_DMA_EnableStream(DMA1, DMA_PWM);

		  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 21); //0 + b7.. b0 + p + ack
		  bufferRxDMAPtr = (uint32_t)bufferRxDMA + 44;

		  //LL_DMA_ClearFlag_TC3(DMA1);                   //Clearing TC just in case
		  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
		  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
		  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
		  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		  LL_TIM_EnableCounter(SWIM_CC_TIM);
		  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
		  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
		  LL_TIM_EnableCounter(SWIM_TIM);

		  //Prepare buffer for tx the ACK pulse
		  bufferTxDMA[0] = 0xffff00D8;
		  bufferTxDMA[1] = 0xffffffff;

	  }
	  NdataProcessed = 0;

	  while(((SWIMStatus & SWIM_ReadCmdAnswrMsk ) == SWIM_ReadCmdAnswr) || ((NdataProcessed < Ndata ) && ((SWIMStatus & SWIM_ErrMask) == 0 ))) {
//		  if (LL_DMA_IsActiveFlag_FE2(DMA1) || LL_DMA_IsActiveFlag_TE2(DMA1))
//				  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
		  if (NdataProcessed < ( Ndata - ((SWIMStatus & ~SWIM_ReadCmdAnswrMsk) >> SWIM_ReadCmdAnswrShf))) {
			  BuffDecodePtr = 6 + (6*NdataProcessed);
			  CurrByte = 0;
			  Parity = 0;
			  if ((bufferRxDMA[BuffDecodePtr] & 0xffff) <= 0x40 ) {
				  CurrByte |= 0x80;
				  Parity ^= 1;
			  }
			  if ((bufferRxDMA[BuffDecodePtr++] >> 16) <= 0x40 ) {
				  CurrByte |= 0x40;
				  Parity ^= 1;
			  }
			  if ((bufferRxDMA[BuffDecodePtr] & 0xffff) <= 0x40 ) {
				  CurrByte |= 0x20;
				  Parity ^= 1;
			  }
			  if ((bufferRxDMA[BuffDecodePtr++] >> 16) <= 0x40 ) {
				  CurrByte |= 0x10;
				  Parity ^= 1;
			  }
			  if ((bufferRxDMA[BuffDecodePtr] & 0xffff) <= 0x40 ) {
				  CurrByte |= 0x08;
				  Parity ^= 1;
			  }
			  if ((bufferRxDMA[BuffDecodePtr++] >> 16) <= 0x40 ) {
				  CurrByte |= 0x04;
				  Parity ^= 1;
			  }
			  if ((bufferRxDMA[BuffDecodePtr] & 0xffff) <= 0x40 ) {
				  CurrByte |= 0x02;
				  Parity ^= 1;
			  }
			  if ((bufferRxDMA[BuffDecodePtr++] >> 16) <= 0x40 ) {
				  CurrByte |= 0x01;
				  Parity ^= 1;
			  }
			  if ((bufferRxDMA[BuffDecodePtr] & 0xffff) <= 0x40 ) {
				 if (Parity == 0)
					 CurrByte = 0xffff;
			  } else {
				 if (Parity == 1)
					 CurrByte = 0xffff;
			  }
			  DataOut[NdataProcessed] = CurrByte;
			  NdataProcessed++;
		  }
	  }
	  while (LL_DMA_IsEnabledStream(DMA1, DMA_PWM))
		  ;
//	  if ((SWIMStatus & SWIM_ErrMask) == 0 ) {  //Trap for debug
//		  SWIMStatus = SWIM_Idle;
//	  }
	  LL_TIM_DisableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_DisableDMAReq_CC2(SWIM_CC_TIM);

	  LL_TIM_DisableCounter(SWIM_TIM);		//Disable counter to be ready for the next command
//	  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);


}

void SWIM_Write(uint32_t addr, uint8_t Ndata, uint16_t * DataIn) {

	  uint32_t * BufferPtr;
	  uint32_t * BufferPtr2;
	  BufferPtr = bufferTxDMA;

	  if (Ndata == 0 || SWIMStatus != SWIM_Idle)
		  return;

//	  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_ResetOutputPin(LD1_GPIO_Port, LD1_Pin);

	  LL_TIM_DisableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_DisableDMAReq_CC2(SWIM_CC_TIM);
	  LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);

	  SWIMStatus = SWIM_WriteCmd;
	  *BufferPtr++ = 0x001A001A;  //DC for 220 (0),  0x16 for 22 (1)  => 1A or D8
	  *BufferPtr++ = 0x001A00D8;
	  *BufferPtr++ = 0xffff00D8;  //6th bit is to make sure PWM stays high
	  *BufferPtr++ = 0xffffffff;  //7-8 th bits is to make sure PWM stays high
	  LL_DMA_SetDataLength(DMA1, DMA_PWM, 6);  //6th is to set FF to wait for ACK pulse, 7th to fill pre-buffer
	  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)bufferTxDMA);  //Already initialized
	  //LL_DMA_SetPeriphAddress(DMA1, DMA_PWM, (uint32_t)&SWIM_TIM->CCR3);
	  LL_DMA_ClearFlag_TC6(DMA1);
	  LL_DMA_ClearFlag_FE6(DMA1); //Sometimes this error appears

	  LL_DMA_EnableStream(DMA1, DMA_PWM);

//	  LL_DMA_EnableIT_TC(DMA1, DMA_InpCapt);
	  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 6); //0 + b2 + b1 + b0 + p + ack
	  LL_DMA_SetMemoryAddress(DMA1, DMA_InpCapt, (uint32_t)bufferRxDMA);
//	  LL_DMA_SetPeriphAddress(DMA1, DMA_InpCapt, (uint32_t)&SWIM_CC_TIM->CCR2);
	  LL_DMA_ClearFlag_TC5(DMA1);                   //Clearing TC just in case
	  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
	  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
//	  LL_TIM_EnableIT_CC4(SWIM_CC_TIM);
	  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
	  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
	  LL_TIM_EnableCounter(SWIM_CC_TIM);



	  //LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
	  //LL_TIM_CC_EnableChannel(SWIM_TIM, SWIM_CH);
	  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
	  //LL_TIM_OC_SetCompareCH3(SWIM_TIM, 42591);  //First low pulse, 16us
	  LL_TIM_EnableCounter(SWIM_TIM);

	  //Prepare next data
	  BufferPtr2 = PrepareTxData(Ndata,BufferPtr); //N

	  while(SWIMStatus == SWIM_WriteCmd)
		  ;
//	  if (LL_DMA_IsActiveFlag_FE2(DMA1) || LL_DMA_IsActiveFlag_TE2(DMA1))
//			  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
	  if (SWIMStatus == SWIM_WriteCmdAck) {  // Send N
		  SWIMStatus = SWIM_WriteCmdData;
		  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
		  LL_DMA_SetDataLength(DMA1, DMA_PWM, 11);  //11th is to set FF to wait for ACK pulse
		  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)BufferPtr);
		  BufferPtr = BufferPtr2;
		  //LL_DMA_ClearFlag_TC2(DMA1);
		  LL_DMA_EnableStream(DMA1, DMA_PWM);

		  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 11); //0 + b7.. b0 + p + ack
		  //LL_DMA_ClearFlag_TC3(DMA1);                   //Clearing TC just in case
		  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
		  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
		  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
		  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		  LL_TIM_EnableCounter(SWIM_CC_TIM);
		  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
		  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
		  LL_TIM_EnableCounter(SWIM_TIM);

		  BufferPtr2 = PrepareTxData(addr >> 16,BufferPtr); //E

	  }
	  while(SWIMStatus == SWIM_WriteCmdData)
		  ;

	  if (SWIMStatus == SWIM_WriteCmdDataAck) {  // Send E
		  SWIMStatus = SWIM_WriteCmdData;
		  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
		  LL_DMA_SetDataLength(DMA1, DMA_PWM, 11);  //11th is to set FF to wait for ACK pulse
		  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)BufferPtr);
		  BufferPtr = BufferPtr2;
		  //LL_DMA_ClearFlag_TC2(DMA1);
		  LL_DMA_EnableStream(DMA1, DMA_PWM);

		  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 11); //0 + b7.. b0 + p + ack
		  //LL_DMA_ClearFlag_TC3(DMA1);                   //Clearing TC just in case
		  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
		  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
		  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
		  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		  LL_TIM_EnableCounter(SWIM_CC_TIM);
		  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
		  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
		  LL_TIM_EnableCounter(SWIM_TIM);

		  BufferPtr2 = PrepareTxData((addr >> 8) & 0xff ,BufferPtr); //H
	  }
	  while(SWIMStatus == SWIM_WriteCmdData)
		  ;

	  if (SWIMStatus == SWIM_WriteCmdDataAck) {  //Send H
		  SWIMStatus = SWIM_WriteCmdData;
		  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
		  LL_DMA_SetDataLength(DMA1, DMA_PWM, 11);  //11th is to set FF to wait for ACK pulse
		  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)BufferPtr);
		  BufferPtr = BufferPtr2;
		  //LL_DMA_ClearFlag_TC2(DMA1);
		  LL_DMA_EnableStream(DMA1, DMA_PWM);

		  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 11); //0 + b7.. b0 + p + ack
		  //LL_DMA_ClearFlag_TC3(DMA1);                   //Clearing TC just in case
		  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
		  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
		  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
		  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		  LL_TIM_EnableCounter(SWIM_CC_TIM);
		  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
		  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
		  LL_TIM_EnableCounter(SWIM_TIM);

		  BufferPtr2 = PrepareTxData( addr & 0xff ,BufferPtr); //L
	  }
	  while(SWIMStatus == SWIM_WriteCmdData)
		  ;

	  while (Ndata != 0xff && SWIM_WriteCmdDataAck) {  //First time will send Addredss L
		  SWIMStatus = SWIM_WriteCmdData;
		  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
		  LL_DMA_SetDataLength(DMA1, DMA_PWM, 11);  //11th is to set FF to wait for ACK pulse
		  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)BufferPtr);
		  BufferPtr = BufferPtr2;
		  //LL_DMA_ClearFlag_TC2(DMA1);
		  LL_DMA_EnableStream(DMA1, DMA_PWM);

		  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 11); //0 + b7.. b0 + p + ack
		  //LL_DMA_ClearFlag_TC3(DMA1);                   //Clearing TC just in case
		  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
		  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
		  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
		  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
		  LL_TIM_EnableCounter(SWIM_CC_TIM);
		  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
		  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
		  LL_TIM_EnableCounter(SWIM_TIM);

		  if (Ndata != 0xff)
			  BufferPtr2 = PrepareTxData((uint32_t) (*DataIn),BufferPtr); //Data

		  if (Ndata != 0x00)
			  Ndata--;
		  else
			  Ndata = 0xff;

		  DataIn++;

		  while(SWIMStatus == SWIM_WriteCmdData)
			  ;
	  }

	  //Clear status if writing went ok
	  if (SWIMStatus == SWIM_WriteCmdDataAck)
		  SWIMStatus = SWIM_Idle;

}
void SWIMInit (void) {

	  unsigned int Data[16];

	  SWIMStatus = 0;
	  bufferTxDMA[0] = 0xABDF;
	  bufferTxDMA[1] = 0xABDF;
	  bufferTxDMA[2] = 0xABDF;
	  bufferTxDMA[3] = 0xABDF;
	  bufferTxDMA[4] = 0x55f055f0;
	  bufferTxDMA[5] = 0x55f055f0;
	  bufferTxDMA[6] = 0xffffffff;
	  bufferTxDMA[7] = 0xffffffff;

	  LL_TIM_DisableCounter(SWIM_TIM);
	  LL_TIM_DisableCounter(SWIM_CC_TIM);
	  LL_TIM_DisableDMAReq_CC2(SWIM_CC_TIM);
	  LL_TIM_DisableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_ClearFlag_UPDATE(SWIM_TIM);
	  LL_TIM_ClearFlag_CC1(SWIM_TIM);
	  LL_TIM_ClearFlag_CC2(SWIM_TIM);
	  LL_TIM_ClearFlag_CC3(SWIM_TIM);
	  LL_TIM_ClearFlag_CC4(SWIM_TIM);
	  LL_DMA_DisableStream(DMA1, DMA_PWM);
	  LL_DMA_DisableStream(DMA1, DMA_InpCapt);
	  LL_TIM_CC_DisableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);  //enabled it later, after the init sequence
	  LL_TIM_SetAutoReload(SWIM_TIM, 43999 );
	  LL_TIM_SetCounterMode(SWIM_TIM,LL_TIM_COUNTERMODE_UP);

	  LL_DMA_EnableIT_TC(DMA1, DMA_InpCapt);
	  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 1); //Double number than buffer size because half word transfer
	  LL_DMA_SetMemoryAddress(DMA1, DMA_InpCapt, (uint32_t)bufferRxDMA);
	  LL_DMA_SetPeriphAddress(DMA1, DMA_InpCapt, (uint32_t)&SWIM_CC_TIM->CCR2);
	  //LL_DMA_EnableStream(DMA1, DMA_InpCapt);
	  LL_TIM_OC_SetCompareCH4(SWIM_CC_TIM, K_TIMTIMEOUT);         //Timeout interrupt
	  LL_TIM_EnableIT_CC4(SWIM_CC_TIM);
	  //LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
	  //LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);  //enabled it later, after the init sequence
	  //LL_TIM_EnableCounter(SWIM_CC_TIM);

	  // Force output for PWM to High
	  LL_TIM_OC_SetMode(SWIM_TIM, SWIM_CH, LL_TIM_OCMODE_FORCED_ACTIVE);
	  LL_TIM_CC_EnableChannel(SWIM_TIM, SWIM_CH);
	  LL_TIM_SetCounter(SWIM_TIM, 0);

	  HAL_Delay(1);
	  LL_GPIO_ResetOutputPin(Rst_GPIO_Port, Rst_Pin);
	  HAL_Delay(1);

	  LL_DMA_EnableIT_TC(DMA1, DMA_PWM);
	  LL_DMA_SetDataLength(DMA1, DMA_PWM, 14);  //13 is to set FF to wait for ACK pulse, 14 is wo waint intill 13th is being pushed out
	  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)bufferTxDMA);
	  LL_DMA_SetPeriphAddress(DMA1, DMA_PWM, (uint32_t)&SWIM_TIM->CCR3);
	  LL_DMA_EnableStream(DMA1, DMA_PWM);

	  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_CC_EnableChannel(SWIM_TIM, SWIM_CH);
	  LL_TIM_SetCounter(SWIM_TIM, 0);
	  LL_TIM_OC_SetCompareCH3(SWIM_TIM, 42591);  //First low pulse, 16us
	  LL_TIM_EnableCounter(SWIM_TIM);
	  LL_TIM_OC_SetMode(SWIM_TIM, SWIM_CH, LL_TIM_OCMODE_PWM1);
	//LL_TIM_SetCounter(TIM_TypeDef *TIMx, uint32_t Counter)
	//LL_TIM_OC_SetCompareCH3(TIM_TypeDef *TIMx, uint32_t CompareValue)


	  while (SWIMStatus < SWIM_Idle)
		  ;
	  if(SWIMStatus == SWIM_Idle) {
		  SWIMRst();
		  HAL_Delay(5);
		  Data[0] = 0xA0;  //Safemask = 1 & SWIM debug mode = 1
		  SWIM_Write(0x7f80, 1,(uint16_t *)&Data);
		  LL_GPIO_SetOutputPin(Rst_GPIO_Port, Rst_Pin);
		  HAL_Delay(10);  //To wait to finish sending the data
		  LL_DMA_DisableIT_TC(DMA1, DMA_InpCapt);
	  } else {
		  LL_GPIO_SetOutputPin(Rst_GPIO_Port, Rst_Pin);
	  }

}

void SWIMCommRst(void) {
	  SWIMStatus = 0;
	  bufferTxDMA[0] = 0xffff0000;
	  bufferTxDMA[1] = 0xffffffff;

//	  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_ResetOutputPin(LD1_GPIO_Port, LD1_Pin);

	  LL_DMA_DisableStream(DMA1, DMA_PWM);
	  LL_DMA_DisableIT_TC(DMA1, DMA_PWM);
	  LL_DMA_DisableStream(DMA1, DMA_InpCapt);
	  LL_DMA_DisableIT_TC(DMA1, DMA_InpCapt);

	  LL_DMA_ClearFlag_TC6(DMA1);
	  LL_DMA_ClearFlag_TC5(DMA1);

	  LL_TIM_ClearFlag_CC2(SWIM_CC_TIM);
	  LL_TIM_ClearFlag_CC4(SWIM_CC_TIM);

	  LL_TIM_DisableCounter(SWIM_TIM);
	  LL_TIM_DisableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_DisableCounter(SWIM_CC_TIM);
	  LL_TIM_DisableDMAReq_CC2(SWIM_CC_TIM);

	  LL_TIM_CC_DisableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);  //enabled it later, after the init sequence
	  LL_TIM_SetAutoReload(SWIM_TIM, 1407 );   //128ck * (88MHz/8) -1
	  LL_TIM_SetCounterMode(SWIM_TIM,LL_TIM_COUNTERMODE_UP);

	  LL_DMA_EnableIT_TC(DMA1, DMA_InpCapt);
	  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 1); //Double number than buffer size because half word transfer
	  LL_DMA_SetMemoryAddress(DMA1, DMA_InpCapt, (uint32_t)bufferRxDMA);
	  LL_DMA_SetPeriphAddress(DMA1, DMA_InpCapt, (uint32_t)&SWIM_CC_TIM->CCR2);
	  //LL_DMA_EnableStream(DMA1, DMA_InpCapt);
	  LL_TIM_EnableIT_CC4(SWIM_CC_TIM);

	  LL_DMA_EnableIT_TC(DMA1, DMA_PWM);
	  LL_DMA_SetDataLength(DMA1, DMA_PWM, 3);  //2 is to set FF to wait for ACK pulse, 3 is to waint untill 13th is being pushed out
	  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)bufferTxDMA);
	  LL_DMA_SetPeriphAddress(DMA1, DMA_PWM, (uint32_t)&SWIM_TIM->CCR3);
	  LL_DMA_EnableStream(DMA1, DMA_PWM);

	  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_CC_EnableChannel(SWIM_TIM, SWIM_CH);
	  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
	  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
	  LL_TIM_EnableCounter(SWIM_TIM);

	  while(SWIMStatus < SWIM_Idle)  //Errors are also > SWIM_Idle
		  ;

}
void SWIMRst(void) {

	  if (SWIMStatus != SWIM_Idle)
		  return;
//	  LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
//	  LL_GPIO_ResetOutputPin(LD1_GPIO_Port, LD1_Pin);

	  LL_TIM_DisableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_DisableDMAReq_CC2(SWIM_CC_TIM);
	  LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);

	  SWIMStatus = SWIM_RST;
	  bufferTxDMA[0] = 0x001A001A;  //DC for 220 (0),  0x16 for 22 (1)
	  bufferTxDMA[1] = 0x001A001A;
	  bufferTxDMA[2] = 0xffff001A;  //6th bit is to make sure PWM stays high
	  bufferTxDMA[3] = 0xffffffff;  //7-8 th bits is to make sure PWM stays high
	  LL_DMA_SetDataLength(DMA1, DMA_PWM, 6);  //6th is to set FF to wait for ACK pulse, 7th to fill pre-buffer
	  LL_DMA_SetMemoryAddress(DMA1, DMA_PWM, (uint32_t)bufferTxDMA);
	  //LL_DMA_SetPeriphAddress(DMA1, DMA_PWM, (uint32_t)&SWIM_TIM->CCR3);
	  LL_DMA_ClearFlag_TC6(DMA1);
	  LL_DMA_ClearFlag_FE6(DMA1); //Sometimes this error appears

	  LL_DMA_EnableStream(DMA1, DMA_PWM);

//	  LL_DMA_EnableIT_TC(DMA1, DMA_InpCapt);
	  LL_DMA_SetDataLength(DMA1, DMA_InpCapt, 6); //0 + b2 + b1 + b0 + p + ack
	  LL_DMA_SetMemoryAddress(DMA1, DMA_InpCapt, (uint32_t)bufferRxDMA);
//	  LL_DMA_SetPeriphAddress(DMA1, DMA_InpCapt, (uint32_t)&SWIM_CC_TIM->CCR2);
	  LL_DMA_ClearFlag_TC5(DMA1);                   //Clearing TC just in case
	  LL_DMA_EnableStream(DMA1, DMA_InpCapt);
	  LL_TIM_SetCounter(SWIM_CC_TIM, 0);                    //Initialize counter for SWIM_CC_TIM
//	  LL_TIM_EnableIT_CC4(SWIM_CC_TIM);
	  LL_TIM_EnableDMAReq_CC2(SWIM_CC_TIM);
//	  LL_TIM_CC_EnableChannel(SWIM_CC_TIM, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
	  LL_TIM_EnableCounter(SWIM_CC_TIM);



	  //LL_TIM_EnableDMAReq_UPDATE(SWIM_TIM);
	  LL_TIM_GenerateEvent_UPDATE(SWIM_TIM);   //To force DMA to update CCR and reset the counter
	  //LL_TIM_CC_EnableChannel(SWIM_TIM, SWIM_CH);
	  LL_TIM_SetCounter(SWIM_TIM, 3);			//Value close to zero, to send the update event as soon as possible
	  //LL_TIM_OC_SetCompareCH3(SWIM_TIM, 42591);  //First low pulse, 16us
	  LL_TIM_EnableCounter(SWIM_TIM);
	  while(SWIMStatus == SWIM_RST)
		  ;

}

