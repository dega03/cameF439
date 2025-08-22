/*
 * Swim.h
 *
 *  Created on: Jun 29, 2020
 *      Author: Marco
 */

#ifndef INC_SWIM_H_
#define INC_SWIM_H_

#include "main.h"
#include "string.h"

#define SWIM_Rst             0x00000000
#define SWIM_InitAck         0x00000001
#define SWIM_Idle            0x00000002
#define SWIM_Timeout         0x80000001
#define SWIM_InitErr         0x80000002
#define SWIM_RST             0x00001001

#define SWIM_ReadCmd         0x00001004
#define SWIM_ReadCmdAck      0x00001005
#define SWIM_WriteCmd        0x00001008
#define SWIM_WriteCmdAck     0x00001009

#define SWIM_ReadCmdData     0x00001806
#define SWIM_ReadCmdDataAck  0x00001807
#define SWIM_ReadCmdAnswr    0x00001808
#define SWIM_ReadCmdAnswrMsk 0xFF00FFFF
#define SWIM_ReadCmdAnswrShf 16



#define SWIM_WriteCmdData    0x0000180A
#define SWIM_WriteCmdDataAck 0x0000180B
#define SWIM_ErrMask         0xFF000000

#define SWIM_Err             0x81000000
#define SWIM_AckErr          0x82000000
#define SWIM_CmdMask         0x00001800
#define SWIM_Cmd5BitCmd      0x00001000

#define SWIM_CC_TIM          TIM3
#define SWIM_TIM             TIM4
#define SWIM_CH              LL_TIM_CHANNEL_CH3

#define DMA_PWM              LL_DMA_STREAM_6
#define DMA_InpCapt          LL_DMA_STREAM_5

#define K_TIMTIMEOUT         0xfff0  //about 750us = 0xfff0 / 88MHz


void SWIM_Read(uint32_t addr, uint8_t Ndata, uint16_t * DataOut);
void SWIM_Write(uint32_t addr, uint8_t Ndata, uint16_t * DataIn);
void SWIMRst (void);
uint32_t * PrepareTxData (uint32_t DataIn,  uint32_t * BufferPtr);
void SWIMInit (void);
void SWIMCommRst(void);

#endif /* INC_SWIM_H_ */
