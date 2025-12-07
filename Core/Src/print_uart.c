/*
 * print_uart.c
 *
 *  Created on: Apr 19, 2023
 *      Author: mdegano
 *
 *      In main.c add a pfp:
 *      void printf_init();
 *      ...
 *      and after uart init:
 *      printf_init();
 *
 *
 *      in stm32h7xx_it.c add a pfp:
 *      void printf_irq();
 *      ...
 *      then in USART3_IRQHandler:
 *      printf_irq();
 *
 *      and in flash loader:
 *       . = ALIGN(4);
 *      .RamD3_sec (NOLOAD) :
 *      {
 *          . = ALIGN(4);
 *      } > RAM_D3
 *
 *      Remember to configure MPU for 0x38000000
 *      Size: 16k
 *      Access permission: all access enable
 *      Instruction access: enable
 *      Shareability: disable
 *      Cacheable:    disable
 *      Bufferable:   disable
 *
 */
#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2


char BufferTx1[2048*4];
char BufferTx2[2048*4];
uint16_t Buff1Len,Buff2Len;
char Buff1Lock = 0;
char Buff2Lock = 0;
char BuffSending = 0;
char Buffer2Send  = 0;

void printf_init() {
	  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_3, LL_USART_DMA_GetRegAddr(USART3));
	  LL_USART_EnableDMAReq_TX(USART3);
	  Buff1Len = 0;
	  Buff2Len = 0;
}

void Check2SendNext() {
	if (Buffer2Send > 0) {
		LL_DMA_ClearFlag_TC3(DMA1);
		LL_DMA_ClearFlag_TE0(DMA1);
		if (Buffer2Send == 1 && Buff1Lock == 0) {
			BuffSending = 1;
			LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t) BufferTx1);
			LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3,Buff1Len);
		} else if (Buffer2Send == 2 && Buff2Lock == 0) {
			BuffSending = 2;
			LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t) BufferTx2);
			LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3,Buff2Len);
		}
		LL_USART_ClearFlag_TC(USART3);
		LL_USART_EnableIT_TC(USART3);
		LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
	}
}


void printf_irq() {
	LL_USART_DisableIT_TC(USART3);
	if (BuffSending == 1) {
		Buff1Len = 0;
	} else {
		Buff2Len = 0;
	}
	BuffSending = 0;
	Check2SendNext();
}


int _write(int fd, char *ptr, int len)
{
	if (BuffSending > 0 && LL_USART_IsActiveFlag_TXE(USART3)) {
		printf_irq();
	}
	  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
		if (BuffSending == 0) {
			LL_DMA_ClearFlag_TC3(DMA1);
			BuffSending = 1;
			memcpy((void *) BufferTx1,(void *) ptr,len);
			LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_3, (uint32_t) BufferTx1);
			LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3,len);
			LL_USART_ClearFlag_TC(USART3);
			LL_USART_EnableIT_TC(USART3);
			LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);
		} else {
			if (BuffSending == 1) {
				Buff2Lock = 1;
				memcpy((void *) BufferTx2 + Buff2Len,(void *) ptr,len);
				Buff2Len += len;
				Buff2Lock = 0;
				Buffer2Send = 2;
			} else if (BuffSending == 2) {
				Buff1Lock = 1;
				memcpy((void *) BufferTx1 + Buff1Len,(void *) ptr,len);
				Buff1Len += len;
				Buff1Lock = 0;
				Buffer2Send = 1;
			}
			if (BuffSending == 0) {
				Check2SendNext();
			}
		}
        return len;
	  }
	  errno = EBADF;
	  return -1;
}
