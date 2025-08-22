/*
 * Came.h
 *
 *  Created on: 30 Jun 2020
 *      Author: Marco
 */

#ifndef INC_CAME_H_
#define INC_CAME_H_

#include "main.h"
#include "string.h"
#include "Swim.h"

#define NCodesInMem 512

typedef enum {
	CameOpIdle = 0,
	CameOpRead,
	CameOpWrite
} CAMESTATUS;

struct CodeLog {
	uint32_t Timestamp;
	uint32_t Code;  //4 bit per key, 0 = no button, 1..9 => 1..9, 0 = A, first button is on the left, then sift to put other, as many as necessary
	uint32_t Status;  //low 4 bits = relay, high 4 bit = result (1 = open, 0 = wrong code) //Using 32 bits because it will align every 4 bytes
};

void CheckStoreCode(void);
void ReadFlashCode(void);
void WriteFlashCode(void);

//Variables used in main.c
extern CAMESTATUS CameStatus;
extern uint8_t  AddrCode;   //Address for code, 0..50
extern uint8_t  CodeData[16];   //[AA or 00] [1/2/3 or FF] [code1] .. [code8]

#endif /* INC_CAME_H_ */
