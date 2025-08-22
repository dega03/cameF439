/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 *
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 *
 * Christiaan Simons rewrote this file to get a more stable echo application.
 *
 **/

 /* This file was modified by ST */


#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "string.h"
#include "Came.h"

#if LWIP_TCP

static struct tcp_pcb *eth_interface_pcb;  //tcp_echoserver

/* ECHO protocol states */
enum eth_interface_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

/* structure for maintaing connection infos to be passed as argument 
   to LwIP callbacks*/
struct eth_interface_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};


static err_t eth_interface_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t eth_interface_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void eth_interface_error(void *arg, err_t err);
static err_t eth_interface_poll(void *arg, struct tcp_pcb *tpcb);
static err_t eth_interface_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void eth_interface_send(struct tcp_pcb *tpcb, struct eth_interface_struct *es);
static void eth_interface_connection_close(struct tcp_pcb *tpcb, struct eth_interface_struct *es);

char DataTcpTx[256];
char CurrCmd[32];
uint32_t RxCmdPtr;
uint32_t LastRxTimestamp;

extern struct CodeLog Codes[NCodesInMem];
extern uint32_t NewCodePtr;
extern uint32_t NewCode2ReadPtr;


/**
  * @brief  Initializes the tcp echo server
  * @param  None
  * @retval None
  */
void eth_interface_init(void)
{
  /* create new tcp pcb */
  eth_interface_pcb = tcp_new();

  if (eth_interface_pcb != NULL)
  {
    err_t err;
    
    /* bind echo_pcb to port 7 (ECHO protocol) */
    err = tcp_bind(eth_interface_pcb, IP_ADDR_ANY, 1978);
    
    if (err == ERR_OK)
    {
      /* start tcp listening for echo_pcb */
      eth_interface_pcb = tcp_listen(eth_interface_pcb);
      
      /* initialize LwIP tcp_accept callback function */
      tcp_accept(eth_interface_pcb, eth_interface_accept);
    }
    else 
    {
      /* deallocate the pcb */
      memp_free(MEMP_TCP_PCB, eth_interface_pcb);
    }
  }
}

/**
  * @brief  This function is the implementation of tcp_accept LwIP callback
  * @param  arg: not used
  * @param  newpcb: pointer on tcp_pcb struct for the newly created tcp connection
  * @param  err: not used 
  * @retval err_t: error status
  */
static err_t eth_interface_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct eth_interface_struct *es;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  /* set priority for the newly accepted tcp connection newpcb */
  tcp_setprio(newpcb, TCP_PRIO_MIN);

  /* allocate structure es to maintain tcp connection informations */
  es = (struct eth_interface_struct *)mem_malloc(sizeof(struct eth_interface_struct));
  if (es != NULL)
  {
    es->state = ES_ACCEPTED;
    es->pcb = newpcb;
    es->retries = 0;
    es->p = NULL;
    
    /* pass newly allocated es structure as argument to newpcb */
    tcp_arg(newpcb, es);
    
    /* initialize lwip tcp_recv callback function for newpcb  */ 
    tcp_recv(newpcb, eth_interface_recv);
    
    /* initialize lwip tcp_err callback function for newpcb  */
    tcp_err(newpcb, eth_interface_error);
    
    /* initialize lwip tcp_poll callback function for newpcb */
    tcp_poll(newpcb, eth_interface_poll, 0);
    
    ret_err = ERR_OK;
  }
  else
  {
    /*  close tcp connection */
    eth_interface_connection_close(newpcb, es);
    /* return memory error */
    ret_err = ERR_MEM;
  }
  return ret_err;  
}


/**
  * @brief  This function is the implementation for tcp_recv LwIP callback
  * @param  arg: pointer on a argument for the tcp_pcb connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  pbuf: pointer on the received pbuf
  * @param  err: error information regarding the reveived pbuf
  * @retval err_t: error code
  */
static err_t eth_interface_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct eth_interface_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct eth_interface_struct *)arg;
  
  /* if we receive an empty tcp frame from client => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    if(es->p == NULL)
    {
       /* we're done sending, close connection */
       eth_interface_connection_close(tpcb, es);
    }
    else
    {
      /* we're not done yet */
      /* acknowledge received packet */
      tcp_sent(tpcb, eth_interface_sent);
      
      /* send remaining data*/
      eth_interface_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  /* else : a non empty frame was received from client but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_ACCEPTED)
  {
    /* first data chunk in p->payload */
    es->state = ES_RECEIVED;
    
    /* store reference to incoming pbuf (chain) */
    es->p = p;
    
    /* initialize LwIP tcp_sent callback function */
    tcp_sent(tpcb, eth_interface_sent);
    
    /* send back the received data (echo) */
    eth_interface_send(tpcb, es);
    
    ret_err = ERR_OK;
  }
  else if (es->state == ES_RECEIVED)
  {
    /* more data received from client and previous data has been already sent*/
    if(es->p == NULL)
    {
      es->p = p;
  
      /* send back received data */
      eth_interface_send(tpcb, es);
    }
    else
    {
      struct pbuf *ptr;

      /* chain pbufs to the end of what we recv'ed previously  */
      ptr = es->p;
      pbuf_chain(ptr,p);
    }
    ret_err = ERR_OK;
  }
  else if(es->state == ES_CLOSING)
  {
    /* odd case, remote side closing twice, trash data */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    /* unkown es->state, trash data  */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_err callback function (called
  *         when a fatal tcp_connection error occurs. 
  * @param  arg: pointer on argument parameter 
  * @param  err: not used
  * @retval None
  */
static void eth_interface_error(void *arg, err_t err)
{
  struct eth_interface_struct *es;

  LWIP_UNUSED_ARG(err);

  es = (struct eth_interface_struct *)arg;
  if (es != NULL)
  {
    /*  free es structure */
    mem_free(es);
  }
}

/**
  * @brief  This function implements the tcp_poll LwIP callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: pointer on the tcp_pcb for the current tcp connection
  * @retval err_t: error code
  */
static err_t eth_interface_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct eth_interface_struct *es;

  es = (struct eth_interface_struct *)arg;
  if (es != NULL)
  {
    if (es->p != NULL)
    {
      tcp_sent(tpcb, eth_interface_sent);
      /* there is a remaining pbuf (chain) , try to send data */
      eth_interface_send(tpcb, es);
    }
    else
    {
      /* no remaining pbuf (chain)  */
      if(es->state == ES_CLOSING)
      {
        /*  close tcp connection */
        eth_interface_connection_close(tpcb, es);
      }
    }
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
    tcp_abort(tpcb);
    ret_err = ERR_ABRT;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data) 
  * @param  None
  * @retval None
  */
static err_t eth_interface_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct eth_interface_struct *es;

  LWIP_UNUSED_ARG(len);

  es = (struct eth_interface_struct *)arg;
  es->retries = 0;
  
  if(es->p != NULL)
  {
    /* still got pbufs to send */
    tcp_sent(tpcb, eth_interface_sent);
    eth_interface_send(tpcb, es);
  }
  else
  {
    /* if no more data to send and client closed connection*/
    if(es->state == ES_CLOSING)
      eth_interface_connection_close(tpcb, es);
  }
  return ERR_OK;
}


/**
  * @brief  This function is used to send data for tcp connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void eth_interface_send(struct tcp_pcb *tpcb, struct eth_interface_struct *es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
  uint32_t CurrTimestamp;
  uint32_t CodeNr,i;
  uint16_t Err;
 
  while ((wr_err == ERR_OK) &&
          (es->p != NULL) &&
         (es->p->len <= tcp_sndbuf(tpcb)))
  {
    
    /* get pointer on pbuf from es structure */
    ptr = es->p;

    /* enqueue data for transmission */
    //wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
    //uint32_t RxCmdPtr;
    //uint32_t LastTxTimestamp;
    CurrTimestamp = LL_TIM_GetCounter(TIM2);
    if ((CurrTimestamp - LastRxTimestamp) > 3){  // 3 sec timeout
    	RxCmdPtr = 0;
    }
    LastRxTimestamp = CurrTimestamp;

    memcpy(CurrCmd + RxCmdPtr, ptr->payload,ptr->len);

    i = 0;
    while ((i < (ptr->len)) && ((CurrCmd[RxCmdPtr + i] < 32) || (CurrCmd[RxCmdPtr + i] > 127)))
    	i++;
    if ((i > 0) && (i < ptr->len)) {
    	memmove(CurrCmd + RxCmdPtr,CurrCmd + RxCmdPtr + i, ptr->len -i);
    }
    RxCmdPtr += ptr->len - i;

    if (CurrCmd[0] == 'W' && CurrCmd[1] == 'C' && RxCmdPtr >12){
    	Err = 0;
    	if (CameStatus != CameOpIdle) {
    		Err |= 1;
    	}
    	AddrCode = ((CurrCmd[2] - '0') * 10) + CurrCmd[3] - '0';
    	if (AddrCode>50)
    		Err |= 2;
    	for (i=0;i<9;i++) {
			CurrCmd[4 + i] |= 0x20; //Converted to lowercase   Address 0 can write any code up to f, to disable master code
			if ( ((AddrCode > 0) && (CurrCmd[4 + i] > 'a')) || (CurrCmd[4 + i] < '0') || ((CurrCmd[4 + i] > '9') && (CurrCmd[4 + i] < 'a')) || ((AddrCode == 0) && (CurrCmd[4 + i] > 'f')) ) {
				Err |= 4;
			} else {
				if (CurrCmd[4 + i] >= 'a')
					CodeData[i+1] = (CurrCmd[4 + i] - 'a' + 10);
				else
					CodeData[i+1] = (CurrCmd[4 + i] - '0');
			}
    	}
    	if ((CodeData[1]>3) || ((CodeData[1]==3) && (AddrCode!= 0)))  //Output greater than 3, not possible, also Address for output 3 must be zero
    		Err |= 8;
    	if (Err == 0) {
    		CodeData[0] = 0xAA;
			sprintf(DataTcpTx,"WC%02d|%02X>%01X.%01X%01X%01X%01X%01X%01X%01X%01X\r\n",AddrCode,CodeData[0],CodeData[1] & 0xf,CodeData[2] & 0xf,CodeData[3] & 0xf,CodeData[4] & 0xf,CodeData[5] & 0xf,CodeData[6] & 0xf,CodeData[7] & 0xf,CodeData[8] & 0xf,CodeData[9] & 0xf);
			wr_err = tcp_write(tpcb, DataTcpTx, 20, 1);
            CameStatus = CameOpWrite;
    	} else {
			sprintf(DataTcpTx,"WC Err = %04X\r\n",Err);
			wr_err = tcp_write(tpcb, DataTcpTx, 15, 1);
    	}
        memmove(CurrCmd, CurrCmd+13,RxCmdPtr-13);
        RxCmdPtr -= 13;
    }
    if (CurrCmd[0] == 'R' && CurrCmd[1] == 'C' && RxCmdPtr >3){  //Read code
    	Err = 0;
    	memset(&CodeData[0],0xff,10);  //Clear current buffer
    	if (CameStatus != CameOpIdle) {
    		Err |= 1;
    	}
    	AddrCode = ((CurrCmd[2] - '0') * 10) + CurrCmd[3] - '0';
    	if (AddrCode>50)
    		Err |= 2;
    	if (Err == 0) {
    		sprintf(DataTcpTx,"RC%02d\r\n",AddrCode);
            wr_err = tcp_write(tpcb, DataTcpTx, 6, 1);
            CameStatus = CameOpRead;
    	} else {
			sprintf(DataTcpTx,"RC Err = %04X\r\n",Err);
			wr_err = tcp_write(tpcb, DataTcpTx, 15, 1);
    	}
        memmove(CurrCmd, CurrCmd+4,RxCmdPtr-4);
        RxCmdPtr -= 4;
    }
    if (CurrCmd[0] == 'D' && CurrCmd[1] == 'C' && RxCmdPtr >5){  //Delete code
    	Err = 0;
    	if (CameStatus != CameOpIdle) {
    		Err |= 1;
    	}
    	AddrCode = ((CurrCmd[2] - '0') * 10) + CurrCmd[3] - '0';
    	CurrCmd[6] = ((CurrCmd[4] - '0') * 10) + CurrCmd[5] - '0';
    	if ((AddrCode>50) || (AddrCode == 0))
    		Err |= 2;
    	if ((AddrCode + CurrCmd[6]) != 55) //Chekum, address must be repeated and second time must be be 55-x
    		Err |= 0x10;
    	if (Err == 0) {
    		CodeData[0] = 0x00;
    		memset(&CodeData[1],0xff,9);
    		sprintf(DataTcpTx,"DC%02d\r\n",AddrCode);
            wr_err = tcp_write(tpcb, DataTcpTx, 6, 1);
            CameStatus = CameOpWrite;
    	} else {
			sprintf(DataTcpTx,"DC Err = %04X\r\n",Err);
			wr_err = tcp_write(tpcb, DataTcpTx, 15, 1);
    	}
        memmove(CurrCmd, CurrCmd+6,RxCmdPtr-6);
        RxCmdPtr -= 6;
    }
    if (CurrCmd[0] == 'R' && CurrCmd[1] == 'B' && RxCmdPtr >1){  //Read code buffer
    	 // sprintf takes about about 58us
        sprintf(DataTcpTx,"RB%02d|%02X>%01X.%01X%01X%01X%01X%01X%01X%01X%01X\r\n",AddrCode,CodeData[0],CodeData[1] & 0xf,CodeData[2] & 0xf,CodeData[3] & 0xf,CodeData[4] & 0xf,CodeData[5] & 0xf,CodeData[6] & 0xf,CodeData[7] & 0xf,CodeData[8] & 0xf,CodeData[9] & 0xf);
        wr_err = tcp_write(tpcb, DataTcpTx, 20, 1);
        memmove(CurrCmd, CurrCmd+2,RxCmdPtr-2);
        RxCmdPtr -= 2;
    }
    if (CurrCmd[0] == 'T' && RxCmdPtr >0){
        sprintf(DataTcpTx,"%08lX\r\n",CurrTimestamp);
        wr_err = tcp_write(tpcb, DataTcpTx, 10, 1);
        memmove(CurrCmd, CurrCmd+1,RxCmdPtr-1);
        RxCmdPtr -= 1;
    }
    if (CurrCmd[0] == 'S' && RxCmdPtr >0){
        sprintf(DataTcpTx,"%04X-%04X|%02X\r\n",(unsigned int)NewCodePtr,(unsigned int)NewCode2ReadPtr,CameStatus);
        wr_err = tcp_write(tpcb, DataTcpTx, 14, 1);
        memmove(CurrCmd, CurrCmd+1,RxCmdPtr-1);
        RxCmdPtr -= 1;
    }
    if (CurrCmd[0] == 'P' && RxCmdPtr>= 5){
    	CurrCmd[1] |= 0x20; //Converted to lowercase
    	CurrCmd[2] |= 0x20; //Converted to lowercase
    	CurrCmd[3] |= 0x20; //Converted to lowercase
    	CurrCmd[4] |= 0x20; //Converted to lowercase
		if (CurrCmd[1] >= 'a')
			CodeNr = CurrCmd[1] - 'a' + 10;
		else
			CodeNr = CurrCmd[1] - '0';
		CodeNr = CodeNr <<4;
		if (CurrCmd[2] >= 'a')
			CodeNr |= CurrCmd[2] - 'a' + 10;
		else
			CodeNr |= CurrCmd[2] - '0';
		CodeNr = CodeNr <<4;
		if (CurrCmd[3] >= 'a')
			CodeNr |= CurrCmd[3] - 'a' + 10;
		else
			CodeNr |= CurrCmd[3] - '0';
		CodeNr = CodeNr <<4;
		if (CurrCmd[4] >= 'a')
			CodeNr |= CurrCmd[4] - 'a' + 10;
		else
			CodeNr |= CurrCmd[4] - '0';

		//Codes[NCodesInMem];
		if (CodeNr>= NCodesInMem ) {
            sprintf(DataTcpTx,"P%04X-XXXXXXXX:XXXXXXXX>XX|XX\r\n",(unsigned int)CodeNr);
            wr_err = tcp_write(tpcb, DataTcpTx, 31, 1);
		} else {
    	//Temp = strtol(CurrCmd +1, NULL, 16);
            sprintf(DataTcpTx,"P%04X-%08lX:%08lX>%02X|%02X\r\n",(unsigned int)CodeNr,Codes[CodeNr].Timestamp,Codes[CodeNr].Code,(unsigned int)(Codes[CodeNr].Status >> 16) & 0xff, (unsigned int)Codes[CodeNr].Status & 0xff);
            wr_err = tcp_write(tpcb, DataTcpTx, 31, 1);
		}
        memmove(CurrCmd, CurrCmd + 5,RxCmdPtr -5);
        RxCmdPtr -= 5;

        NewCode2ReadPtr = (CodeNr + 1 );
		if (NewCode2ReadPtr >= NCodesInMem)
			NewCode2ReadPtr = 0;

    }
    if (CurrCmd[0] == '@' && RxCmdPtr >0){
        NVIC_SystemReset();
        memmove(CurrCmd, CurrCmd+1,RxCmdPtr-1);
        RxCmdPtr -= 1;
    }

    
    if (wr_err == ERR_OK)
    {
      u16_t plen;
      u8_t freed;

      plen = ptr->len;
     
      /* continue with next pbuf in chain (if any) */
      es->p = ptr->next;
      
      if(es->p != NULL)
      {
        /* increment reference count for es->p */
        pbuf_ref(es->p);
      }
      
     /* chop first pbuf from chain */
      do
      {
        /* try hard to free pbuf */
        freed = pbuf_free(ptr);
      }
      while(freed == 0);
     /* we can read more data now */
     tcp_recved(tpcb, plen);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later / harder, defer to poll */
     es->p = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
}

/**
  * @brief  This functions closes the tcp connection
  * @param  tcp_pcb: pointer on the tcp connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void eth_interface_connection_close(struct tcp_pcb *tpcb, struct eth_interface_struct *es)
{
  
  /* remove all callbacks */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  
  /* delete es structure */
  if (es != NULL)
  {
    mem_free(es);
  }  
  
  /* close tcp connection */
  tcp_close(tpcb);
}

#endif /* LWIP_TCP */
