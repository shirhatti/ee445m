// FIFO.c
// Runs on any LM3Sxxx
// Provide functions that initialize a FIFO, put data in, get data out,
// and return the current size.  The file includes a transmit FIFO
// using index implementation and a receive FIFO using pointer
// implementation.  Other index or pointer implementation FIFOs can be
// created using the macros supplied at the end of the file.
// Daniel Valvano
// June 16, 2011

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to the Arm Cortex M3",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2011
   Programs 3.7, 3.8., 3.9 and 3.10 in Section 3.7

 Copyright 2011 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include "FIFO.h"
#include "os.h"
#include "UART_FIFO.h"

// Two-index implementation of the transmit FIFO
// can hold 0 to TXFIFOSIZE elements
#define TXFIFOSIZE 16 // must be a power of 2
#define TXFIFOSUCCESS 1
#define TXFIFOFAIL    0


unsigned long volatile Tx_UARTPutI;// put next
unsigned long volatile Tx_UARTGetI;// get next
tx_UARTDataType static Tx_UARTFifo[TXFIFOSIZE];

Sema4Type Tx_UARTRoomType;
	
// initialize index FIFO
void Tx_UARTFifo_Init(void){ long sr;
  sr = StartCritical(); // make atomic
  OS_InitSemaphore(&Tx_UARTRoomType, TXFIFOSIZE);
  Tx_UARTPutI = Tx_UARTGetI = 0;  // Empty
  EndCritical(sr);
}
// add element to end of index FIFO
// return TXFIFOSUCCESS if successful
int Tx_UARTFifo_Put(tx_UARTDataType data){
  OS_Wait(&Tx_UARTRoomType);
	
	if((Tx_UARTPutI-Tx_UARTGetI) & ~(TXFIFOSIZE-1)){
    return(TXFIFOFAIL); // Failed, fifo full
  }
  Tx_UARTFifo[Tx_UARTPutI&(TXFIFOSIZE-1)] = data; // put
  Tx_UARTPutI++;  // Success, update
  return(TXFIFOSUCCESS);
}
// remove element from front of index FIFO
// return TXFIFOSUCCESS if successful
int Tx_UARTFifo_Get(tx_UARTDataType *datapt){
  if(Tx_UARTPutI == Tx_UARTGetI ){
    return(TXFIFOFAIL); // Empty if TxPutI=TxGetI
  }
  *datapt = Tx_UARTFifo[Tx_UARTGetI&(TXFIFOSIZE-1)];
  Tx_UARTGetI++;  // Success, update
  OS_Signal(&Tx_UARTRoomType);
	
  return(TXFIFOSUCCESS);
}
// number of elements in index FIFO
// 0 to TXFIFOSIZE-1
unsigned short Tx_UARTFifo_Size(void){
 return ((unsigned short)(Tx_UARTPutI-Tx_UARTGetI));
}

// Two-pointer implementation of the receive FIFO
// can hold 0 to RXFIFOSIZE-1 elements
#define RXFIFOSIZE 10 // can be any size
#define RXFIFOSUCCESS 1
#define RXFIFOFAIL    0

rx_UARTDataType volatile *Rx_UARTPutPt; // put next
rx_UARTDataType volatile *Rx_UARTGetPt; // get next
rx_UARTDataType static Rx_UARTFifo[RXFIFOSIZE];

Sema4Type Rx_UARTDataAvailable;

// initialize pointer FIFO
void Rx_UARTFifo_Init(void){ long sr;
  sr = StartCritical();      // make atomic
  OS_InitSemaphore(&Rx_UARTDataAvailable, 0);
  Rx_UARTPutPt = Rx_UARTGetPt = &Rx_UARTFifo[0]; // Empty
  EndCritical(sr);
}
// add element to end of pointer FIFO
// return RXFIFOSUCCESS if successful
int Rx_UARTFifo_Put(rx_UARTDataType data){
  rx_UARTDataType volatile *nextPutPt;
  nextPutPt = Rx_UARTPutPt+1;
  if(nextPutPt == &Rx_UARTFifo[RXFIFOSIZE]){
    nextPutPt = &Rx_UARTFifo[0];  // wrap
  }
  if(nextPutPt == Rx_UARTGetPt){
    return(RXFIFOFAIL);      // Failed, fifo full
  }
  else{
    *(Rx_UARTPutPt) = data;       // Put
    Rx_UARTPutPt = nextPutPt;     // Success, update
	OS_Signal(&Rx_UARTDataAvailable);
    return(RXFIFOSUCCESS);
  }
}
// remove element from front of pointer FIFO
// return RXFIFOSUCCESS if successful
int Rx_UARTFifo_Get(rx_UARTDataType *datapt){
	OS_Wait(&Rx_UARTDataAvailable);
	
	if(Rx_UARTPutPt == Rx_UARTGetPt ){
    return(RXFIFOFAIL);      // Empty if PutPt=GetPt
  }
  *datapt = *(Rx_UARTGetPt++);
  if(Rx_UARTGetPt == &Rx_UARTFifo[RXFIFOSIZE]){
     Rx_UARTGetPt = &Rx_UARTFifo[0];   // wrap
  }
  return(RXFIFOSUCCESS);
}
// number of elements in pointer FIFO
// 0 to RXFIFOSIZE-1
unsigned short Rx_UARTFifo_Size(void){
  if(Rx_UARTPutPt < Rx_UARTGetPt){
    return ((unsigned short)(Rx_UARTPutPt-Rx_UARTGetPt+(RXFIFOSIZE*sizeof(rx_UARTDataType)))/sizeof(rx_UARTDataType));
  }
  return ((unsigned short)(Rx_UARTPutPt-Rx_UARTGetPt)/sizeof(rx_UARTDataType));
}
