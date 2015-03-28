#ifndef UART_FIFO_H
#define UART_FIFO_H

typedef char tx_UARTDataType;
typedef char rx_UARTDataType;

// initialize index FIFO
void Tx_UARTFifo_Init(void);
// add element to end of index FIFO
// return TXFIFOSUCCESS if successful
int Tx_UARTFifo_Put(tx_UARTDataType data);
// remove element from front of index FIFO
// return TXFIFOSUCCESS if successful
int Tx_UARTFifo_Get(tx_UARTDataType *datapt);
// number of elements in index FIFO
// 0 to TXFIFOSIZE-1
unsigned short Tx_UARTFifo_Size(void);

// initialize pointer FIFO
void Rx_UARTFifo_Init(void);
// add element to end of pointer FIFO
// return RXFIFOSUCCESS if successful
int Rx_UARTFifo_Put(rx_UARTDataType data);
// remove element from front of pointer FIFO
// return RXFIFOSUCCESS if successful
int Rx_UARTFifo_Get(rx_UARTDataType *datapt);
// number of elements in pointer FIFO
// 0 to RXFIFOSIZE-1
unsigned short Rx_UARTFifo_Size(void);

#endif
