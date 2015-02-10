/***************************************************
Created by Sourabh Shirhatti and Nelson Wu for EE 445M, Spring 2015
****************************************************/

#include <stdint.h>
#include "cmdline.h"
#include "PLL.h"
#include "UART.h"
#include "ST7735.h"
#include "OS.h"

void dummy(void) {
	UART_OutUDec(OS_ReadPeriodicTime());
}

int main(void){
  char string[80];  // global to assist in debugging
  PLL_Init();               // set system clock to 50 MHz
	Output_Init();
  UART_Init();              // initialize UART
	OS_AddPeriodicThread(dummy, 1000, 3);
		
  while(1){
    OutCRLF(); UART_OutString(">");
    UART_InString(string,79);
		CmdLineProcess(string);
  }
}
