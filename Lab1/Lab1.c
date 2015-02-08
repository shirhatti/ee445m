/***************************************************
Created by Sourabh Shirhatti and Nelson Wu for EE 445M, Spring 2015
****************************************************/

#include <stdint.h>
#include "cmdline.h"
#include "PLL.h"
#include "UART.h"


int main(void){
  char string[80];  // global to assist in debugging
  PLL_Init();               // set system clock to 50 MHz
  UART_Init();              // initialize UART
	
  while(1){
    OutCRLF(); UART_OutString(">");
    UART_InString(string,79);
		CmdLineProcess(string);
  }
}
