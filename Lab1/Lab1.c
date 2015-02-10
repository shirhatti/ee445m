/***************************************************
Created by Sourabh Shirhatti and Nelson Wu for EE 445M, Spring 2015
****************************************************/

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "cmdline.h"
#include "PLL.h"
#include "UART.h"
#include "ST7735.h"
#include "OS.h"

#define GPIO_PORTF2             (*((volatile uint32_t *)0x40025010))
	
void dummy(void) {
	// UART_OutUDec(OS_ReadPeriodicTime());
}

int main(void){
  char string[80];  // global to assist in debugging
  PLL_Init();               // set system clock to 50 MHz
	Output_Init();
  UART_Init();              // initialize UART
	OS_AddPeriodicThread(dummy, 1000, 3);
	
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F
	GPIO_PORTF_DIR_R |= 0x04;             // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;          // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;             // enable digital I/O on PF2
                                        // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
  GPIO_PORTF2 = 0;                      // turn off LED	
	
  while(1){
    OutCRLF(); UART_OutString(">");
    UART_InString(string,79);
		if(CmdLineProcess(string) == -1) {
			UART_OutString("command not recognized");
		}
  }
}
