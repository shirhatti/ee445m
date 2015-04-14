// InputCaptureTestMain.c
// Runs on LM4F120/TM4C123
// Use Timer0A in edge time mode to request interrupts on the rising
// edge of PB0 (CCP0), and count the pulses.
// Daniel Valvano
// September 11, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Example 6.1, Program 6.1

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// external signal connected to PB6 (T0CCP0) (trigger on rising edge)
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "InputCapture.h"
#include "PLL.h"
#include "can0.h"

#define PB6  (*((volatile unsigned long *)0x40005100))
	
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

volatile uint32_t Count;      // incremented on interrupt
uint32_t pingTime;
uint32_t TimeDiff, FirstTime;
uint32_t jitter;

void UserTask(void){
  GPIO_PORTF_DATA_R = GPIO_PORTF_DATA_R^0x04; // toggle PF2
  Count = Count + 1;
}

static int counter;
void Init_Timer2A(uint32_t period) {
	long sr;
	volatile unsigned long delay;
	
	sr = StartCritical();
	counter = 0;
  SYSCTL_RCGCTIMER_R |= 0x04;
	
  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;
	
  TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER2_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;
  TIMER2_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 31-29 for timer2A
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|(1 << 29);	
  NVIC_EN0_R = NVIC_EN0_INT23;     // 8) enable interrupt 23 in NVIC
  TIMER2_TAPR_R = 0;
  TIMER2_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer2A
	
  EndCritical(sr);
}

void Timer2A_Handler(void){ 
	unsigned long sr;
	
	TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer2A timeout
	
	if(counter == 100)
	{
		counter = 1;
		
		sr = StartCritical();
		GPIO_PORTB_AFSEL_R &= ~0x40; // regular port function
		GPIO_PORTB_DIR_R |= 0x40;    // make PD3-0 out
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x00000000;
		PB6 = 0x00;
		PB6 = 0x40;
		Timer4A_Wait(800);	// 10 us
		PB6 = 0x00;
		
		GPIO_PORTB_DIR_R &= ~0x40;       // make PB6 in
		GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x07000000;
		
		EndCritical(sr);
	}
	else {
		counter++;
	}
}

int main(void){
	PLL_Init();
	CAN0_Open();
                                   // activate port F
  SYSCTL_RCGCGPIO_R |= 0x20;
  while((SYSCTL_PRGPIO_R&0x0020) == 0){};// ready?
  Count = 0;                       // allow time to finish activating
  GPIO_PORTF_DIR_R |= 0x04;        // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;     // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;        // enable digital I/O on PF2
                                   // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;          // disable analog functionality on PF
		
	DisableInterrupts();
	Init_Timer4A();
	Init_Timer2A(80000);

#ifdef DEBUG		
	SYSCTL_RCGCGPIO_R |= 0x02; // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
		
	GPIO_PORTB_DIR_R |= 0x40;    // make PE3-0 output heartbeats
  GPIO_PORTB_AFSEL_R &= ~0x40;   // disable alt funct on PE3-0
  GPIO_PORTB_DEN_R |= 0x40;     // enable digital I/O on PE3-0
  GPIO_PORTB_PCTL_R = ~0x0000FFFF;
  GPIO_PORTB_AMSEL_R &= ~0x40;;      // disable analog functionality on PF	
		
	PB6 = 0x00;
	PB6 = 0x40;
	Timer4A_Wait(80000);
	PB6 = 0x00;
	
	GPIO_PORTB_DIR_R &= ~0x40;       // make PB6 in
  GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
  GPIO_PORTB_DEN_R |= 0x40;        // enable digital I/O on PB6
                                   // configure PB6 as T0CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x07000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;     // disable analog functionality on PB6
#endif
	
	TimerCapture_Init(UserTask);
	EnableInterrupts();
	
  while(1){
    WaitForInterrupt();
  }
}
