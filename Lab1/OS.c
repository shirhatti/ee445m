//******Filename: OS.c**************//
//*******Authors: Sourabh Shirhatti*//
//*************** Nelson Wu*********//
//*******Created: Jan 24, 2015******//
//***Description: ***//
//*************** **// 
//*************** ******//
//*********Lab #: 1***********************************************//
//***********TAs: ,****************//
//*******Revised: Feb 8, 2015************************************//
//*****HW Config: None********************************************//

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "OS.h"

#define NVIC_EN0_INT21          0x00200000  // Interrupt 21 enable

#define TIMER_CFG_32_BIT_TIMER  0x00000000  // 32-bit timer configuration
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                            // Mask
#define TIMER_ICR_TATOCINT      0x00000001  // GPTM TimerA Time-Out Raw
                                            // Interrupt
#define TIMER_TAILR_M           0xFFFFFFFF  // GPTM Timer A Interval Load
                                            // Register

#define GPIO_PORTF2             (*((volatile uint32_t *)0x40025010))
	
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PeriodicTask)(void);   // user function

static unsigned long Counter = 0;

/**************OS_AddPeriodicThread***************
 Activate Timer1 interrupts to run user task periodically
   Input: task is a pointer to a user function
				  period in ms
				  priority (0-7)
 Outputs: error (1); success (0)
*************************************************/ 
int OS_AddPeriodicThread(void (*task)(void), unsigned long period, unsigned long priority) {
  long sr;
	
  if(priority > 7) { return 1; }
  Counter = 0;
  
  sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x02;
  
  PeriodicTask = task;             // user function
  TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER1_TAILR_R = (period*50000) - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;
  TIMER1_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 15-13 for timer1A
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|(priority << 13);	
  NVIC_EN0_R = NVIC_EN0_INT21;     // 8) enable interrupt 21 in NVIC
  TIMER1_TAPR_R = 0;
  TIMER1_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer1A
	
  EndCritical(sr);
	return 0;
}

void Timer1A_Handler(void){ 
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
	Counter++;
	
	(*PeriodicTask)();                // execute user task
}

//	GPIO_PORTF2 = 0x04;  
//  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
//	Counter++;
//  
//	/*if(Counter == 0xFFFFFFFF) {
//			OverflowCount++;
//			Counter = 0;
//	} */
//	
//	(*PeriodicTask)();                // execute user task
//	GPIO_PORTF2 = 0x00;

/**************OS_ClearPeriodicTime***************
 clear the 32-bit global counter
  Input: none
  Output: none
*************************************************/
void OS_ClearPeriodicTime(void) {
	Counter = 0;
}

/**************OS_ReadPeriodicTime***************
 reads the current value of the 32-bit global counter
  Input: none
  Output: current counter value
*************************************************/
unsigned long OS_ReadPeriodicTime(void) {
	return Counter;
}

void OS_StartPeriodicThread(void) {
	TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void OS_StopPeriodicThread(void) {
	TIMER1_CTL_R |= ~TIMER_CTL_TAEN;
}
