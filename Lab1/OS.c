//******Filename: OS.h**************//
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

#define NVIC_EN0_INT19          0x00080000  // Interrupt 19 enable

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

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PeriodicTask)(void);   // user function

static unsigned long Counter = 0;

/**************OS_AddPeriodicThread***************
 Activate Timer0 interrupts to run user task periodically
   Input: task is a pointer to a user function
				  period in units (1/clockfreq)
				  priority (0-7)
 Outputs: error (1); success (0)
*************************************************/ 
int OS_AddPeriodicThread(void (*task)(void), unsigned long period, unsigned long priority) {
	long sr;
	
	if(priority > 7) { return 1; }
	Counter = 0;
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x01;
  
  PeriodicTask = task;             // user function
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer0A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER0_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer0A timeout flag
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
																	 // 7) priority shifted to bits 31-29 for timer0A
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|(priority << 29);	
  NVIC_EN0_R = NVIC_EN0_INT19;     // 8) enable interrupt 19 in NVIC
  TIMER0_TAPR_R = 0;
  TIMER0_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer0A
	
  EndCritical(sr);
	return 0;
}

void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer0A timeout
	Counter++;
  
	/*if(Counter == 0xFFFFFFFF) {
			OverflowCount++;
			Counter = 0;
	} */
	
	(*PeriodicTask)();                // execute user task
}

/**************OS_ClearPeriodicTime***************
 converts fixed point number to ASCII string
 format: signed 32-bit with resolution 0.001
  range: -9.999 to +9.999
  Input: signed 32-bit integer part of fixed point number
				 greater or less than range is invalid number
  Output: null-terminated string exactly 6 characters plus null 
*************************************************/
void OS_ClearPeriodicTime(void) {
	Counter = 0;
}

/**************OS_ReadPeriodicTime***************
 converts fixed point number to ASCII string
 format: signed 32-bit with resolution 0.001
  range: -9.999 to +9.999
  Input: signed 32-bit integer part of fixed point number
				 greater or less than range is invalid number
  Output: null-terminated string exactly 6 characters plus null 
*************************************************/
unsigned long OS_ReadPeriodicTime(void) {
	return Counter;
}
