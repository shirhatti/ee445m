/***************************************************
Modified by Sourabh Shirhatti and Nelson Wu for EE 445M, Spring 2015
****************************************************/

// Used with Testmain2; comment out when running Testmain1
#define WITH_SYSTICK

// os.c
// Runs on LM4F120/TM4C123
// A very simple real time operating system with minimal features.
// Daniel Valvano
// January 29, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

   Programs 4.4 through 4.12, section 4.2

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

#include <stdint.h>
#include "os.h"
#include "PLL.h"
#include "ST7735.h"
// Additional includes for Lab 2
#include "inc/tm4c123gh6pm.h"
#include "UART.h"

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority

// Additional defines for Lab 2
#define NVIC_EN0_INT21          0x00200000  // Interrupt 21 enable
#define NVIC_EN1_INT35					0x00000008
#define NVIC_EN2_INT70					0x00000040	
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
	
// function definitions in osasm.s
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
int32_t StartCritical(void);
void EndCritical(int32_t primask);
void StartOS(void);

#define NUMTHREADS  10       // maximum number of threads
#define STACKSIZE   100      // number of 32-bit words in stack

#define OSFIFOSIZE  16
#define NUMPRI			6				 // maximum number of priorities

struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
  Sema4Type *status; // pointer to resource thread is blocked on (0 if not)
  uint32_t sleepCt;	 // sleep counter in MS
  uint32_t age;      // how long the thread has been active
  uint32_t id;       // thread #
	struct tcb *prev;	 // previous thread
  uint32_t priority; // used in priority scheduling
};

tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
tcbType *NextRunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];

static int i;

tcbType* RunPtArray[NUMPRI];
uint32_t PriCount[NUMPRI];
int32_t  available[NUMTHREADS];
uint32_t PriLevel;
uint32_t Launched;

void InitAvailable(void) {
	for (i = 0; i < NUMTHREADS; i++) {
		available[i] = 1;
	}
}

void InitPriorities(void) {
	int j;
	PriLevel = 0;
	InitAvailable();
	for (j = 0; j < NUMPRI; j++) {
		RunPtArray[j] = 0;
		PriCount[j] = 0;
	}
}

int add_thread(void) {
  int ret;
  for (i=0; i < NUMTHREADS; i++) {
    if (available[i]) {
      ret = i;
      available[i] = 0;
      return ret;
    }
  }
  return -1;
}

int add_thread_pri(uint32_t thread, uint32_t priority) {
	tcbType *next, *prev;
		
	if(PriCount[priority] == 0) {
		tcbs[thread].prev = &tcbs[thread];
		tcbs[thread].next = &tcbs[thread];
		RunPtArray[priority] = &tcbs[thread];
	}		
	else {
		prev = RunPtArray[priority];
		next = prev->next;
		tcbs[thread].prev = prev;
		tcbs[thread].next = next;
		prev->next = &tcbs[thread];
		next->prev = &tcbs[thread];
	}	
	PriCount[priority]++;	
	
	return 1;
}

int add_thread_sema4(Sema4Type *semaPt, uint32_t priority) {
	tcbType *priList = semaPt->TcbPri[priority];
	
	RunPt->status = semaPt;
	RunPt->next = 0;
	if(priList == 0) {	// first node
		semaPt->TcbPri[priority] = RunPt;
		RunPt->prev = 0;
	}
	else {
		while(priList->next) {
			priList = priList->next;
		}
		priList->next = RunPt;
		RunPt->prev = priList;
	}
	
	return 1;
}

int delete_thread_sema4(Sema4Type *semaPt) {
	uint32_t priority = 0;
	tcbType *priNode;
	
	for(priority = 0; priority < NUMPRI; priority++) {
		if(semaPt->TcbPri[priority] != 0) { break; }
	}
	if(priority == NUMPRI) {} // Shouldn't happen, ERROR
	
	priNode = semaPt->TcbPri[priority];
	semaPt->TcbPri[priority] = priNode->next;
	// Don't bother replacing priNode->next->prev
	
	return priNode->id;
}

int delete_thread_pri(uint32_t priority) {
  tcbType *next, *prev;
	
	prev = RunPt->prev;
	next = RunPt->next;
	
	prev->next = next;
	next->prev = prev;
	
	RunPtArray[priority] = prev;
	PriCount[priority]--;
	
	return 1;
}

int delete_thread(int thread) {
  if (available[thread]) {
    return -1;
    // Cannot release thread which is already available
  }
  available[thread] = 1;
	return 1;
}

int find_prev(int thread) {
  int ret;
  for (i = (thread+NUMTHREADS-1)%NUMTHREADS; i != thread; i = (i+NUMTHREADS-1)%NUMTHREADS ) {
    if (!available[i]) {
      ret = i;
      return ret;
    }
  }
  return -1;
}

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

//******** OSAddThreads ***************
// add three foregound threads to the scheduler
// Inputs: three pointers to a void/void foreground tasks
// Outputs: 1 if successful, 0 if this thread can not be added
int OSAddThreads(void(*task0)(void),
                 void(*task1)(void),
                 void(*task2)(void)){ int32_t status;
  status = StartCritical();
  tcbs[0].next = &tcbs[1]; // 0 points to 1
  tcbs[1].next = &tcbs[2]; // 1 points to 2
  tcbs[2].next = &tcbs[0]; // 2 points to 0
  SetInitialStack(0); Stacks[0][STACKSIZE-2] = (int32_t)(task0); // PC
  SetInitialStack(1); Stacks[1][STACKSIZE-2] = (int32_t)(task1); // PC
  SetInitialStack(2); Stacks[2][STACKSIZE-2] = (int32_t)(task2); // PC
  RunPt = &tcbs[0];       // thread 0 will run first
  EndCritical(status);
  return 1;               // successful
}

void InitTimer2A(uint32_t period);
void InitTimer3A(uint32_t period);
static uint32_t MSTime;
// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers 
// input:  none
// output: none
void OS_Init(void) {
  OS_DisableInterrupts();
  PLL_Init();                 // set processor clock to 80 MHz
#ifdef RROBIN
  InitAvailable();						// all locations free
#else	
  InitPriorities();						// initial priority = 0
#endif
  InitTimer2A(TIME_1MS);			// sleep decrementer
  InitTimer3A(0xFFFFFFFF);		// OS time timer
  UART_Init();              	// initialize UART
  Output_Init();							// initialize ST7735 LCD
  MSTime = 0;									// current OS time = 0
	Launched = 0;
  
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
															// lowest PRI so only foreground interrupted
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7

	// Use PendSV to trigger a context switch
	NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00D00000; // priority 6	
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value) { 
  int i;
	
	semaPt->Value = value;   // Should be free first (>0)
	for(i = 0; i < NUMPRI; i++) {
		semaPt->TcbPri[i] = 0;
	}
} 

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt) { 
  int32_t status; 
	
	status = StartCritical();
	
	semaPt->Value -= 1;
  if(semaPt->Value < 0) {
		delete_thread_pri(RunPt->priority);
		add_thread_sema4(semaPt, RunPt->priority);
		EndCritical(status);
		OS_Suspend();
		status = StartCritical();
	}
	
	EndCritical(status);
} 

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt) { 
  int32_t status, thread, priority;
	
  status = StartCritical();
  
	semaPt->Value += 1;
	if(semaPt->Value <= 0) {
		thread = delete_thread_sema4(semaPt);
		priority = tcbs[thread].priority;
		add_thread_pri(thread, priority);
		
		if(priority < PriLevel) {
			EndCritical(status);
			OS_Suspend();
			status = StartCritical();
		}
	}
	
  EndCritical(status);
} 

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt) { 
  OS_DisableInterrupts();
  while(semaPt->Value == 0) {
    OS_EnableInterrupts();
	  OS_Suspend();       // run thread switcher
	  OS_DisableInterrupts();
  }
  semaPt->Value = 0;
  OS_EnableInterrupts();
} 

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt) { 
  int32_t status;
  status = StartCritical();
  semaPt->Value = 1;
  EndCritical(status);
} 

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
static uint32_t NumThreads = 0;
int OS_AddThread(void(*task)(void), 
   unsigned long stackSize, unsigned long priority) {		 
	int32_t status, thread, error; 
	 
  status = StartCritical();
  
	thread = add_thread();
	error = add_thread_pri(thread, priority);
	
	if(NumThreads == 0) {
		RunPt = &tcbs[thread];     // thread 0 will run first
		PriLevel = priority;
	}
	
  tcbs[thread].status   = 0;
  tcbs[thread].sleepCt  = 0;
  tcbs[thread].age      = 0;
  tcbs[thread].id       = thread;
  tcbs[thread].priority = priority;
	
	SetInitialStack(thread); 
	Stacks[thread][STACKSIZE-2] = (int32_t)(task); // PC
	NumThreads++;
  EndCritical(status);
	
  return error;               // successful
}

int OS_AddThread_RoundRobin(void(*task)(void), 
   unsigned long stackSize, unsigned long priority) {		 
	int32_t status, thread, prev;
	 
  status = StartCritical();
	
	thread = add_thread();
	if(NumThreads == 0) {
		tcbs[0].next = &tcbs[0]; // 0 points to 0
		RunPt = &tcbs[0];     // thread 0 will run first
	}
	else {
		prev = find_prev(thread);
		tcbs[thread].next = tcbs[prev].next;
		tcbs[prev].next = &tcbs[thread];
	}
	
  tcbs[thread].status   = 0;
  tcbs[thread].sleepCt  = 0;
  tcbs[thread].age      = 0;
  tcbs[thread].id       = thread;
  tcbs[thread].priority = priority;
	
	SetInitialStack(thread); 
	Stacks[thread][STACKSIZE-2] = (int32_t)(task); // PC
	NumThreads++;
  EndCritical(status);
	
  return 1;               // successful
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void) { 
	return RunPt->id;
}

void InitTimer3A(uint32_t period) {
	long sr;
	volatile unsigned long delay;
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x08;
	
  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;
	
  TIMER3_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER3_TAILR_R = 0xFFFFFFFF - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;
  TIMER3_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 15-13 for timer1A
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|(1 << 29);	//3
  NVIC_EN1_R = NVIC_EN1_INT35;     // 8) enable interrupt 21 in NVIC
  TIMER3_TAPR_R = 0;
  TIMER3_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer1A
	
  EndCritical(sr);
}

void Timer3A_Handler(void){ 
	
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
//	SystemTime = SystemTime + 1;

	GPIO_PORTE_DATA_R ^= 0x02;
	
}

//struct PeriodicTcb {
//	void (*PeriodicTask)(void);
//	uint32_t priority;
//	uint32_t period;
//};
//typedef struct PeriodicTcb PeriodicTcbType;
//PeriodicTcbType PeriodicThreads[5];
//static uint32_t MinPeriod = 0;
//static uint32_t NumPeriodicTasks = 0;

void (*PeriodicTask)(void);
void (*PeriodicTask2)(void);
void InitTimer1A(uint32_t period, uint32_t priority) {
	long sr;
	volatile unsigned long delay;
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x02;
	
  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;
	
  TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER1_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;
  TIMER1_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 15-13 for timer1A
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|((priority+3) << 13);	//3
  NVIC_EN0_R = NVIC_EN0_INT21;     // 8) enable interrupt 21 in NVIC
  TIMER1_TAPR_R = 0;
  TIMER1_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer1A
	
  EndCritical(sr);
}

void Timer1A_Handler(void){ 
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
	(*PeriodicTask)();
//	for(i = 0; i < NumPeriodicTasks; i++) {
//		if(
//	}
}

void InitTimer4A(uint32_t period, uint32_t priority) {
	long sr;
	volatile unsigned long delay;
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x10;
	
  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;
	
  TIMER4_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER4_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER4_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER4_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;
  TIMER4_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 15-13 for timer1A
  NVIC_PRI17_R = (NVIC_PRI17_R&0xFF00FFFF)|((priority+3) << 21);	//3
  NVIC_EN2_R = NVIC_EN2_INT70;     // 8) enable interrupt 21 in NVIC
  TIMER4_TAPR_R = 0;
  TIMER4_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer1A
	
  EndCritical(sr);
}

void Timer4A_Handler(void){ 
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
	(*PeriodicTask2)();
}
//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), 
   unsigned long period, unsigned long priority) { 	
//	PeriodicThreads[NumPeriodicTasks].PeriodicTask = task;
//	PeriodicThreads[NumPeriodicTasks].priority = priority;
//	PeriodicThreads[NumPeriodicTasks].period = period;
		 
//	if(period < MinPeriod) {
//		InitTimer1A(period);
//	}
	static uint32_t NumPeriodicTasks = 0;
	
	if(NumPeriodicTasks == 0) {	
		PeriodicTask = task;
		InitTimer1A(period, priority);
	} 
	else {
		PeriodicTask2 = task;
		InitTimer4A(period, priority);
	}
	NumPeriodicTasks++;
  return 1;
}

uint32_t LastPF4, LastPF0;

void (*SWOneTask)(void);
void SWOneInit(uint32_t priority){
  unsigned long volatile delay;
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTF_CR_R = 0x10;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0;    //     PF4,PF0 is not both edges
  GPIO_PORTF_ICR_R = 0x10; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x10;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R |= 0x10;      //
	GPIO_PORTF_ICR_R |= 0x10;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4,PF0

  LastPF4 = GPIO_PORTF_DATA_R & 0x10;

  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|((priority+2) << 21); // (g) priority 2
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void (*SWTwoTask)(void);
void SWTwoInit(uint32_t priority){
  unsigned long volatile delay;
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x01;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x01;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x01;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x01;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000F000F; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x01;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x01;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x01;     // (d) PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R |= 0x01;    //     PF4,PF0 is both edges
  GPIO_PORTF_ICR_R |= 0x01;      // (e) clear flags 4,0
  GPIO_PORTF_IM_R |= 0x01;      // (f) arm interrupt on PF4,PF0

  LastPF0 = GPIO_PORTF_DATA_R & 0x01;

  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|((priority+2) << 21); // (g) priority 5 0x00400000
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
}


void static DebouncePF4(void) {
  OS_Sleep(2);      //foreground sleep, must run within 5ms
  LastPF4 = GPIO_PORTF_DATA_R & 0x10;
  GPIO_PORTF_ICR_R = 0x10;
  GPIO_PORTF_IM_R |= 0x10;
  OS_Kill(); 
}

void static DebouncePF0(void) {
  OS_Sleep(2);      //foreground sleep, must run within 5ms
  LastPF0 = GPIO_PORTF_DATA_R & 0x01;
  GPIO_PORTF_ICR_R = 0x01;
  GPIO_PORTF_IM_R |= 0x01;
  OS_Kill(); 
}

void GPIOPortF_Handler(void) {  // called on touch of either SW1 or SW2
  if(GPIO_PORTF_RIS_R&0x01) {   // SW2 touch
    if (LastPF0) { (*SWTwoTask)(); }
    GPIO_PORTF_IM_R &= ~0x01;
    OS_AddThread(&DebouncePF0, 128, 2);
  }
  if(GPIO_PORTF_RIS_R&0x10) {   // SW1 touch
		if (LastPF4) { (*SWOneTask)(); }
    GPIO_PORTF_IM_R &= ~0x10;
    OS_AddThread(&DebouncePF4, 128 ,2);
  }
}

//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), unsigned long priority) { 
	SWOneTask = task;
	SWOneInit(priority);
	
	return 1;
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), unsigned long priority) { 
	SWTwoTask = task;
	SWTwoInit(priority);
	
	return 1;
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime) { 
	RunPt->sleepCt = sleepTime;
	OS_Suspend(); // May need to change
} 

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void) { 
	uint32_t thread, priority; 
	GPIO_PORTD_DATA_R ^= 0x01;
	thread = RunPt->id;
	priority = RunPt->priority;

	delete_thread_pri(priority);
	delete_thread(thread);

	NumThreads--;
	GPIO_PORTD_DATA_R ^= 0x01;
	OS_Suspend();
}

void OS_Kill_Round_Robin(void) { 
	uint32_t thread, prev;
	thread = RunPt->id;
	prev = find_prev(thread);
	delete_thread(thread);
	NumThreads--;
	tcbs[prev].next = tcbs[thread].next;
	OS_Suspend();
}  

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void) { 
	GPIO_PORTD_DATA_R ^= 0x02;
#ifdef WITH_SYSTICK	
	NVIC_ST_CURRENT_R = 0;					// clear counter
	GPIO_PORTD_DATA_R ^= 0x02;
	NVIC_INT_CTRL_R = 0x04000000;		// trigger SysTick
#else
	NextRunPt = RunPt->next;
	GPIO_PORTD_DATA_R ^= 0x02;
	NVIC_INT_CTRL_R = 0x10000000;		// trigger PendSV
#endif
}

uint16_t static OS_Fifo [OSFIFOSIZE];
uint16_t *PutPt, *GetPt;
Sema4Type FifoAvailable;
//Sema4Type DataRoomLeft;

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
void OS_Fifo_Init(unsigned long size) {
  long sr;  
  sr = StartCritical();        
	
	OS_InitSemaphore(&FifoAvailable, 0);
	//OS_InitSemaphore(&DataRoomLeft, OSFIFOSIZE);
		
  PutPt = GetPt = &OS_Fifo[0]; 
  EndCritical(sr); 	
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data) { 
  uint16_t volatile *nextPutPt; 
	
  nextPutPt = PutPt + 1;        
  if(nextPutPt == &OS_Fifo[OSFIFOSIZE]){ 
    nextPutPt = &OS_Fifo[0];       
  }                                     
  if(nextPutPt == GetPt ){      
    return(0);                       
  }                                     
  else{                                 
    *( PutPt ) = data;          
    PutPt = nextPutPt; 
		OS_Signal(&FifoAvailable);
    return(1); 
	}		
}  

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void) { 
	uint16_t data;
	
	// ERROR CHECKING
	OS_Wait(&FifoAvailable);
	if( PutPt == GetPt ){ 
    return(0);                       
  }                                     
  
	data = *( GetPt++);    
  if( GetPt == &OS_Fifo[OSFIFOSIZE]){ 
    GetPt = &OS_Fifo[0];   
  }                                     
  
	return(data); 
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void) { 
	if( PutPt < GetPt ){  
		return ((unsigned short)( PutPt - GetPt + (OSFIFOSIZE*sizeof(uint16_t)))/sizeof(uint16_t)); 
  }                                     
  return ((unsigned short)( PutPt - GetPt )/sizeof(uint16_t)); 
}

/*FOLLOW GUIDELINES IN LAB MANUAL*/
static Sema4Type DataValid;
static Sema4Type BoxFree;
static uint32_t MailBox;
// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void) { 
	OS_InitSemaphore(&DataValid, 0);
	OS_InitSemaphore(&BoxFree, 1);
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data) {
	OS_bWait(&BoxFree);
	MailBox = data;
	OS_bSignal(&DataValid);
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void) { 
	uint32_t retVal;
	
	OS_bWait(&DataValid);
	retVal = MailBox;
	OS_bSignal(&BoxFree);
	
	return retVal;
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void) { 
	return TIMER3_TAILR_R - TIMER3_TAV_R;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop) {
	unsigned long difference;
	if (stop < start) {
		difference = 0xFFFFFFFF - start + stop;
	} 
	else {
		difference = stop-start;
	}
	return difference;
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void) {
	MSTime = 0;
}

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void) {
  //uint32_t retVal = (OS_Time()-MSTime)/TIME_1MS;	
	return MSTime;
}

void InitTimer2A(uint32_t period) {
	long sr;
	volatile unsigned long delay;
	
	sr = StartCritical();
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
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|(2 << 29);	
  NVIC_EN0_R = NVIC_EN0_INT23;     // 8) enable interrupt 23 in NVIC
  TIMER2_TAPR_R = 0;
  TIMER2_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer2A
	
  EndCritical(sr);
}

void Timer2A_Handler(void){ 
	int j;
	
	TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer2A timeout
	MSTime++;
	
	for(j = 0; j < NUMTHREADS; j++) {
		if(!available[j]) {
			tcbs[j].age += 1;
			if(tcbs[j].sleepCt) {
				tcbs[j].sleepCt -= 1;
			}
		}
	}
}

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(unsigned long theTimeSlice) {
#ifdef WITH_SYSTICK
	NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
#endif
  Launched = 1;
  StartOS();                   // start on the first task
}

static int pri;
void SysTick_Handler(void) {
	GPIO_PORTD_DATA_R ^= 0x08;
	RunPtArray[PriLevel] = RunPtArray[PriLevel]->next;
	
	pri = 0;
	while((pri < NUMPRI) && (PriCount[pri] == 0)) {
		pri++;
	}
	if(pri == NUMPRI) {}// ERROR, NO ACTIVE THREADS, ADD IDLE TASK 
	PriLevel = pri;
	
	while(PriLevel < NUMPRI) {
		for(pri = 0; (pri < PriCount[PriLevel]) && RunPtArray[PriLevel]->sleepCt; pri++) {
			RunPtArray[PriLevel] = RunPtArray[PriLevel]->next;
		}
		if(pri == PriCount[PriLevel]) {
			PriLevel++;
		}
		else { break; }
	}
	if(PriLevel == NUMPRI) {} // Error

	NextRunPt = RunPtArray[PriLevel];
	GPIO_PORTD_DATA_R ^= 0x08;
  NVIC_INT_CTRL_R = 0x10000000;		// trigger PendSV
}

void SysTick_RoundRobin(void) {	
	NextRunPt = RunPt->next;
	
	while(NextRunPt->sleepCt) {
		NextRunPt = NextRunPt->next;
	}

  NVIC_INT_CTRL_R = 0x10000000;		// trigger PendSV
}
