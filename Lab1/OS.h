#ifndef __OS_H__ // do not include more than once
#define __OS_H__

/**************OS_AddPeriodicThread***************
 Activate Timer0 interrupts to run user task periodically
   Input: task is a pointer to a user function
				  period in units (1/clockfreq)
				  priority (0-7)
 Outputs: error (1); success (0)
*************************************************/ 
int OS_AddPeriodicThread(void (*task)(void), unsigned long period, unsigned long priority);

/**************OS_ClearPeriodicTime***************
 clear the 32-bit global counter
  Input: none
  Output: none
*************************************************/
void OS_ClearPeriodicTime(void);

/**************OS_ReadPeriodicTime***************
 reads the current value of the 32-bit global counter
  Input: none
  Output: current counter value
*************************************************/
unsigned long OS_ReadPeriodicTime(void);

#endif // __OS_H__
