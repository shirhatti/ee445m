//*****************************************************************************
//
// Lab5.c - user programs, File system, stream data onto disk
// Jonathan Valvano, March 16, 2011, EE345M
//     You may implement Lab 5 without the oLED display
//*****************************************************************************
// PF1/IDX1 is user input select switch
// PE1/PWM5 is user input down switch 
#include <stdio.h>
#include "diskio.h"
#include "efile.h"

// From Lab 4
#include "os.h"
#include "inc/tm4c123gh6pm.h"
#include <string.h> 
#include "ST7735.h"
#include "ADCT0ATrigger.h"
#include "UART.h"
#include "cmdline.h"

unsigned long NumCreated;   // number of foreground threads created
unsigned long NumSamples;   // incremented every sample
unsigned long DataLost;     // data sent by Producer, but not received by Consumer

int Running;                // true while robot is running

#define TIMESLICE 2*TIME_1MS  // thread switch time in system time units

// PF1 is profile for eDisk_WriteBlock
// PF2 on for running, low for error
// PF3 is profile for eDisk_ReadBlock
#define GPIO_PF0  (*((volatile unsigned long *)0x40025004))
#define PF1  (*((volatile unsigned long *)0x40025008))
#define PF2  (*((volatile unsigned long *)0x40025010))
#define PF3  (*((volatile unsigned long *)0x40025020))
#define GPIO_PG1  (*((volatile unsigned long *)0x40026008))
// PE1/PWM5 is user input down switch 
// PF0/PWM0 is debugging output on Systick 
// PG1/PWM1 is debugging output 

int realmain(void);
int testmain1(void);
int testmain2(void);
	
int main(void) {
	testmain1();
}

void PortE_Init(void){ 
	unsigned long volatile delay;
  SYSCTL_RCGCGPIO_R |= 0x10;       // activate port E
  delay = SYSCTL_RCGCGPIO_R;        
  delay = SYSCTL_RCGCGPIO_R;         
  GPIO_PORTE_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTE_AFSEL_R &= ~0x0F;   // disable alt funct on PE3-0
  GPIO_PORTE_DEN_R |= 0x0F;     // enable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R = ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R &= ~0x0F;;      // disable analog functionality on PF
	
	SYSCTL_RCGCGPIO_R |= 8;
	delay = SYSCTL_RCGCGPIO_R;
	delay = SYSCTL_RCGCGPIO_R;
	GPIO_PORTD_DIR_R = 0;
	GPIO_PORTD_DEN_R = 0xFF;
	GPIO_PORTD_AMSEL_R = 0;
	GPIO_PORTD_AFSEL_R = 0;
	GPIO_PORTD_DATA_R = 0;
}

void PortF_Init(void){  unsigned long volatile delay;
  SYSCTL_RCGCGPIO_R |= 0x20; // activate port F
  delay = SYSCTL_RCGCGPIO_R;
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R |= 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_DIR_R = 0x0E;     // make PF3-1 output (PF3-1 built-in LEDs),PF4,0 input
  GPIO_PORTF_PUR_R = 0x11;     // PF4,0 have pullup
  GPIO_PORTF_AFSEL_R = 0x00;   // disable alt funct on PF4-0
  GPIO_PORTF_DEN_R = 0x1F;     // enable digital I/O on PF4-0
  GPIO_PORTF_PCTL_R = 0x00000000;
  GPIO_PORTF_AMSEL_R = 0;      // disable analog functionality on PF
}

//******** Robot *************** 
// foreground thread, accepts data from producer
// inputs:  none
// outputs: none
void Robot(void){   
unsigned long data;      // ADC sample, 0 to 1023
unsigned long voltage;   // in mV,      0 to 3000
unsigned long time;      // in 10msec,  0 to 1000 
unsigned long t=0;
  OS_ClearMsTime();    
  DataLost = 0;          // new run with no lost data 
  ST7735_OutString("Robot running...");
  eFile_RedirectToFile("Robot");
  printf("time(sec)\tdata(volts)\n\r");
  do{
    t++;
    time=OS_MsTime();            // 10ms resolution in this OS
    data = OS_Fifo_Get();        // 1000 Hz sampling get from producer
    voltage = (300*data)/1024;   // in mV
    printf("%0u.%02u\t%0u.%03u\n\r",time/100,time%100,voltage/1000,voltage%1000);
  }
  while(time < 1000);       // change this to mean 10 seconds
  eFile_EndRedirectToFile();
  ST7735_OutString("done.\n\r");
  Running = 0;                // robot no longer running
  OS_Kill();
}
  
//************ButtonPush*************
// Called when Select Button pushed
// background threads execute once and return
void ButtonPush(void){
  if(Running==0){
    Running = 1;  // prevents you from starting two robot threads
    NumCreated += OS_AddThread(&Robot,128,1);  // start a 20 second run
  }
}
//************DownPush*************
// Called when Down Button pushed
// background threads execute once and return
void DownPush(void){

}

//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 1 kHz, started by your ADC_Collect
// The timer triggers the ADC, creating the 1 kHz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 12-bit sample 
// sends data to the Robot, runs periodically at 1 kHz
// inputs:  none
// outputs: none
void Producer(unsigned long data){  
  if(Running){
    if(OS_Fifo_Put(data)){     // send to Robot
      NumSamples++;
    } else{ 
      DataLost++;
    } 
  }
}
 
//******** IdleTask  *************** 
// foreground thread, runs when no other work needed
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
static unsigned long Idlecount=0;
void IdleTask(void){ 
  while(1) { 
    Idlecount++;        // debugging 
  }
}

//******** Interpreter **************
// your intepreter from Lab 4 
// foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none

// add the following commands, remove commands that do not make sense anymore
// 1) format 
// 2) directory 
// 3) print file
// 4) delete file
// execute   eFile_Init();  after periodic interrupts have started
void BackspacePreprocessString(char* string) {
	int index = 0;
	int offset = 0;
	while (string[index+offset]) {
		string[index] = string[index+offset];
		// PuTTY config generates 0x7F for backspace
		if (string[index+offset] == 0x7F) {
			index--;
			offset+=2;
		}
		else {
			index++;
		}
	}
	string[index] = string[index+offset];
	UART_OutString("\r\n");
}

void Interpreter(void) {
	char string[80];
	eFile_Init();
  while(1){
    OutCRLF(); UART_OutString(">");
    UART_InString(string,79);
		BackspacePreprocessString(string);
		if(CmdLineProcess(string) == -1) {
			UART_OutString("\r\ncommand not recognized");
		}
  }
}

//*******************lab 5 main **********
int realmain(void){        // lab 5 real main
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
	PortF_Init();

  Running = 0;         // robot not running
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;

//********initialize communication channels
  OS_Fifo_Init(512);    // ***note*** 4 is not big enough*****
  ADC_Collect(0, 1000, &Producer); // start ADC sampling, channel 0, 1000 Hz

//*******attach background tasks***********
  OS_AddSW1Task(&ButtonPush,2);
  OS_AddSW2Task(&DownPush,3);
//  OS_AddPeriodicThread(disk_timerproc,10*TIME_1MS,3);

  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&Interpreter,128,2); 
  NumCreated += OS_AddThread(&IdleTask,128,5);  // runs when nothing useful to do
 
  OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
  return 0;             // this never executes
}


//*****************test programs*************************
unsigned char buffer[512];
#define MAXBLOCKS 100
//void diskError(char* errtype, unsigned long n){
//  PF2 = 0x00;      // turn LED off to indicate error
//  OS_Kill();
//}

void TestDisk(void){  DSTATUS result;  unsigned short block;  int i; unsigned long n;
  // simple test of eDisk
//  result = eDisk_Init(0);  // initialize disk
//  if(result) diskError("eDisk_Init",result);
		eFile_Init();
  n = 1;    // seed
  for(block = 0; block < MAXBLOCKS; block++){
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      buffer[i] = 0xFF&n;
    }
    PF1 = 0x02;
//    if(eDisk_WriteBlock(buffer,block))diskError("eDisk_WriteBlock",block); // save to disk
    PF1 = 0;
  }
  n = 1;  // reseed, start over to get the same sequence
  for(block = 0; block < MAXBLOCKS; block++){
    PF3 = 0x08;
 //   if(eDisk_ReadBlock(buffer,block))diskError("eDisk_ReadBlock",block); // read from disk
    PF3 = 0;
    for(i=0;i<512;i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      if(buffer[i] != (0xFF&n)){
          PF2 = 0x00;   // turn LED off to indicate error
					OS_Kill();
      }
    }
  }
	OS_Kill();
}

void RunTest(void){
  NumCreated += OS_AddThread(&TestDisk,128,1);  
}

//******************* test main1 **********
// SYSTICK interrupts, period established by OS_Launch
// Timer interrupts, period established by first call to OS_AddPeriodicThread
int testmain1(void){   // testmain1
	OS_Init();           // initialize, disable interrupts
	PortE_Init();
	PortF_Init();

//*******attach background tasks***********
//  OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk
  OS_AddSW1Task(&RunTest,2);
  
  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&TestDisk,128,1);  
  NumCreated += OS_AddThread(&IdleTask,128,5); 
 
  OS_Launch(2*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}

void TestFile(void){   int i; char data; 
  UART_OutString("\n\rEE345M/EE380L, Lab 5 eFile test\n\r");
  // simple test of eFile
//  if(eFile_Init())              diskError("eFile_Init",0); 
//  if(eFile_Format())            diskError("eFile_Format",0); 
//  eFile_Directory(&ST7735_OutChar);
//  if(eFile_Create("file1"))     diskError("eFile_Create",0);
//  if(eFile_WOpen("file1"))      diskError("eFile_WOpen",0);
//  for(i=0;i<1000;i++){
//    if(eFile_Write('a'+i%26))   diskError("eFile_Write",i);
//    if(i%52==51){
//      if(eFile_Write('\n'))     diskError("eFile_Write",i);  
//      if(eFile_Write('\r'))     diskError("eFile_Write",i);
//    }
//  }
//  if(eFile_WClose())            diskError("eFile_Close",0);
//  eFile_Directory(&ST7735_OutChar);
//  if(eFile_ROpen("file1"))      diskError("eFile_ROpen",0);
//  for(i=0;i<1000;i++){
//    if(eFile_ReadNext(&data))   diskError("eFile_ReadNext",i);
//    ST7735_OutChar(data);
//  }
//  if(eFile_Delete("file1"))     diskError("eFile_Delete",0);
//  eFile_Directory(&ST7735_OutChar);
//  UART_OutString("Successful test of creating a file\n\r");
  OS_Kill();
}

//******************* test main2 **********
// SYSTICK interrupts, period established by OS_Launch
// Timer interrupts, period established by first call to OS_AddPeriodicThread
int testmain2(void){ 
  OS_Init();           // initialize, disable interrupts
	PortE_Init();
	PortF_Init();
	
//*******attach background tasks***********
//  OS_AddPeriodicThread(&disk_timerproc,10*TIME_1MS,0);   // time out routines for disk
  
  NumCreated = 0 ;
// create initial foreground threads
  NumCreated += OS_AddThread(&TestFile,128,1);  
  NumCreated += OS_AddThread(&IdleTask,128,3); 
 
  OS_Launch(10*TIME_1MS); // doesn't return, interrupts enabled in here
  return 0;               // this never executes
}
