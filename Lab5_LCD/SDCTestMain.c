// SDCTestMain.c
// Runs on TM4C123/LM4F120
// Test Secure Digital Card read/write interface by writing test
// data, reading them back, and verifying that they match as
// expected.  Running the unformatted file tests will destroy
// any formatting already on the disk.  The formatted file tests
// will not work unless the disk has already been formatted.
// Valvano
// March 17, 2014

/* This example accompanies the books
   Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers, Volume 3,  
   ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2013

   Volume 3, Program 6.3, section 6.6   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) connected to PA4 (SSI0Rx)
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss) <- GPIO high to disable TFT
// CARD_CS (pin 5) connected to PD7 GPIO output 
// Data/Command (pin 4) connected to PA6 (GPIO)<- GPIO low not using TFT
// RESET (pin 3) connected to PA7 (GPIO)<- GPIO high to disable TFT
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

#include <stdint.h>
#include "..//inc/tm4c123gh6pm.h"
#include "PLL.h"
#include "diskio.h"
#include "ST7735.h"
#include "efile.h"


// PF1 is profile for eDisk_WriteBlock
// PF2 on for running, low for error
// PF3 is profile for eDisk_ReadBlock
unsigned char buffer[512];
#define MAXBLOCKS 100
#define PF1  (*((volatile unsigned long *)0x40025008))
#define PF2  (*((volatile unsigned long *)0x40025010))
#define PF3  (*((volatile unsigned long *)0x40025020))

// The simple unformatted test will destroy the formatting and
// erase everything on the SD card.
void SimpleUnformattedTest(void){ DSTATUS result; uint16_t block; int i; uint32_t n; uint32_t errors = 0;
  // simple test of eDisk
  result = disk_initialize(0);  // initialize disk
  if(result) diskError("disk_initialize", result, 0);
  n = 1;    // seed
  for(block = 0; block < MAXBLOCKS; block++){
    for(i=0; i<512; i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      buffer[i] = 0xFF&n;
    }
    result = disk_write (0,buffer, block, 1);
    if(result) diskError("disk_write", result, block); // save to disk
  }
  n = 1;  // reseed, start over to get the same sequence
  for(block = 0; block < MAXBLOCKS; block++){
    result = disk_read (0,buffer, block,1);
    if(result) diskError("disk_read ", result, block); // read from disk
    for(i=0; i<512; i++){
      n = (16807*n)%2147483647; // pseudo random sequence
      if(buffer[i] != (0xFF&n)){
        errors = errors + 1;
      }
    }
  }
  ST7735_DrawString(0, 0, "Test done", ST7735_Color565(0, 255, 0));
  ST7735_DrawString(0, 1, "Mismatches:", ST7735_Color565(0, 255, 0));
  ST7735_SetCursor(12, 1);
  ST7735_SetTextColor(ST7735_Color565(0, 255, 0));
  ST7735_OutUDec(errors);
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

void test1(void);

int main(void){ 
	int i=0;
	char ch;
  PLL_Init();    // bus clock at 80 MHz
  PortF_Init();  // LaunchPad switches and LED
  PF2 = 0x04;    // turn blue LED on
	Output_Init();
	test1();
// *******************unformatted file tests********************
  while(1){
    //SimpleUnformattedTest();
    i = i + 1;
  }
// ****************end of unformatted file tests****************
}

void test1() {
	int i, result = 0, x=0, y = 0;
	char ch;
	result = eFile_Init();
//	result = eFile_Format();
//	eFile_Create("file1");
//	ls(currentDirectoryBlock);
//	eFile_WOpen("file1");
//  for(i=0;i<600;i++){
//		eFile_Write('a'+i%26);
//		if(i%15==15){
//      eFile_Write('\n');  
//      eFile_Write('\r');
//    }
//	}
//	eFile_Write('\0');
//	eFile_WClose();
//	eFile_ROpen("file1");
//	ST7735_SetCursor(0, 0);
//	Output_Clear();
//  while(1){
//		
//		eFile_ReadNext(&ch);
//		if (ch == '\0') break;
//		ST7735_DrawChar(x, y, ch, ST7735_Color565(255, 255, 0), 0, 1);
//		x = x + 6;
//		if(x > 122){
//			x = 0;                          // start over on the next line
//			y = y + 10;
//		}
//		if(y > 150){
//			y = 10;                         // the screen is full
//		}
//		//ST7735_OutChar(ch);
//	}
//	eFile_RClose();
//	Output_Clear();
//	ls(currentDirectoryBlock);
//	Output_Clear();
//	//eFile_Delete("file1");
//	ls(currentDirectoryBlock);
//	Output_Clear();
	eFile_ROpen("file1");
	eFile_WOpen("file1");
	eFile_Write('t');eFile_Write('e');eFile_Write('s');eFile_Write('t');eFile_Write('\0');
	eFile_WClose();
	eFile_RClose();
	eFile_ROpen("file2");
	ST7735_SetCursor(0, 0);
	Output_Clear();
	x=0;y=0;
  while(1){
		
		eFile_ReadNext(&ch);
		if (ch == '\0') break;
		ST7735_DrawChar(x, y, ch, ST7735_Color565(255, 255, 0), 0, 1);
		x = x + 6;
		if(x > 122){
			x = 0;                          // start over on the next line
			y = y + 10;
		}
		if(y > 150){
			y = 10;                         // the screen is full
		}
		ST7735_OutChar(ch);
	}
	eFile_RClose();
}