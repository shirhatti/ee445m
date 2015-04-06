/***************************************************
Modified by Sourabh Shirhatti and Nelson Wu for EE 445M, Spring 2015
****************************************************/

//*****************************************************************************
//
// cmdline.c - Functions to help with processing command lines.
//
// Copyright (c) 2007-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.0.1.11577 of the Tiva Utility Library.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup cmdline_api
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "cmdline.h"
#include "UART.h"
#include "ST7735.h"
#include "os.h"
#include "ADCT0ATrigger.h"

//*****************************************************************************
//
// Defines the maximum number of arguments that can be parsed.
//
//*****************************************************************************
#ifndef CMDLINE_MAX_ARGS
#define CMDLINE_MAX_ARGS        8
#endif

//*****************************************************************************
//
// An array to hold the pointers to the command line arguments.
//
//*****************************************************************************
static char *g_ppcArgv[CMDLINE_MAX_ARGS + 1];

//*****************************************************************************
//
//! Process a command line string into arguments and execute the command.
//!
//! \param pcCmdLine points to a string that contains a command line that was
//! obtained by an application by some means.
//!
//! This function will take the supplied command line string and break it up
//! into individual arguments.  The first argument is treated as a command and
//! is searched for in the command table.  If the command is found, then the
//! command function is called and all of the command line arguments are passed
//! in the normal argc, argv form.
//!
//! The command table is contained in an array named <tt>g_psCmdTable</tt>
//! containing <tt>tCmdLineEntry</tt> structures which must be provided by the
//! application.  The array must be terminated with an entry whose \b pcCmd
//! field contains a NULL pointer.
//!
//! \return Returns \b CMDLINE_BAD_CMD if the command is not found,
//! \b CMDLINE_TOO_MANY_ARGS if there are more arguments than can be parsed.
//! Otherwise it returns the code that was returned by the command function.
//
//*****************************************************************************
int
CmdLineProcess(char *pcCmdLine)
{
    char *pcChar;
    uint_fast8_t ui8Argc;
    bool bFindArg = true;
    tCmdLineEntry *psCmdEntry;

    //
    // Initialize the argument counter, and point to the beginning of the
    // command line string.
    //
    ui8Argc = 0;
    pcChar = pcCmdLine;

    //
    // Advance through the command line until a zero character is found.
    //
    while(*pcChar)
    {
        //
        // If there is a space, then replace it with a zero, and set the flag
        // to search for the next argument.
        //
        if(*pcChar == ' ')
        {
            *pcChar = 0;
            bFindArg = true;
        }

        //
        // Otherwise it is not a space, so it must be a character that is part
        // of an argument.
        //
        else
        {
            //
            // If bFindArg is set, then that means we are looking for the start
            // of the next argument.
            //
            if(bFindArg)
            {
                //
                // As long as the maximum number of arguments has not been
                // reached, then save the pointer to the start of this new arg
                // in the argv array, and increment the count of args, argc.
                //
                if(ui8Argc < CMDLINE_MAX_ARGS)
                {
                    g_ppcArgv[ui8Argc] = pcChar;
                    ui8Argc++;
                    bFindArg = false;
                }

                //
                // The maximum number of arguments has been reached so return
                // the error.
                //
                else
                {
                    return(CMDLINE_TOO_MANY_ARGS);
                }
            }
        }

        //
        // Advance to the next character in the command line.
        //
        pcChar++;
    }

    //
    // If one or more arguments was found, then process the command.
    //
    if(ui8Argc)
    {
        //
        // Start at the beginning of the command table, to look for a matching
        // command.
        //
        psCmdEntry = &g_psCmdTable[0];

        //
        // Search through the command table until a null command string is
        // found, which marks the end of the table.
        //
        while(psCmdEntry->pcCmd)
        {
            //
            // If this command entry command string matches argv[0], then call
            // the function for this command, passing the command line
            // arguments.
            //
            if(!strcmp(g_ppcArgv[0], psCmdEntry->pcCmd))
            {
                return(psCmdEntry->pfnCmd(ui8Argc, g_ppcArgv));
            }

            //
            // Not found, so advance to the next entry.
            //
            psCmdEntry++;
        }
    }

    //
    // Fall through to here means that no matching command was found, so return
    // an error.
    //
    return(CMDLINE_BAD_CMD);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

// Used for UART commands
char HelpADC[] = "Send commands to the ADC";
char HelpLCD[] = "Send commands to the LCD";
char HelpOS[] = "Send commands to the OS";
char HelpEcho[] = "Echo command";

/*******************CommandADC********************
 Send debugging commands to the ADC
   Input: ui8Argc is the number of commands parsed
          g_ppcArgv is an array containing the commands
 Outputs: error (-1); success (channel #)
*************************************************/ 
void CommandADC(uint_fast8_t ui8Argc, char *g_ppcArgv[]) {
    uint32_t channel;
//		uint32_t Fs, numSamples;
//    uint16_t buffer[100];
	
    //Check if number of arguments are correct
    if (ui8Argc != 3) {
        OutCRLF();
        UART_OutString("All adc functions require exactly 1 parameter");
        return;
    }
    // Check if a valid channel
   if (strcmp(g_ppcArgv[2],"0")==0 || (strcmp(g_ppcArgv[2],"1")==0) || strcmp(g_ppcArgv[2],"2")==0 || (strcmp(g_ppcArgv[2],"3")==0) || strcmp(g_ppcArgv[2],"4")==0 || (strcmp(g_ppcArgv[2],"5")==0) || strcmp(g_ppcArgv[2],"6")==0 || (strcmp(g_ppcArgv[2],"7")==0) || strcmp(g_ppcArgv[2],"8")==0 || (strcmp(g_ppcArgv[2],"9")==0) || strcmp(g_ppcArgv[2],"10")==0 || (strcmp(g_ppcArgv[2],"11")==0)) {
            channel = atoi(g_ppcArgv[2]);
    } else {
        OutCRLF();
        UART_OutString("Invalid channel number specified to adc function");
        return;
    }
    
    // Check for command open
    if (strcmp("open", g_ppcArgv[1])==0) {
        ADC_Init(channel);   
    } else if(strcmp("collect",g_ppcArgv[1]) == 0) {
//        ADC_Collect(channel, Fs, buffer, numSamples);
    } else if(strcmp("read",g_ppcArgv[1]) == 0) {
        UART_OutUDec(ADC_In());
        //Display value to screen/UART?
    } 
}

/*******************CommandLCD********************
 Send debugging commands to the LCD
   Input: ui8Argc is the number of commands parsed
          g_ppcArgv is an array containing the commands
 Outputs: error (-1); success (0)
*************************************************/ 
void CommandLCD(uint_fast8_t ui8Argc, char *g_ppcArgv[]) {
    char message[80];
		uint16_t i; 
		int16_t screen = 0, line = 0;
	
    if (ui8Argc <= 3) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command lcd");
        return;
    }
    
    //Check for valid screen number
    if (strcmp(g_ppcArgv[1],"0")==0 || (strcmp(g_ppcArgv[1],"1")==0)) {
        screen = atoi(g_ppcArgv[1]);
    } else {
        OutCRLF();
        UART_OutString("Invalid parameter for screen number");
        return;
    }

    // Check for valid line number
        if (strcmp(g_ppcArgv[2],"0")==0 || (strcmp(g_ppcArgv[2],"1")==0) || strcmp(g_ppcArgv[2],"2")==0 || (strcmp(g_ppcArgv[2],"3")==0) || strcmp(g_ppcArgv[2],"4")==0 || (strcmp(g_ppcArgv[2],"5")==0) || strcmp(g_ppcArgv[2],"6")==0 || (strcmp(g_ppcArgv[2],"7")==0)) {
        line = atoi(g_ppcArgv[2]);
    } else {
        OutCRLF();
        UART_OutString("Invalid parameter for line number");
        return;
    }
    
    // Display message
    message[0] = '\0';
    for (i = 3; i < ui8Argc; i++) {
        sprintf(message, "%s %s", message, g_ppcArgv[i]);
    }
    ST7735_MessageString(screen, line, (unsigned char*)message);
    OutCRLF(); UART_OutString(message);
}

/*******************CommandOS*********************
 Send debugging commands to the OS
   Input: ui8Argc is the number of commands parsed
          g_ppcArgv is an array containing the commands
 Outputs: error (-1); success (0)
*************************************************/ 
void CommandOS(uint_fast8_t ui8Argc, char *g_ppcArgv[]) {
    if (ui8Argc < 2) {
        OutCRLF();
        UART_OutString("Insufficient arguments for command os");
        return;    
    }
    
    if (strcmp(g_ppcArgv[1],"clear") == 0) {
        OS_ClearMsTime();
    }
    else if (strcmp(g_ppcArgv[1],"read") == 0) {
        UART_OutString(" ");
				UART_OutUDec(OS_MsTime());
    }
		else if (strcmp("stop", g_ppcArgv[1]) == 0) {
//        OS_StopPeriodicThread();
    }
		else if (strcmp(g_ppcArgv[1],"start") == 0) {
//        OS_StartPeriodicThread();
    }
    else {
        UART_OutString("command os argument not recognized");
    }
}

void CommandEcho(uint_fast8_t ui8Argc, char *g_ppcArgv[]) {
	OutCRLF();
	UART_OutString("echo");
}

// Command Table as defined by Tivaware
tCmdLineEntry g_psCmdTable[] = {
    { "adc", CommandADC, HelpADC },
    { "lcd", CommandLCD, HelpLCD },
    { "os", CommandOS, HelpOS },
		{	"echo", CommandEcho, HelpEcho},
};
