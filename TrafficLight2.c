// TrafficLight.c
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.
// Your Name: Moses Mccabe
// created: 
// last modified by Hadil Mustafa: 9/27/15
// 

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2 Program 3.1, Example 3.1

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


#include "PLL.h"
#include "SysTick.h"

// symbolic names instead of addresses
// Use PORTE for lights
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_LOCK_R       (*((volatile unsigned long *)0x40024520))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
	
// Use PORTB for LED
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
	
// sensors and lights
//#define SENSORS                 (*((volatile unsigned long *)0x4002400C))   // for port E
//#define LIGHT                   (*((volatile unsigned long *)0x40005438))  // for port B
	

// add any needed port definitions 
void PortE_Init(void);
void PortB_Init(void);
void States_Init(void);


// direction names
volatile unsigned int sensors;


// Linked data structure
struct Traffic
{
	unsigned int output;
	unsigned int time;
	struct Traffic *Next[16];
};
// define your structure here
typedef struct Traffic state;


//define your states here e.g. #define stateName 0, etc.
#define GoIvy &controls[0]
#define WalkWest &controls[1]
#define WaitIvy &controls[2]
#define GoWest &controls[3]
#define WalkIvy &controls[4]
#define  WaitWest &controls[5]


//Declare your states here 
state controls[6] = {
                      {0x28,7,{GoIvy,WalkWest,WaitIvy,WalkWest,WaitIvy,WaitIvy,WaitIvy,WaitIvy,GoIvy,WalkWest,WaitIvy,WalkWest,WaitIvy,WaitIvy,WaitIvy,WaitIvy}},
                      {0x29,4,{GoIvy,WalkWest,WaitIvy,WaitIvy,WaitIvy,WaitIvy,WaitIvy,WaitIvy,GoIvy,WalkWest,WaitIvy,WaitIvy,WaitIvy,WaitIvy,WaitIvy,WaitIvy}},
                      {0x48,3,{GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest,GoWest}},
                      {0x82,7,{WaitWest,WaitWest,WalkIvy,WalkIvy,GoWest,WaitWest,WalkIvy,WalkIvy,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest}},
                      {0x92,4,{WaitWest,WaitWest,WalkIvy,WaitWest,GoWest,WaitWest,WalkIvy,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest,WaitWest}},
                      {0x84,3,{GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy,GoIvy}},
                    };
	

int main(void)
{
  state *curr;         // pointer of object Traffice
	
  PLL_Init();           // 80 MHz, Program 10.1
  SysTick_Init();       // Initialize SysTick <-- function came from "SysTick.h" which allow us to use it functions

    
	// initialize ports
    PortE_Init();
    PortB_Init();
	
	
	curr = &controls[0];                 // set pointer to first node in lists
	GPIO_PORTB_DATA_R = curr->output;    // use current pointer to access node one of the lists and pass it output to PORTB_DATA register
    while(1){
        SysTick_Wait10ms(curr->time);        // use curr pointer to access the current node of the lists and pass it time value to systick to create a delay
        sensors = GPIO_PORTE_DATA_R & 0x0F;  // mask out PE0 - PE4 and store the result in sensors
        curr = curr->Next[sensors];          // advance curr point to the next state
        GPIO_PORTB_DATA_R = curr->output;    // update PORTB Data Register
   }
}


// set up PORTE for Switches (inputs=0)
void PortE_Init(void){
		SYSCTL_RCGC2_R  |= 0x10;                  // Turn on PORTE clock
		unsigned int delay = SYSCTL_RCGC2_R;      // allow clock to turn on
		GPIO_PORTE_DIR_R &= ~0x0F;                 /* The direction register specifies bit for bit whether the
                                                       corresponding pins are input or output 1-output 0-input  (PE0 - PE4 are input). */
		GPIO_PORTE_DEN_R |= 0x0F;      //  enable the corresponding I/O pins by writing ones
		GPIO_PORTE_AMSEL_R  &= ~0xFF;  //  disable the analog funct PF7-0
		GPIO_PORTE_AFSEL_R &= ~0xFF;   // disable alt funct on PF7-0	
}


//set up PORTB for lights (outputs=1)
void PortB_Init(void)
{
	  SYSCTL_RCGC2_R  |= 0x02;                      // Turn on PORTE clock
		unsigned int delay = SYSCTL_RCGC2_R;        // allow clock to turn on
		GPIO_PORTE_DIR_R |= 0xFF;                   /* The direction register specifies bit for bit whether the corresponding pins are
                                                       input or output 1-output 0-input (PB0 - P7 are outputs */
		GPIO_PORTE_DEN_R |= 0xFF;      //  enable the corresponding I/O pins by writing ones
		GPIO_PORTE_AMSEL_R  &= ~0xFF;  //  disable the analog funct PF7-0
		GPIO_PORTE_AFSEL_R &= ~0xFF;   // disable alt funct on PF7-0	
}
