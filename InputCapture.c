// InputCapture.c
// Runs on MSP432
// Use Timer A0 in capture mode to request interrupts on the rising
// edge of P7.3 (TA0CCP0), and call a user function.
// Daniel Valvano
// July 27, 2015

/* This example accompanies the book
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2015
   Example 6.1, Program 6.1

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

// external signal connected to P7.3 (TA0CCP0) (trigger on rising edge)

//=============================INITIALIZATION FUNCTION FOR ENCODER FUNCTIONS===================
#include <stdint.h>
#include "msp.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*LeftTask)(uint32_t time);// user function
void (*RightTask)(uint32_t time);// user function


void EncoderCapture_Init(void(*task1)(uint32_t time), void(*task2)(uint32_t time)){long sr;
  sr = StartCritical();
  LeftTask = task1;              // user function
	RightTask = task2;
  // initialize P8.0 (ta1.0) and P7.7 (ta1.1)
  P8SEL0 |= 0x01;
  P8SEL1 &= ~0x01;                 // configure P8.0 as TA0CCP0
  P8DIR &= ~0x01;                  // make P8.0 in
	
	P7SEL0 |= 0x80;
  P7SEL1 &= ~0x80;                 // configure P8.0 as TA0CCP0
  P7DIR &= ~0x80;                  // make P8.0 in
	
  TA1CTL &= ~0x0030;               // halt Timer A0
  TA1CTL = 0x0200;
  TA1CCTL0 = 0x4910;							//for TA1.0
	TA1CCTL1 = 0x4910;							//TA1.1

  TA1EX0 &= ~0x0007;       // configure for input clock divider /1
  NVIC_IPR2 = (NVIC_IPR2&0xFFFFFF00)|0x40400000; // priority 2 IPR2 bits 23-21 and priority 2 on IPR2 bits 29-30
// interrupts enabled in the main program after all devices initialized
  NVIC_ISER0 |= 0x00000C00; // enable interrupt 10 and 11 in NVIC
  TA1CTL |= 0x0024;        // reset and start Timer A0 in continuous up mode

  EndCritical(sr);
}

void TA1_0_IRQHandler(void){
  TA1CCTL0 &= ~0x0001;             // acknowledge capture/compare interrupt 0
  (*LeftTask)(TA1CCR0);         // execute user task
}
void TA1_N_IRQHandler(void){
  TA1CCTL1 &= ~0x0001;             // acknowledge capture/compare interrupt 0
  (*RightTask)(TA1CCR1);         // execute user task
}

