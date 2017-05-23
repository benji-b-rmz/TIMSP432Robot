// PeriodMeasure.c
// Runs on MSP432
// Use Timer A0 in 16-bit capture mode to request interrupts on the
// rising edge of P7.3 (TA0CCP0), and measure period between pulses.
// Daniel Valvano
// July 27, 2015


//Modified for Wall following car
//Benjamin Ramirez, Alexander Barrick
// Embedded Systems Spring 2016

/* This example accompanies the book
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2015
   Example 7.2, Program 7.2

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

#include <stdint.h>
#include "msp.h"
#include "ClockSystem.h"
#include "InputCapture.h"
#include "PWM.h"
//#include "PWM.h"  //For the motors


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
//encoder globals
volatile uint16_t left_count = 0;
volatile uint16_t right_count = 0;
//pwm globals
volatile uint16_t left_duty = 10000;
volatile uint16_t right_duty = 10000;
uint16_t LEFT_TRIM = 0;
uint16_t RIGHT_TRIM = 1150;
int DoneL;
int DoneR;
int GOAL_DIST = 21000; //roughly 21 cm
int RIGHT_THRESH = 55000;
int KPL = 6;	//.25 factor		//factor for controller
int KPR = 1; // 
	
volatile uint16_t b_right_dist = 0;
volatile uint16_t f_right_dist = 0;
volatile uint16_t front_dist = 0;
volatile uint32_t dist_error;
int done_f_right;
int done_b_right;
int done_front;


uint8_t WALL = 1;
uint8_t LEFT = 2;
uint8_t RIGHT = 3;
uint8_t CHEAT = 4;
uint8_t STRAIGHT = 5;
volatile uint8_t STATE = 0;
                   

#ifdef __TI_COMPILER_VERSION__
  //Code Composer Studio Code
  void Delay(uint32_t ulCount){
  __asm (  "dloop:   subs    r0, #1\n"
      "    bne     dloop\n");
}
#else
  //Keil uVision Code
  __asm void
  Delay(uint32_t ulCount)
  {
    subs    r0, #1
    bne     Delay
    bx      lr
  }
#endif


//========================================SENSOR INITS=================================================
//*******************INitializing TA2 for FRONT facing sensor****************

void PWMeasureRight_Init(void){//long sr; 
	P5SEL0 |= 0xC0;
  P5SEL1 &= ~0xC0;                 // configure P5.6, P5.7 as TA2CCP0
  P5DIR &= ~0xC0;                  // make P5.6, P5.7 in	
  TA2CTL &= ~0x0030;               // halt Timer A2
  TA2CTL = 0x0200;	
	TA2EX0 &= ~0x0007; 	
	TA2CCTL1 = 0x8910;    //falling, capture, sync, arm
	TA2CCTL2 = 0x4900;		//rising, capture, sync, disarm
  NVIC_IPR3 = (NVIC_IPR3&0xFFFFFF00)|0x00004000; // priority 2 IPR2 bits 15-13
// interrupts enabled in the main program after all devices initialized
	NVIC_ISER0 |= 0x00002000; // enable interrupt 13 in NVIC
	TA2CTL |= 0x0024;        // reset and start Timer A2 in continuous up mode
	EnableInterrupts();
	//EndCritical(sr);
}

void TA2_N_IRQHandler(void){
  TA2CCTL1 &= ~0x0001;             // acknowledge capture/compare interrupt 0
  front_dist = TA2CCR1 - TA2CCR2;
	done_front = 1;
}
//**************INITIALIZING TA3 for RIGHT facing sensors*************************


void PWMeasureFront_Init(void){//long sr;
  // initialize P10.4 and make it input (P10.4 TA3CCP0)
	P10SEL0 |= 0x30;
  P10SEL1 &= ~0x30;                 // configure P10.4as TA3CCP0, 10.5 TA3CCP0
  P10DIR &= ~0x30;                  // make P10.4 in, 10.5
	
	P8SEL0 |= 0x04;
  P8SEL1 &= ~0x04;                 // configure P8.2 as TA3CCP1
  P8DIR &= ~0x04;                  // make P8.2 in
	
	P9SEL0 |= 0x04;
  P9SEL1 &= ~0x04;                 // configure P9.2 as TA3CCP3
  P9DIR &= ~0x04;                  // make P9.2 in
	
	
  TA3CTL &= ~0x0030;               // halt Timer A0
  TA3CTL = 0x0200;	
	TA3EX0 &= ~0x0007; 	
	//for the front right sensor on TA3.0, TA3.1
	TA3CCTL0 = 0x8910;    //falling, capture, sync, arm
	TA3CCTL1 = 0x4900;		//rising, capture, sync, disarm
	//for the back right sensor on TA3.2, TA3.3
	TA3CCTL2 = 0x8910;    //falling, capture, sync, arm
	TA3CCTL3 = 0x4900;		//rising, capture, sync, disarm
	
  NVIC_IPR3 = (NVIC_IPR3&0xFFFFFF00)|0x40400000; // priority 2 IPR3 bits 23-21
	//also set priority 2 for bits 29-31 for the back sensor
// interrupts enabled in the main program after all devices initialized
	NVIC_ISER0 |= 0x0000C000; // enable interrupt 14 and 15 in NVIC
	TA3CTL |= 0x0024;        // reset and start Timer A3 in continuous up mode
	EnableInterrupts();
	//EndCritical(sr);
}

void TA3_0_IRQHandler(void){
  TA3CCTL0 &= ~0x0001;             // acknowledge capture/compare interrupt 0
  f_right_dist = TA3CCR0 - TA3CCR1;
	done_f_right = 1;
}

void TA3_N_IRQHandler(void){
  TA3CCTL2 &= ~0x0001;             // acknowledge capture/compare interrupt 2
  b_right_dist = TA3CCR2 - TA3CCR3;
	done_b_right = 1;
}

//====================================END OF SENSOR INITS==================================



//*************************OTHER STUFF****************************
// max period is (2^19-1)*83.3 ns = 43.4612 ms
// min period determined by time to run ISR, which is about 1 us

//====================================ENCODER CODE======================================

void Encoder_LED_Init(void){
// initialize P2.2-P2.0 and make them outputs (P2.2-P2.0 built-in RGB LEDs)
  P2SEL0 &= ~0x07;
  P2SEL1 &= ~0x07;                 // configure built-in RGB LEDs as GPIO
  P2DS |= 0x07;                    // make built-in RGB LEDs high drive strength
  P2DIR |= 0x07;                   // make built-in RGB LEDs out
  P2OUT &= ~0x07;                  // RGB = off
}
void Left_Encoder(uint32_t time){
	
	P2OUT= P2OUT^0x02;					//toggle the green light
  DoneL = 1;
	left_count = left_count +1; 
}

void Right_Encoder(uint32_t time){
	//Toggle blue light
	P2OUT= P2OUT^0x04;
  DoneR = 1;
	right_count = right_count +1;
}



//====================================MOTOR CODE======================================

void Direction_Init(){ //initializes the direciotn bits of motor forward
	//motor direction bits
	P4DIR |= 0x0F;          // P4.4(IN C), P4.5(IN D) output
	P4SEL0 &= ~0x0F;         // P4.4, P4.5 direciton bits
  P4SEL1 &= ~0x0F;        // P4.4, P4.5 
	P4OUT |= 0x05;				//initially P4.0 INA and P4.2INC
	P4OUT &= ~0x0A;					//turning P4.1 and P4.3 OFF
}

void Stop_Motors(void){
	PWM_Left(0);
	PWM_Right(0);
}

void Left_Turn(void){
	PWM_Left(0);
	PWM_Right(6000);
}

//====================================RANGEFINDER CODE======================================

void pulseInit(void){//setting up port 3.0(front) and 3.6(front-right)+3.5(back right) for trigger pulses
	P3SEL0 &= ~0x61;
  P3SEL1 &= ~0x61;                 
  P3DS |= 0x61;                    
  P3DIR |= 0x61;                   
  P3OUT = 0x00;                  
	
	//testing PW
	P1SEL0 &= ~0x01;
  P1SEL1 &= ~0x01;                
  //P2DS |= 0x01;                    
  P1DIR |= 0x01;                  
  P1OUT = 0x00;                  
}
void pulseRight(void){
	//pulsing the front right sensor
	P3OUT |= 0x40;
	Delay(59990); //~10000 us low
	P3OUT = 0x00;
	Delay(60); //10 us high
	
	//pulsing the back right sensor
	P3OUT |= 0x20;
	Delay(59990); //~10000 us low
	P3OUT = 0x00;
	Delay(60); //10 us high
}
void pulseFront(void){
	P3OUT |= 0x01;
	Delay(59990); //~10000 us low
	P3OUT = 0x00;
	Delay(60); //10 us high	
}

int absolute(uint16_t first, uint16_t second){
	int value = 0;
	if(first >= second){
		value = first - second;
	}else{
		value = second - first;
	}
	return value;
}


int main(void){
	
	Clock_Init48MHz();               // 48 MHz clock; 12 MHz Timer A clock
	pulseInit();
	Encoder_LED_Init();
	Direction_Init();	
  PWM_Init(15000,10000,10000); //while(1){};

  DoneL = 0;                        // set on subsequent
	DoneR = 0;
  
	STATE = CHEAT;
  EncoderCapture_Init(&Left_Encoder, &Right_Encoder); //initialize Timer A1 in capture mode 8.0
  PWMeasureFront_Init();
	PWMeasureRight_Init();
	pulseFront();
	pulseRight();
	while(1){	
		//pulseFront();
		pulseRight();
		right_count = 0;
		left_count = 0;

		left_duty = 9500 + LEFT_TRIM;
		right_duty = 9500 + RIGHT_TRIM;
		if (STATE == WALL){
			
			if(f_right_dist > RIGHT_THRESH)
			{
				STATE = STRAIGHT;
				P1OUT = 0x00;
			}
			else
			{
				P1OUT = 0x01;
				dist_error = absolute(f_right_dist-300, b_right_dist); 
		
				if (dist_error > 3500){
					dist_error = 3500;
				}
				if (b_right_dist >= (f_right_dist)){
					right_duty = right_duty + dist_error/KPR + dist_error/3;
				}else{
					left_duty = left_duty + dist_error/KPL;
				}
				PWM_Left(left_duty);
				PWM_Right(right_duty);
			
			}
			
		}
		else if(STATE == CHEAT)
		{
			right_count = 0;
			if(f_right_dist > RIGHT_THRESH)
			{
				STATE = RIGHT;
				P1OUT = 0x00;
			}
			else
			{
				P1OUT = 0x00;
				dist_error = absolute(f_right_dist, b_right_dist); 
		
				if (dist_error > 3500){
					dist_error = 3500;
				}
				if (b_right_dist >= (f_right_dist)){
					right_duty = right_duty + dist_error/KPR +100;
				}else{
					left_duty = left_duty + dist_error/KPL;
				}
				PWM_Left(left_duty);
				PWM_Right(right_duty);
			
			}
		}
		else if(STATE == RIGHT)
		{
			left_count = 0;
			while(left_count <25)
			{
				PWM_Left(left_duty);
				PWM_Right(right_duty+1700);
			}
			left_count = 0;
			while(left_count <17){//turn 
				PWM_Left(left_duty);
				PWM_Right(1000);
			}
			
			STATE = STRAIGHT; 
		}
		else if (STATE == STRAIGHT)
		{
			left_count = 0;
			while(left_count <60)
			{
				PWM_Left(left_duty);
				PWM_Right(right_duty+1660);
			}
			if(b_right_dist < RIGHT_THRESH){
				STATE = WALL;
			}
		}
  }
}

