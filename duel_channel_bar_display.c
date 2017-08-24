// ***********************************************************************
//
// Dual-channel LED bar graph display                    
// ***********************************************************************
//	 	   			 		  			 		  		
// Completed by: Vinay Nagarajan
//               
//
//
// 
//
// ***********************************************************************

#include <hidef.h>           /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>
#define TSIZE 100
// All funtions after main should be initialized here

// Note: inchar and outchar can be used for debugging purposes

char inchar(void);
void outchar(char x);
			 		  		
//  Variable declarations  	   			 		  			 		       
int tenthsec = 0;  // one-tenth second flag
int leftpb = 0;    // left pushbutton flag
int rghtpb = 0;    // right pushbutton flag
int runstp = 0;    // run/stop flag                         
int rticnt = 0;    // RTICNT (variable)
int prevpb = 0;    // previous state of pushbuttons (variable)
int prvlpb = 0; // previous left pushbutton state
int prvrpb = 0; // previous right pushbutton state	 	   		
int out0 = 0;
int out1 = 0;

int isf = 0;
int osf = 0;

int h = 0;
int go_up = 1;
int ct = 0;
int i = 0;
int tbuf[TSIZE]; 
int j = 0;
int count = 0; 
// Initializations
 
void  initializations(void) {

// Set the PLL speed (bus clock = 24 MHz)

  		CLKSEL = CLKSEL & 0x80; // disengage PLL from system
  		PLLCTL = PLLCTL | 0x40; // turn on PLL
  		SYNR = 0x02;            // set PLL multiplier
  		REFDV = 0;              // set PLL divider
  		while (!(CRGFLG & 0x08)){  }
  		CLKSEL = CLKSEL | 0x80; // engage PLL
  
// Disable watchdog timer (COPCTL register)

      COPCTL = 0x40;    //COP off - RTI and COP stopped in BDM-mode

// Initialize asynchronous serial port (SCI) for 9600 baud, no interrupts

      SCIBDH =  0x00; //set baud rate to 9600
      SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
      SCICR1 =  0x00; //$9C = 156
      SCICR2 =  0x0C; //initialize SCI for program-driven operation
         
//  Initialize Port AD pins 7 and 6 for use as digital inputs

	    DDRAD = 0; 		//program port AD for input mode
      ATDDIEN = 0xC0; //program PAD7 and PAD6 pins as digital inputs
         
//  Add additional port pin initializations here  (e.g., Other DDRs, Ports) 


//  Define bar graph segment thresholds (THRESH1..THRESH5)
//  NOTE: These are binary fractions
    #define THRESH1 0x2B  
    #define THRESH2 0x55
    #define THRESH3 0x80
    #define THRESH4 0xAA
    #define THRESH5 0xD5 

//  Add RTI/interrupt initializations here
     CRGINT = 0x80;//enable RTI interrupt
     RTICTL = 0x70;//set interrupt rate to 8.192ms
     ATDCTL2 = 0x80;
     ATDCTL3 = 0x10 ;
     ATDCTL4 = 0x85;
     DDRT = 0xFF;


  TIOS = 0x80;
  TSCR1 = 0x80;
  TSCR2 = 0x0C;
  TIE = 0x80;
  TC7 = 15000;
  
  // PWM
  MODRR = 0x01;
  PWME = 0x01;
  PWMPOL = 0x01;
  PWMPER0 = 0xFF;
  PWMDTY0 = 0x7F;
  PWMPRCLK = 0x00;
  PWMCLK = 0x00;
  
  // ATD
  ATDCTL2 = 0x80;
  ATDCTL3 = 0x08;
  ATDCTL4 = 0x85;

  
}



int wait2(){
  int i = 0;
  for (i = 0; i < 5000; i++){
  }
  
  return 0;
}
	 		  			 		  		
 void f1(){
     if (i ==0) {
   h = 0;
 } else if (i > 0 && i <10) {
   h = 51;
 } else if (i ==10 && i < 20) {
   h = 102;
 }else if (i == 20 && i <30) {
   h = 153;
 }else if (i ==30 && i <40) {
   h = 206;
 }else if (i == 40 && i <60) {
   h = 255;
 }else if (i == 60 && i <70) {
   h = 206;
 }else if (i == 70 && i <80) {
   h = 153;
 }else if (i == 80 && i <90) {
   h = 102;
 }else if (i ==90 && i <100) {
   h = 51;
 }else if (i == 100){
  h = 0;
 }
 
 
 
tbuf[i] = h; 


}


void f2(){

           if(i==0) h=255;
   else if(i<20){
    h-=5;
   } 
   else if(i>=20&&i<80){
    h=255/2;
   }
    else if (i>=80&&i<100){
    h-=5;
    } 
    else{
      i=0;
    }
       wait2();
    tbuf[i] = h;


}
// Main (non-terminating loop)
 
void main(void) {
	initializations(); 		  			 		  		
	EnableInterrupts;


  for(;;) {


// Main program loop (state machine)
// Start of main program-driven polling loop

	 	if (tenthsec == 1) {
    tenthsec = 0;
    if(runstp == 1) {
           if (leftpb == 1) {
    leftpb = 0;
    isf = (isf + 1) % 3;
    if (isf == 0) {
      TC7 = 15000;
      TSCR2 = 0x0C;
    } else if(isf == 1) {
      TC7 = 3000;
      TSCR2 = 0x0C;

    } else if(isf == 2) {
      TC7 = 1500;
      TSCR2 = 0x0C;
         PTT_PTT0 =0;
    PTT_PTT1 =1;  
    }
 
  }
  
  if(rghtpb == 1) {
    rghtpb = 0;
    osf = (osf + 1) % 2;
    if (osf == 0) {
      f1();
    } else if(osf == 1) {
      f2();
          PTT_PTT0 =1;
    PTT_PTT1 =0;
    } 
  }
      
   }   			 		  			 		  		
//  If the "tenth second" flag is set, then
//    - clear the "tenth second" flag
//    - if "run/stop" flag is set, then
//       - initiate ATD coversion sequence
//       - apply thresholds to converted values
//       - determine 5-bit bar graph bit settings for each input channel
//       - transmit 10-bit data to external shift register
//    - endif
//  Endif

	 	   			 		  			 		  		
//  If the left pushbutton ("stop BGD") flag is set, then:
//    - clear the left pushbutton flag
//    - clear the "run/stop" flag (and "freeze" BGD)
//    - turn on left LED/turn off right LED (on docking module)
//  Endif

    if(leftpb ==1 ){
    runstp =0;
    leftpb = 0;
    PTT_PTT0 =0;
    PTT_PTT1 =1;          
  }			 		  			 		  		

//  If the right pushbutton ("start BGD") flag is set, then
//    - clear the right pushbutton flag
//    - set the "run/stop" flag (enable BGD updates)
//    - turn off left LED/turn on right LED (on docking module)
//  Endif
	 	   			
	 	   			
 if(rghtpb == 1){
    runstp =1;
    rghtpb = 0;
    PTT_PTT0 =1;
    PTT_PTT1 =0;
   } 		  			 		  		

  } /* loop forever */
  
 } /* make sure that you never leave main */

}


// ***********************************************************************                       
// RTI interrupt service routine: rti_isr
//
//  Initialized for 5-10 ms (approx.) interrupt rate - note: you need to
//    add code above to do this
//
//  Samples state of pushbuttons (PAD7 = left, PAD6 = right)
//
//  If change in state from "high" to "low" detected, set pushbutton flag
//     leftpb (for PAD7 H -> L), rghtpb (for PAD6 H -> L)
//     Recall that pushbuttons are momentary contact closures to ground
//
//  Also, keeps track of when one-tenth of a second's worth of RTI interrupts
//     accumulate, and sets the "tenth second" flag         	   			 		  			 		  		
 
interrupt 7 void RTI_ISR( void)
{
 // set CRGFLG bit to clear RTI device flag
  	CRGFLG = CRGFLG | 0x80; 
//	  if(runstp) { // only bump rticnt if stopwatch is running
     rticnt = (rticnt+1) % 12;
    if(rticnt == 0) {tenthsec = 1;}
   
    if(PTAD_PTAD7 == 0) { // check left pushbutton
    if(prvlpb == 1) {
     leftpb = 1;
    }
   }
    prvlpb = PTAD_PTAD7;
    if(PTAD_PTAD6 == 0) { // check right pushbutton
    if(prvrpb == 1) {
      rghtpb = 1;
     }
    }
    prvrpb = PTAD_PTAD6;
  // }
}

 /*
***********************************************************************                       
  TIM interrupt service routine   
***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{
  int tmp;
  // clear TIM CH 7 interrupt flag 
  TFLG1 = TFLG1 | 0x80; 
  
  ATDCTL5 = 0x10;
  while(ATDSTAT1_CCF0 != 1){
    
  }
 
    PWMDTY0 = tbuf[i];
    i = (i+1)%TSIZE;

}

/*
***********************************************************************                       
  SCI interrupt service routine   
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
 


}
