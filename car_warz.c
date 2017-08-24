/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Fall 2016
***********************************************************************
	 	   			 		  			 		  		
 Team ID: 38
 Project Name: Car  Warz
 Team Members:
   - Team/Doc Leader: All 3 of us            Signature: ______________________
   
   - Software Leader: Vinay Nagarajan       Signature: ______________________
   - Interface Leader: Alek Patel           Signature: ______________________
   - Peripheral Leader: Ribhav Agarwal      Signature: ______________________
 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.
***********************************************************************
 The objective of this Mini-Project is to create a simultaneously autonomous
 and radio-frequency controlled land drone by utilizing various peripherals of
 the 9S12 family Freescale microcontrollers (specifically, the PWM, SCI, ATD, and 
 TIM) 
***********************************************************************
 List of project-specific success criteria (functionality that will be
 demonstrated):
 1. Remote-controlled wireless communication
 2. Autonomous obstacle detection
 3. Synchronized motor and controls 
 4. Implementation of hardware sensors
***********************************************************************
  Date code started: 11/20/2016
  Update history (add an entry every time a significant change is made):
  Date: 11/25  Name: Vinay   Update: Autonomous drive
  Date: 11/30  Name: Alek   Update: Initializations
    
  Date: 12/3   Name: Vinay   Update: SCI routine
  Date: 12/5   Name: Ribhav   Update: Finalized design
***********************************************************************
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All functions after main should be initialized here */
char inchar(void);
void outchar(char x);
void tdisp();
void shiftout(char x);
void lcdwait(void);
void send_byte(char x);
void send_i(char x);
void chgline(char x);
void print_c(char x);
void pmsglcd(char[]);

/* Variable declarations */  	   			 		  			 		       

/* ASCII character definitions */
#define CR 0x0D	// ASCII return character   

/* LCD COMMUNICATION BIT MASKS */
#define RS 0x04		// RS pin mask (PTT[2])
#define RW 0x08		// R/W pin mask (PTT[3])
#define LCDCLK 0x10	// LCD EN/CLK pin mask (PTT[4])

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F	// LCD initialization command
#define LCDCLR 0x01	// LCD clear display command
#define TWOLINE 0x38	// LCD 2-line enable command
#define CURMOV 0xFE	// LCD cursor move instruction
#define LINE1  0x80	// LCD line 1 cursor position
#define LINE2  0xC0	// LCD line 2 cursor position

/* LED BIT MASKS */
#define GREEN 0x20
#define RED 0x40
#define YELLOW 0x80


int temp = 0;
long react = 0;
long react2 = 0;	 	   		
/*	 	   		
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40   ; //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, interrupts off initially */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)    //0x38
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port

/* Initialize peripherals */
  ATDDIEN = 0x00;
  ATDCTL2 = 0xC0;
  ATDCTL3 = 0x10;
  ATDCTL4 = 0x85;           

/*
  Initialize the RTI for an 8.192 ms interrupt rate
*/
  
   
/* PWM initializations */
  MODRR = 0x0F;    //PT3,2,1,0 used as PWM Ch 3,2,1,0 output
  PWME = 0x0F;    //enable PWM Ch 0,1,2,3
  PWMPOL = 0x01;  //negative polarity
//  PWMCTL	= 0x00;  // no concatenate (8-bit)
//  PWMCAE	= 0x00;  // left-aligned output mode
  PWMPER3 = 0xFF;	// set maximum 8-bit period                      //24,000,000/32/30/255 = 98 Hz
  PWMDTY3 = 0x7F;  // initially clear DUTY register
  PWMPER2 = 0xFF;	// set maximum 8-bit period                      //24,000,000/32/30/255 = 98 Hz
  PWMDTY2 = 0x7F;  // initially clear DUTY register
  PWMPER1 = 0xFF;	// set maximum 8-bit period                      //24,000,000/32/30/255 = 98 Hz
  PWMDTY1 = 0x7F;  // initially clear DUTY register
  PWMPER0 = 0xFF;	// set maximum 8-bit period                      //24,000,000/32/30/255 = 98 Hz
  PWMDTY0 = 0x7F;  // initially clear DUTY register
  PWMCLK	= 0x0F;  // select scaled clock 
//  PWMSCLB = 15;     //scale B register
//  PWMSCLA = 15;     //scale B register
  PWMPRCLK	= 0x00; //A = B = bus clock / 32 	      	      



/* Initialize asynchronous serial port (SCI) for 9600 baud, no interrupts */
 
         
         
/* Add additional port pin initializations here */
  DDRT = 0xFF;
  DDRM = 0x30;

/* Initialize SPI for baud rate of 6 Mbs */
  SPICR1 = 0x50;
  SPICR2 = 0x0;
  SPIBR = 0x01;

/* Initialize digital I/O port pins */
  DDRAD = 0x0;
  ATDDIEN = 0xC0;


  

/* Initialize the LCD
     - pull LCDCLK high (idle)
     - pull R/W' low (write state)
     - turn on LCD (LCDON instruction)
     - enable two-line mode (TWOLINE instruction)
     - clear LCD (LCDCLR instruction)
     - wait for 2ms so that the LCD can wake up     
*/ 
  PTT_PTT6 = 1;
  PTT_PTT5 = 0;
  send_i(LCDON);
  send_i(TWOLINE);
  send_i(LCDCLR);
  lcdwait();
  
  
  
    
/* Initialize RTI for 2.048 ms interrupt rate */	
  CRGINT = 0x80;//enable RIT interrupt
  RTICTL = 0x1F;//set interrupt rate

/* Initialize TIM Ch 7 (TC7) for periodic interrupts every 1.000 ms
     - enable timer subsystem
     - set channel 7 for output compare
     - set appropriate pre-scale factor and enable counter reset after OC7
     - set up channel 7 to generate 1 ms interrupt rate
     - initially disable TIM Ch 7 interrupts      
*/
  TSCR1_TEN = 1;
  TIOS = 0x80;
  TSCR2 = 0x0C; //pre-scale factor is 16
  TC7 = 1500;
  TIE_C7I = 0; 
  
   


}
	 		  			 		  		
/*	 		  			 		  		
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  DisableInterrupts
	initializations(); 		  			 		  		
	EnableInterrupts;



 for(;;) {
 

     
     
    temp = inchar();
    if(temp == 'O'){
    
    //    PTT_PTT6 = 0;
    // PTT_PTT5 = 0;
      PWMDTY0 = 0;
      PWMDTY1 = 0;
      PWMDTY3 = 0;
      PWMDTY2 = 255;
     send_i(LCDCLR);
  chgline(LINE1);
   TIE_C7I = 0;
   react = react2;

   pmsglcd("Moving Forward !");
   chgline(LINE2);
     pmsglcd("RT = "); 
         print_c((react/10000)%10 + 48);

    print_c((react/1000)%10 + 48);
    print_c((react/100)%10 + 48);
    print_c((react/10)%10 + 48);
    print_c((react%10) + 48); 
    pmsglcd(" ms");  
    
    TCNT = 0; 
        react2 = 0;
        TIE_C7I = 1;
 
      
    } else if(temp == 'M' )  //REVERSE
    {
    //PTT_PTT6 = 0;
    // PTT_PTT5 = 0;
      PWMDTY0 = 255;
      PWMDTY1 = 255;
      PWMDTY3 = 255;
      PWMDTY2 = 0;
       TIE_C7I = 0;
   react = react2;

      send_i(LCDCLR);
  chgline(LINE1);
   pmsglcd("Moving Reverse !");
     chgline(LINE2);
          pmsglcd("RT = "); 
 
              print_c((react/10000)%10 + 48);
         print_c((react/1000)%10 + 48);
      print_c((react/100)%10 + 48);

    print_c((react/10)%10 + 48);
    print_c((react%10) + 48); 
    pmsglcd(" ms");    
 
   lcdwait();
   TCNT = 0; 
        react2 = 0;
        TIE_C7I = 1;
      
    } 
    
   else if(temp >= 17 && temp <=20 )  //RIGHT 
    {
    //PTT_PTT5 = 0;
      PWMDTY0 = 0;
      PWMDTY2 = 255;
      PWMDTY1 = 0;
      PWMDTY3 = 255; 
                      TIE_C7I = 0;
        send_i(LCDCLR);
  chgline(LINE1);
   pmsglcd("Turning Right !");
     chgline(LINE2);
          pmsglcd("RT = "); 
 
   react = react2;
                           print_c((react/10000)%10 + 48);
                          print_c((react/1000)%10 + 48);

    print_c((react/100)%10 + 48);
    print_c((react/10)%10 + 48);
    print_c((react%10) + 48); 
    pmsglcd(" ms"); 
    lcdwait();   
    TCNT = 0; 
        react2 = 0;
        TIE_C7I = 1;
    }  
    
    
    else if(temp == 39)  //turn LEFT 
    {
     //PTT_PTT6 = 0;
       PWMDTY0 = 0;
      PWMDTY1 = 255;
      PWMDTY3 = 0;
      PWMDTY2 = 255; 
     // PTT_PTT5 = 1;
      TIE_C7I = 0;
           send_i(LCDCLR);
  chgline(LINE1);
   pmsglcd("Turning Left !");
     chgline(LINE2); 
          pmsglcd("RT = "); 

        react = react2;
                          print_c((react/10000)%10 + 48);
         print_c((react/1000)%10 + 48);

    print_c((react/100)%10 + 48);
    print_c((react/10)%10 + 48);
    print_c((react%10) + 48); 
    pmsglcd(" ms");    
 
lcdwait();
 TCNT = 0; 
        react2 = 0;
        TIE_C7I = 1;
    } 
    
    else if(temp == 38){
      PTT_PTT7 = 1;  
            
    }
    
    else{
    
   
       PTT_PTT7 = 0;  
      
    // PTT_PTT6 = 0;
    // PTT_PTT5 = 0;
       PWMDTY0 = 255;
      PWMDTY1 = 0;
      PWMDTY3 = 0;
      PWMDTY2 = 0;
       TIE_C7I = 0;   
        send_i(LCDCLR);
  chgline(LINE1);
   pmsglcd("Stop !");
     chgline(LINE2); 
          pmsglcd("RT = "); 

        react = react2;
                             print_c((react/10000)%10 + 48);
         print_c((react/1000)%10 + 48);

    print_c((react/100)%10 + 48);
    print_c((react/10)%10 + 48);
    print_c((react%10) + 48); 
    pmsglcd(" ms");    
 
lcdwait();
 TCNT = 0; 
 
        react2 = 0;
        TIE_C7I = 1;
    }
      
 
  
   } /* loop forever */
   
}   /* do not leave main */


/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR
************************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flagt 
  	CRGFLG = CRGFLG | 0x80; 
}

/*
***********************************************************************                       
  TIM interrupt service routine	  		
***********************************************************************
*/
interrupt 15 void TIM_ISR(void)
{
	// clear TIM CH 7 interrupt flag
 	TFLG1 = TFLG1 | 0x80;
 	
 	 
 	react2 = react2 + 1; 
 	

}

/*
***********************************************************************                       
  SCI interrupt service routine		 		  		
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
 
}

/*
***********************************************************************
  shiftout: Transmits the character x to external shift 
            register using the SPI.  It should shift MSB first.  
             
            MISO = PM[4]
            SCK  = PM[5]
***********************************************************************
*/
 
void shiftout(char x)

{
 
  // read the SPTEF bit, continue if bit is 1
  // write data to SPI data register
  // wait for 30 cycles for SPI data to shift out 
  int i;
  while (SPISR_SPTEF == 0) {
  }
  SPIDR = x;
  for (i=0;i<30;i++) {
  //do nothing just wait
    
  }
}

/*
***********************************************************************
  lcdwait: Delay for approx 2 ms
***********************************************************************
*/

void lcdwait()
{
  int i;
  for(i=0;i<5000;i++) {
  }
  //_delay_ms(2);
}

/*
*********************************************************************** 
  send_byte: writes character x to the LCD
***********************************************************************
*/

void send_byte(char x)
{
     // shift out character
     // pulse LCD clock line low->high->low
     // wait 2 ms for LCD to process data
  shiftout(x);
  PTT_PTT6 = 0;
  PTT_PTT6 = 1;
  PTT_PTT6 = 0;
  lcdwait();

}

/*
***********************************************************************
  send_i: Sends instruction byte x to LCD  
***********************************************************************
*/

void send_i(char x)
{
        // set the register select line low (instruction data)
        // send byte
  PTT_PTT4 = 0;
  send_byte(x);
}

/*
***********************************************************************
  chgline: Move LCD cursor to position x
  NOTE: Cursor positions are encoded in the LINE1/LINE2 variables
***********************************************************************
*/

void chgline(char x)
{
  send_i(CURMOV);
  send_i(x);  
}

/*
***********************************************************************
  print_c: Print (single) character x on LCD            
***********************************************************************
*/
 
void print_c(char x)
{
  PTT_PTT4 = 1;
  send_byte(x);  
}

/*
***********************************************************************
  pmsglcd: print character string str[] on LCD
***********************************************************************
*/

void pmsglcd(char str[])
{
  int i = 0;
  while (str[i] != 0) {
    print_c(str[i]);
    i++;
  }
}

/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 (for debugging only)
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
 Example:      char ch1 = inchar();
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}

/*
***********************************************************************
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}
