
#include "pwm.h"

static void CLK_Initialize(void){
  // Set the oscillator to the internal oscillator with a clock frequency of 8MHz
  OSCCONbits.IRCF0 = 1;
  OSCCONbits.IRCF1 = 1;
  OSCCONbits.IRCF2 = 1;
  OSCCONbits.SCS0 = 1;
  OSCCONbits.SCS1 = 1;
}

static void PWM_Initialize(unsigned char period)
{
  // Set the PWM output pin as an output by writing a '0' to the corresponding TRIS register bit
  TRISCbits.TRISC1 = 0; //RC1 Pin as Output
  //Set Timer2 post-scaler 1:1
  T2CONbits.T2OUTPS0 = 0;
  T2CONbits.T2OUTPS1 = 0;
  T2CONbits.T2OUTPS2 = 0;
  T2CONbits.T2OUTPS3 = 0;
  //Set pre-scaler 1:1
  T2CONbits.T2CKPS = 0x00;

  //CCPxCON Setting
  CCP2CONbits.DC2B = 0;
  CCP2CONbits.CCP2M = 0x0C;
  CCPR2L = 0;
  
  //set the PWM period by writing to PR2 Register
  //AS per computation:
  //PWD_Period = (PR2+1)*(4*TOSC*Prescaler)
  //PR2 = [(1/400KHZ)/(4*(1/8MHz)*1)]-1
  PR2 = period;

    
  T2CONbits.TMR2ON = 1;
}