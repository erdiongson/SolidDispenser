/*
 * File:   MotorDriver.c
 * Author: Eric Liu
 *
 * Created on May 18, 2020, 10:54 AM
 */


#include <xc.h>
#include "IO.h"
#include "main.h"

void initMotor(void);
void Set_RG3(void);
void Clr_RG3(void);
void MotorON(void);
void MotorBREAK(void);

unsigned char PWM_reg;

void initMotor(void)
{
    PR4 = 0xC7;
    T3CONbits.T3CCP1 = 0;   // to set Timer3 and Timer4 are the clock sources for ECCP2, ECCP3, CCP4 and CCP5;
    T3CONbits.T3CCP2 = 1;    
    T4CON = 0x00;
}

void Set_RG3(void)
{	
    /*CCP4CON = 0x0C;  //0b00001100;	// setup as PWM 
    CCPR4L = PWM_reg;*/
    CCP4CON = 0x08;  //0b00001000;	// set pin high 
 	T4CONbits.TMR4ON = 1;
}

void Clr_RG3(void)
{
	CCP4CON = 0x00;		//Disable PWM
    T4CONbits.TMR4ON = 0;
}

void MotorON(void)
{
	MotorDrive_RG3 = 1;
	MotorDrive_RG4 = 1;

	Set_RG3();
}

void MotorBREAK(void)
{

	Clr_RG3();

	MotorDrive_RG3 = 1;
	MotorDrive_RG4 = 1;
}
