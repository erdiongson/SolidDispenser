#include "IO.h"
#include "DRV8872.h"


void Set_RG3_PWM(void)
{
	CCP4CON |= 0x0C;  //0b00001100;	// setup as PWM
    CCP4CON &= 0xCF;
    CCPR4L = PWM_reg;

    T3CONbits.T3CCP1 = 0;   // timer4 drives PWM
    T3CONbits.T3CCP2 = 1;   // timer4 drives PWM
    PR4 = 0xC7;
    
    T4CON = 0x00;
 	T4CONbits.TMR4ON = 1;
}

void Clr_RG3_PWM(void)
{
	T4CONbits.TMR4ON = 0;
	CCP4CON = 0x00;		//Disable PWM
    MotorDrive_RG3 = 0; 
}

void MotorON_PWM(void)
{
	MotorDrive_RG3 = 1;
	MotorDrive_RG4 = 1;

	Set_RG3_PWM();
}

void MotorBREAK(void)
{

	Clr_RG3_PWM();

	MotorDrive_RG3 = 1;
	MotorDrive_RG4 = 1;
}

