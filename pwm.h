/* 
 * File:   pwm.h
 * Author: ediongson
 *
 * Created on December 9, 2022, 11:09 AM
 */

#ifndef PWM_H
#define	PWM_H



#ifdef	__cplusplus
extern "C" {
#endif
#include "pic18f65j50.h"
    
static void PWM_Initialize(unsigned char period);
static void CLK_Initialize(void); //Oscillator Initialization

#ifdef	__cplusplus
}
#endif

#endif	/* PWM_H */

