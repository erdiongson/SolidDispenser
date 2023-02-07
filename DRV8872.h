/* 
 * File:   DRV8872.h
 * Author: victorlu
 *
 * Created on November 29, 2019, 11:30 AM
 */

#ifndef DRV8872_H
#define	DRV8872_H

#ifdef	__cplusplus
extern "C" {
#endif


void Motor_Init(void);
void MotorON_PWM(void);

void HomeBREAK(void);
void MotorBREAK(void);

void Set_RG3_PWM(void);
void Set_RG4_PWM(void);

void Clr_RG3_PWM(void);
void Clr_RG4_PWM(void);


#ifdef	__cplusplus
}
#endif

#endif	/* DRV8872_H */

