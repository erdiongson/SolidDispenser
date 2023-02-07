/* 
 * File:   IO.h
 * Author: victorlu
 *
 * Created on November 29, 2019, 10:28 AM
 */

#ifndef IO_H
#define	IO_H

#ifdef	__cplusplus
extern "C" {
#endif

//-------------- I/O setting ---------------------
// 1 = in 0 = out
//#define 		 	PORTAbits.RA0		//AN0 - Battery supply 
#define IR_Status 		PORTAbits.RA1		//AN1 - IR sensor 
#define IR_ON	 		LATAbits.LATA2
#define UP		     	PORTAbits.RA3
#define DOWN			PORTAbits.RA4
#define LEFT			PORTAbits.RA5
//#define 			PORTAbits.RA6		//OSC1
//#define 			PORTAbits.RA7		//OSC2
#define PORTA_TRIS      0xFB     //11111011    

#define SENSOR_A		PORTBbits.RB0
#define SENSOR_B		PORTBbits.RB1
#define MOTOR_ON_BUT   	PORTBbits.RB2
#define RIGHT			PORTBbits.RB3
#define CENTER	    	PORTBbits.RB4
//#define			LATB5
//#define PGC		LATB6
//#define PGD		LATB7
#define PORTB_TRIS      0XFF      // 0&2in

//#define 			PORTCbits.RC0	
#define VIB_MOTOR_ON	LATCbits.LATC1	
//#define 			PORTCbits.RC2	
#define SCL				LATCbits.LATC3
#define SEN				LATCbits.LATC4
#define SDATA			LATCbits.LATC5
//#define 			PORTCbits.RC6		//TXT
//#define 			PORTCbits.RC7		//RX
#define PORTC_TRIS      0X84     // all out      

#define GREEN_LED		LATDbits.LATD0
#define AMBER_LED		LATDbits.LATD1
//#define 			PORTDbits.RD2
//#define 			PORTDbits.RD3
//#define 			PORTDbits.RD4
#define SDA2         	PORTDbits.RD5
#define SCL2         	PORTDbits.RD6
//#define          	PORTDbits.RD7
#define PORTD_TRIS      0XC0      //      

//#define 			PORTEbits.RE0
//#define 			PORTEbits.RE1
//#define 			PORTEbits.RE2
//#define 			PORTEbits.RE3
//#define 			PORTEbits.RE4
#define TP6				LATEbits.LATE5
#define TP7				LATEbits.LATE6
//#define 			PORTEbits.RE7
#define PORTE_TRIS      0X00      //

//#define 			PORTFbits.RF2
//#define 			PORTFbits.RF3	//Data-
//#define 			PORTFbits.RF4	//Data+
#define TP5				LATFbits.LATF5
#define TP8				LATFbits.LATF6
//#define 			PORTFbits.RF7
#define PORTF_TRIS      0X00      //

#define nFault			PORTGbits.RG0
#define TX2				PORTGbits.RG1
#define RX2 			PORTGbits.RG2
#define MotorDrive_RG3	LATGbits.LATG3	
#define MotorDrive_RG4	LATGbits.LATG4
#define PORTG_TRIS      0X05      //  



#ifdef	__cplusplus
}
#endif

#endif	/* IO_H */

