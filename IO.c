/*
 * File:   IO.c
 * Author: Eric Liu
 *
 * Created on May 18, 2020, 10:50 AM
 */


#include <xc.h>
#include "IO.h"
#include "main.h"

void init(void);

/****************************************************************************
Function:		Port Initialization
******************************************************************************/
void init(void)
{
    TRISA = PORTA_TRIS;
    TRISB = PORTB_TRIS;
    TRISC = PORTC_TRIS;
    TRISD = PORTD_TRIS;
    TRISE = PORTE_TRIS;
    TRISF = PORTF_TRIS;
    TRISG = PORTG_TRIS;
    
    //enable or disable adc converter
    ADCON0bits.VCFG1 = 0; // internal VDD as reference
	ADCON0bits.VCFG0 = 0; // internal VDD as reference
    ADCON0bits.ADON=0;// 0 = OFF adc 1 = ON adc

    //AD converter configuration register
	ADCON1bits.ADFM = 1;  //  right justified
	ADCON1bits.ADCAL = 0; //  calibration disabled
	ADCON1bits.ACQT2 = 0; //
	ADCON1bits.ACQT1 = 1; //
	ADCON1bits.ACQT0 = 0; //
	ADCON1bits.ADCS2 = 1; //
	ADCON1bits.ADCS1 = 0; //
	ADCON1bits.ADCS0 = 0; //
    
    //WDT configuration
	WDTCONbits.ADSHR = 1; // shared address access
    
	//CONFIG2Hbits.WDTPS3 = 1;
    //WDTCONbits.SWDTEN = 1;
    
    //to enable or disable analog port - 0 is enable 1 is disable
	ANCON0bits.PCFG0 = 0; // AN0 analog port
	ANCON0bits.PCFG1 = 1; // RA1 digital port
	ANCON0bits.PCFG2 = 1; // RA2 digital port
	ANCON0bits.PCFG3 = 1; // RA3 digital port
	ANCON0bits.PCFG4 = 1; // RA5 digital port
    ANCON0bits.PCFG7 = 1; // RA5 digital port
    
    WDTCONbits.ADSHR = 0;
    
    INTCONbits.GIE = 1;	/* Enable Global Interrupt */
    //INTCONbits.PEIE = 1;/* Enable Peripheral Interrupt */
    //PIE1bits.RCIE = 1;	/* Enable Receive Interrupt*/
    //PIE1bits.TXIE = 1;	/* Enable Transmit Interrupt*/
    
    OSCCONbits.SCS = 0b11; //Postscaled internal clock (INTRC/INTOSC derived)
    OSCCONbits.IRCF = 0b111;//8 MHz (INTOSC drives clock directly)
	
}