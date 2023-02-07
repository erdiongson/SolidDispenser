/*
 * File:   usart.c
 * Author: Eric Liu
 *
 * Created on May 18, 2020, 10:48 AM
 */


#include <xc.h>
#include "IO.h"
#include "main.h"

void initUSART(void);
void Write1USART(char data);
char Read1USART(void);

/****************************************************************************
Function:		USART Initialization
******************************************************************************/
void initUSART(void)
{
    // setup USART
    TRISCbits.TRISC6 = 0; // TX as output
    TRISCbits.TRISC7 = 1; // RX as input
    TXSTA1bits.SYNC = 0; // Async operation
    TXSTA1bits.TX9 = 0; // No tx of 9th bit
    TXSTA1bits.TXEN = 1; // Enable transmitter
    RCSTA1bits.RX9 = 0; // No rx of 9th bit
    RCSTA1bits.CREN = 1; // Enable receiver
    TXSTA1bits.CSRC = 0; //Clock Source Select bit for syn - Asyn is dont care
    RCSTA1bits.ADDEN = 1;
    RCSTA1bits.SPEN = 1; // Enable serial port
    
    // Setting for 19200 BPS
    BAUDCON1bits.BRG16 = 0; // Divisor at 8 bit
    TXSTA1bits.BRGH = 1; // No high-speed baudrate
    PIE1bits.RC1IE = 1;
    PIE1bits.TX1IE = 0;
    //(FOSC/baudrate/64)-1 = SPBRG1
    SPBRG1 = 25;  //48MHZ = 38; // divisor value for 19200
    SPBRGH1 = 0;
    
}

/****************************************************************************
Function:		USART Write Function
******************************************************************************/
void Write1USART(char data)
{
  	TXREG1 = data;      // Write the data byte to the USART2

	while (Busy1USART());
}	// end Write1USART

/****************************************************************************
Function:		USART Read Function
******************************************************************************/
char Read1USART(void)
{
    char result;
    
    if (RCSTA1bits.OERR) 
    {    //has there been an overrun error?
        RCSTA1bits.CREN = 0; //disable Rx to clear error
        result = RCREG1;    //purge receive buffer
        result = RCREG1;    //purge receive buffer
        RCSTA1bits.CREN = 1;    //re-enable Rx
    } 
    else 
    {
        result = RCREG1;
    }
    
    return result;
}

