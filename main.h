/* 
 * File:   main.h
 * Author: victorlu
 *
 * Created on November 29, 2019, 2:43 PM
 * 
 * Created: 2023-04-19, 12:00PM
 * Author: erdiongson
 * Version 3.7_1R : i. Created version dedicated for SDB1R product
 *                  ii. Added UART2 for communication with Arduino AtMega
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

    // CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on SWDTEN bit))
#pragma config PLLDIV = 1	//1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config STVREN = ON	//OFF     // Stack Overflow/Underflow Reset Enable bit (Reset on stack overflow/underflow disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
#pragma config DEBUG = OFF

    // CONFIG1H
#pragma config CPUDIV = OSC1    // CPU System Clock Postscaler (No CPU system clock divide)
#pragma config CP0 = OFF        // Code Protection bit (Program memory is not code-protected)

    // CONFIG2L
#pragma config FOSC = INTOSCPLLO	//HSPLL   // Oscillator Selection bits (INTOSC with PLL enabled, CLKO on RA6 and Port function on RA7)
#pragma config FCMEN = OFF	//ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF	//ON        // Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit (Two-Speed Start-up enabled)

    // CONFIG2H
#pragma config WDTPS = 2048    // Watchdog Timer Postscaler Select bits (1:32768) default:32768

    // CONFIG3L

    // CONFIG3H
#pragma config CCP2MX = DEFAULT // ECCP2 MUX bit (ECCP2/P2A is multiplexed with RC1)
#pragma config MSSPMSK = MSK5	//MSK7   // MSSP Address Masking Mode Select bit (7-Bit Address Masking mode enable)


    //#include <xc.h>

#include <stdbool.h>

#define _XTAL_FREQ 8000000

#define EEPROM_VibMode              0x0010
#define EEPROM_DevID                0x0020
#define EEPROM_MotorSpeed           0x0030
#define EEPROM_VibTime              0x0040
#define EEPROM_MotorStopPosition    0x0050
#define EEPROM_MotorPauseTime       0x0060
#define EEPROM_PWMDutyCycle         0x0070

#define Busy1USART( )  (!TXSTA1bits.TRMT)
#define DataRdy1USART( ) (PIR1bits.RC1IF)

    //20230419 (SDB1R): erdiongson - Added for UART2
#define Busy2USART( )  (!TXSTA2bits.TRMT)
#define DataRdy2USART( ) (PIR3bits.RC2IF)    

    extern unsigned char PWM_reg;

    void init(void);
    void initMotor(void);
    void Set_RG3(void);
    void Clr_RG3(void);
    void MotorON(void);
    void MotorBREAK(void);
    unsigned int Read_IR(void);
    void MotorPosition_Init(void);
    void STLED316s_Delay(void);
    void STLED316s_SPI_SendData(unsigned char Data);
    void WriteSTLED316SData(int number, char v_mode);
    void WriteSTLED316SMode(char msg);
    void WriteSTLED316SErr(char msg);
    void InitSTLED316(unsigned char Brightness);
    unsigned char Get7Seg(int Digit);
    void ToggleVIB_Mode(void);
    void Homing_Again_Manual(void);
    void i2c_Init(void);
    void i2c_Wait(void);
    void i2c_Start(void);
    void i2c_Restart(void);
    void i2c_Stop(void);
    void i2c_Write(unsigned char data);
    void i2c_Address(unsigned char address, unsigned char mode);
    int i2c_Read(unsigned char ack);
    void write_i2c(long address, int data);
    int read_i2c(long address);
    void Vibration_Time(unsigned int vib_time);
    void Delay_Time(unsigned int delaytime);
    void initUSART(void);
    void Write1USART(char data);
    char Read1USART(void);
    void flush(void);
    void flushOut(void);
    void readWeighingData(void);
    void Homing_Again_Auto(void);
    //20230419 : erdiongson - Added Write2USART and Read2USART for SDB1R
    void Write2USART(char data);
    char Read2USART(void);

    //Added by leo
    unsigned int duty_cycle = 0;
    volatile unsigned char PWM_Duty_Cycle;
    void vibrationMotorControl(bool powerState, unsigned int pwm_msg);

    void PWM1_Init(long desiredFrequency);
    void PWM1_SetDutyCycle(unsigned int dutyCycle);

    void PWM1_Start();
    void PWM1_Stop();

#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

