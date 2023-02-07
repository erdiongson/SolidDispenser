/*
 * File:   main.c
 * Author: Eric Liu
 *
 * Created on November 30, 2019, 10:21 AM
 * Version 3.1 : i. Move from old complier C18 to new complier XC8
 *               ii. remove old timer and change to use XC8 timer
 */


#include <xc.h>
#include "IO.h"
#include "DRV8872.h"
#include "main.h"
#include "Led_Display.h"
#include "i2c.h"
#include "pic18f65j50.h"

#define VERSION 31

#define OFF         0
#define ON			1
#define MANUAL_MODE	2
#define IDLE_MODE	3
#define AUTO_MODE	4
#define NORMAL   	0x3F
#define FAST    	0x00
#define SLOW    	0x7F
#define one_sec     0x81
#define two_sec     0x82
#define three_sec   0x83
#define four_sec    0x84
#define five_sec    0x85
#define Serial_SOT	0xA5
#define Serial_EOT	0x5A
#define NAK         0x15
#define CMD_BUSY    0x16

unsigned char Serial_Flag;
unsigned char Serial_GData;
unsigned char Serial_Buffer[16];
unsigned char Serial_Temp_Buffer[16];
unsigned char Serial_Buffer_Out[16];
unsigned char Stop = 0;
unsigned char motor_status;
unsigned char Busy = 0;
unsigned char vibration_mode;
unsigned char data;

unsigned int Serial_Count;
unsigned int EAdd_High, EAdd_Low, ETemp;
unsigned int Motor_Speed;
unsigned int Vmotor_Time = 2000;
unsigned int Motor_Stop_Delay_Time = 0;
unsigned int Device_ID;
unsigned int Motor_Pause_Time = 0;
unsigned int NUM;
unsigned int NUM_REC;
unsigned int i=0;

volatile char OpMode = MANUAL_MODE;
volatile char IR_SENSORF = 0;

volatile unsigned char pause_Time;
volatile unsigned char vib_Time;
volatile unsigned char delay_motor_stop_time;
volatile unsigned char PWM_Duty_Cycle;
volatile unsigned char PWM_reg = 0x3F;

void init(void);
void initMotor(void);
void Set_RG3_PWM(void);
void Clr_RG3_PWM(void);
void MotorON_PWM(void);
void MotorBREAK(void);
void Read_IR(void);
void MotorPosition_Init(void);
void STLED316s_Delay (void);
void STLED316s_SPI_SendData ( unsigned char Data );
void WriteSTLED316SData( int number, char v_mode);
void WriteSTLED316SMode( char msg);
void InitSTLED316( unsigned char Brightness );
unsigned char Get7Seg (int Digit);
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
void WriteSTLED316SErr( char msg);


/****************************************************************************
Function:		Main Loop
******************************************************************************/
void main(void) 
{    
    init();
    InitSTLED316(0x77);
    initMotor();
    i2c_Init();
    initUSART();
    VIB_MOTOR_ON = 0;
    
    /* Enable interrupt priority */
  	RCONbits.IPEN = 1;
    
    /* Make receive interrupt high priority */
  	IPR1bits.RCIP = 1;
    
    /* Enable all high priority interrupts */
  	INTCONbits.GIEH = 1;
    INTCONbits.GIE=1;
    //PIE1bits.RCIE=1;
 
    //RCSTA1bits.CREN = 1; // Continuos receiver
    
    WDTCONbits.SWDTEN = ON; // turn ON watchdog timer
    
    GREEN_LED = 1;
    
    WriteSTLED316SData(VERSION, 0xFF);
    __delay_ms(100);
    //MotorPosition_Init();
    
    NUM = 1;
    WriteSTLED316SData(NUM, vibration_mode);
    NUM_REC = 1;
    
    vibration_mode = 1;
    
    if (vibration_mode)
        RED_LED = 1;
    else
        RED_LED = 0;
    
    
 /****************************************************************************
               Read Parameters
******************************************************************************/
    
    /*************************************************************************
               Read Device ID
    **************************************************************************/
    ETemp = read_i2c(EEPROM_DevID);
    Device_ID = ETemp & 0x00FF;
    if(Device_ID<0x31 || Device_ID>0x3A)
    {
        Device_ID=0x31;
        write_i2c(EEPROM_DevID, Device_ID);
    }
    
    /*************************************************************************
               Read Motor Pause Time
    **************************************************************************/
    ETemp = read_i2c(EEPROM_MotorPauseTime);
    pause_Time = ETemp & 0x00FF;
    if(pause_Time<0x30 || pause_Time>0x35)
    {
        pause_Time = 0x30;
        Motor_Pause_Time=0;
        write_i2c(EEPROM_MotorPauseTime, pause_Time);
    }
    
    /*************************************************************************
               Read Motor Stop Delay Time
    **************************************************************************/ 
    ETemp = read_i2c(EEPROM_MotorStopDelay);
    delay_motor_stop_time = ETemp & 0x00FF;
    if(delay_motor_stop_time>0x96)
    {
        delay_motor_stop_time=0x00;
        write_i2c(EEPROM_MotorStopDelay, delay_motor_stop_time);
        Motor_Stop_Delay_Time=0;
    }
    else
    {
        Motor_Stop_Delay_Time = delay_motor_stop_time;             
    }
    
    /*************************************************************************
               Read Motor Vibration Time
    **************************************************************************/
    ETemp = read_i2c(EEPROM_VibTime);
    vib_Time = ETemp & 0x00FF;   
    if( (vib_Time!=one_sec && vib_Time!=two_sec && vib_Time!=three_sec && vib_Time!=four_sec && vib_Time!=five_sec) )
    {
        Vmotor_Time = 2000;      // default is 2 sec
        vib_Time = two_sec;
        write_i2c(EEPROM_VibTime,vib_Time);
    }
    else
    {
        switch(vib_Time)
        {
            case 0x81:
                Vmotor_Time=1000;
                break;
            case 0x82:
            default:
                Vmotor_Time=2000;      // default is 2 sec  
                vib_Time=0x82;
                break;
            case 0x83:
                Vmotor_Time=3000;
                break;
            case 0x84:
                Vmotor_Time=4000;
                break;
            case 0x85:
                Vmotor_Time=5000;
                break;
        }
    }
    
    /*************************************************************************
               Read Motor Speed
    **************************************************************************/  
    PWM_reg = NORMAL;
     
    ETemp = read_i2c(EEPROM_MotorSpeed); 
    
    PWM_reg = ETemp & 0x00FF;
    
    if( (PWM_reg!=FAST && PWM_reg!=NORMAL && PWM_reg!=SLOW) )
    {
        PWM_reg=NORMAL;
        write_i2c(EEPROM_MotorSpeed,PWM_reg);       
    }
    
    MotorPosition_Init();
    
 /****************************************************************************
                While(1) loop
******************************************************************************/    
    while(1)
    {
        ClrWdt();
        
        switch(OpMode)
        {
             /****************************************************************
              Manual Operation mode
            *****************************************************************/
            case MANUAL_MODE:
                
                NUM = NUM_REC;

                if (CENTER == 0) 
                {
                    ToggleVIB_Mode();
                    while (CENTER == 0);
                }

                if ((RIGHT == 0) && NUM != 99) 
                {
                    NUM = NUM + 1;
                    while (RIGHT == 0);
                }

                if (LEFT == 0 && NUM != 0) 
                {
                    NUM = NUM - 1;
                    while (LEFT == 0);
                }

                if (DOWN == 0 && NUM <= 89) 
                {
                    NUM = NUM + 10;
                    while (DOWN == 0);
                }

                if (UP == 0 && NUM >= 10) 
                {
                    NUM = NUM - 10;
                    while (UP == 0);
                }

                NUM_REC = NUM;
                WriteSTLED316SData(NUM, vibration_mode);

                if (MOTOR_ON_BUT == 0)
                {
                    Busy = 1;
                    Homing_Again_Manual();
                    Stop = 0;
                    Busy = 0;

                    do
                    {
                        WriteSTLED316SErr('E');
                    }
                    while (!MOTOR_ON_BUT); //Loop until the pushbutton release
                }
            
            /****************************************************************
              Auto Operation mode
            *****************************************************************/
            case AUTO_MODE:
                
                if(Serial_Flag==1)
                {                   
                    
                    switch(Serial_Buffer[1])
                    {
                        case 0x44: //start or stop command
                            
                            if(Serial_Buffer[2] == 0xF1 && Busy == 0) //auto start command
                            {
                                Stop = 0;
                                Busy = 1;
                                NUM = NUM_REC;
                                                              
                                Homing_Again_Auto();
                                
                            }
                            else if(Serial_Buffer[2] == 0xF2 && Busy == 0) //semi auto
                           {
                                Stop = 0;
                                Busy = 1;
                                NUM = NUM_REC;
                                WriteSTLED316SData(NUM, vibration_mode);
                                                              
                                Homing_Again_Manual();
                                
                                //send semi-auto dispense completed command to PC
                                if(Stop == 0)
                                {
                                    Serial_Buffer_Out[0] = Serial_SOT;
                                    Serial_Buffer_Out[1] = 0x44;
                                    Serial_Buffer_Out[2] = 0xF9;
                                    Serial_Buffer_Out[3] = 0X3D;
                                    Serial_Buffer_Out[4] = Serial_EOT;
                                    for (i=0; i<5;i++)
                                        Write1USART(Serial_Buffer_Out[i]);
                                }
                                
                            }
                                flush();
                                flushOut();
                                Stop = 0;
                                Busy = 0;    
                                break;
                            
                            
                        case 0x23: //program device ID
                            
                            if(Busy==0)
                            {
                                Busy = 1;
                                if(Serial_Buffer[2]>=0x30 && Serial_Buffer[2]<=0x35)
                                {
                                    //Device_ID = Serial_Buffer[2];
                                    pause_Time = Serial_Buffer[2];
                                    switch(pause_Time)
                                    {
                                        case 0x30:
                                        default:
                                            Motor_Pause_Time = 0;
                                            break;
                                            
                                        case 0x31:
                                            Motor_Pause_Time = 1000;
                                            break;
                                            
                                        case 0x32:
                                            Motor_Pause_Time = 2000;
                                            break;
                                            
                                        case 0x33:
                                            Motor_Pause_Time = 3000;
                                            break;
                                            
                                        case 0x34:
                                            Motor_Pause_Time = 4000;
                                            break;
                                            
                                        case 0x35:
                                            Motor_Pause_Time = 5000;
                                            break;                                            
                                    }
                                }
                                //write_i2c(EEPROM_DevID, Device_ID);
                                write_i2c(EEPROM_MotorPauseTime, pause_Time);
                                flush();
                                Busy = 0;
                            }
                            break;
                            
                            
                        case 0x51: //query status command
                            
                            if(Busy==0)
                            {
                                if (Serial_Buffer[2] == 0x00)
                                {                               
                                    Busy = 1;
                                    //Device_ID = read_i2c(EEPROM_DevID);
                                    pause_Time = read_i2c(EEPROM_MotorPauseTime);
                                    vib_Time = read_i2c(EEPROM_VibTime);
                                    Motor_Speed = read_i2c(EEPROM_MotorSpeed);
                                    delay_motor_stop_time = read_i2c(EEPROM_MotorStopDelay);                                                                      

                                    Serial_Buffer_Out[0] = 0x51;
                                    //Serial_Buffer_Out[1] = Device_ID;
                                    Serial_Buffer_Out[1] = pause_Time;
                                    Serial_Buffer_Out[2] = Motor_Speed;
                                    Serial_Buffer_Out[3] = vib_Time;
                                    Serial_Buffer_Out[4] = delay_motor_stop_time;
                                    
                                    __delay_ms(100);
                                    
                                    for (i=0; i<5;i++)
                                        Write1USART(Serial_Buffer_Out[i]);
                                }
                                flushOut();
                                Busy = 0;
                            }
                            break;
                            
                        case 0x64: //program motor speed
                            
                            if(Busy==0)
                            {
                                Busy = 1;
                                PWM_Duty_Cycle = Serial_Buffer[2];

                                switch(PWM_Duty_Cycle)
                                {
                                    case 0x00:  
                                        PWM_reg=FAST;
                                        break;

                                    case 0x3F:
                                    default:
                                        PWM_reg=NORMAL;
                                        break;

                                    case 0x7F:  
                                        PWM_reg=SLOW;                                   
                                        break;
                                }
                                    write_i2c(EEPROM_MotorSpeed,PWM_reg);
                                    Busy = 0;
                            }
                            break;
                            
                        case 0x65: //program motor pause/vibration time
                            
                            if(Busy == 0)
                            {
                                Busy = 1;
                                vib_Time = Serial_Buffer[2];

                                switch(vib_Time)
                                {
                                    case 0x81:  
                                        Vmotor_Time=1000;
                                        break;

                                    case 0x82:
                                    default:
                                        Vmotor_Time=2000;
                                        vib_Time=0x82;
                                        break;

                                    case 0x83:  
                                        Vmotor_Time=3000;                                   
                                        break;

                                    case 0x84:  
                                        Vmotor_Time=4000;                                   
                                        break;

                                    case 0x85:  
                                        Vmotor_Time=5000;                                   
                                        break;
                                }
                                    write_i2c(EEPROM_VibTime,vib_Time);
                                    Busy = 0;
                            }
                            
                        case 0x66: //program motor stop delay
                            
                            if(Busy == 0)
                            {
                                Busy = 1;
                                delay_motor_stop_time = Serial_Buffer[2];
                                Motor_Stop_Delay_Time = delay_motor_stop_time;                             
                                write_i2c(EEPROM_MotorStopDelay,delay_motor_stop_time);
                                Busy = 0;
                            }
                            
                            break;
                            
                    }
                    
                    Serial_Flag = 0;
                    Serial_GData = 0;
                    OpMode = MANUAL_MODE;
                }
                else
                {            
                    OpMode = MANUAL_MODE;
                }
                
            
        } //end switch case
    }//end while loop


    //return;
}

/****************************************************************************
Function:		USART Interrupt function
******************************************************************************/
void __interrupt() high_isr(void)
{
    unsigned char c, i;
	unsigned char Temp;
    
    if (DataRdy1USART()) //USART Receive interrupt FLAG
	{
		//Get the character received from the USART
		c=Read1USART();
        
		if ((c == Serial_SOT) && ((Serial_Flag == 0)||(Busy == 1)))
		{
			Serial_GData = 1;
			Serial_Count = 0;
			Serial_Buffer[Serial_Count] = c;
            Serial_Count++;
		}
		else if (Serial_GData == 1)
		{           
			Serial_Buffer[Serial_Count] = c;
            
			if(Serial_Count<5)
            {
                if (c == Serial_EOT)
                {
                    Temp = Serial_Buffer[1] + Serial_Buffer[2];

                    if (Temp == Serial_Buffer[3])
                    {
                        if(Busy==0 || Serial_Buffer[2]==0xF5)
                        {
                            for (i=0; i<5; i++)
                                Write1USART(Serial_Buffer[i]);
                        }
                        else
                        {
                            for (i=0; i<5; i++)
                                Write1USART(CMD_BUSY);
                        }

                        Serial_Flag = 1;
                        Serial_Count = 0;
                        OpMode = AUTO_MODE;

                    }
                    else
                    {
                        Serial_Flag = 0;
                        Serial_Count = 0;
                        for (i=0; i<5; i++)
                            Write1USART(NAK);
                    }

                    if(Serial_Buffer[2]==0xF5 && OpMode == AUTO_MODE)
                    {
                        Stop = 1;
                    }
                }
            }
            else
            {
                Serial_GData = 0;
            }
            
            Serial_Count++;           
		}
	}
    else
    {
        if (RCSTA1bits.OERR == 1)
        {
            RCSTA1bits.OERR = 0; // clear overrun if it occurs
            RCSTA1bits.CREN = 0;
            RCSTA1bits.CREN = 1;
        }
    }
    
}

/****************************************************************************
Function:		delay 1ms function
******************************************************************************/
void delay_1ms(unsigned int time)
 {
   while(time > 0)
   {
       __delay_ms(1);
      time--;
    }
}

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

/****************************************************************************
Function:		i2C Function
******************************************************************************/
void write_i2c(long address, int data)
{
	i2c_Start();      					// send Start
	i2c_Address(I2C_SLAVE, I2C_WRITE);	// Send  slave address with write operation
	i2c_Write((unsigned char)(address/0x100));			// High Address
	i2c_Write((unsigned char)(address&0xFF));			// Low Address
	i2c_Write((unsigned char)data);					// Data
 	i2c_Stop();			  				// send Stop
}

//long read_i2c(unsigned char address)
int read_i2c(long address)
{
	int read_byte;

	// Read one byte (i.e. servo pos of one servo)
	i2c_Start();      					// send Start
	i2c_Address(I2C_SLAVE, I2C_WRITE);	// Send slave address with write operation
	i2c_Write((unsigned char)(address/0x100));			// High Address
	i2c_Write((unsigned char)(address&0xFF));			// Low Address
	i2c_Restart();						// Restart
	i2c_Address(I2C_SLAVE, I2C_READ);	// Send slave address with read operation
	read_byte = i2c_Read(0);			// Read one byte
										// If more than one byte to be read, (0) should
										// be on last byte only
										// e.g.3 bytes= i2c_Read(1); i2c_Read(1); i2c_Read(0);
   	i2c_Stop();							// send Stop
	return (read_byte);					// return byte. If reading more than one byte
										// store in an array
}

/****************************************************************************
Function:		Motor Function
******************************************************************************/
void initMotor(void)
{
    PR4 = 0xC7;
    T3CONbits.T3CCP1 = 0;   // to set Timer3 and Timer4 are the clock sources for ECCP2, ECCP3, CCP4 and CCP5;
    T3CONbits.T3CCP2 = 1;    
    T4CON = 0x00;
}

void Set_RG3_PWM(void)
{	
    CCP4CON = 0x0C;  //0b00001100;	// setup as PWM 
    CCPR4L = PWM_reg;
 	T4CONbits.TMR4ON = 1;
}

void Clr_RG3_PWM(void)
{
	CCP4CON = 0x00;		//Disable PWM
    T4CONbits.TMR4ON = 0;
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

/****************************************************************************
Function:		7Segment Display
******************************************************************************/
void InitSTLED316( unsigned char Brightness ) //default 0x77
{   
	int i;
	
	STB_Set;	
											
	STB_Clr;
						
	STLED316s_SPI_SendData ( CHIP_CONFIG_ADDRESS + CHIP_CONFIG_PAGE + FIX_Address + WRITE_Command ); 
	                     //    0x0                  0x10                 0x20            0 
	
	STB_Set;	
												
	STB_Clr;
	
	
	STLED316s_SPI_SendData ( CHIP_CONFIG_ADDRESS + CHIP_CONFIG_PAGE + FIX_Address + WRITE_Command );
	                     //    0x0                  0x10                 0x20            0 
	STLED316s_SPI_SendData ( ( ( Brightness>>4 )<<5 ) | 0x00 | 0x5 );	                    
												
	STB_Set;	
										
	STB_Clr;

			 
	STLED316s_SPI_SendData ( DIGIT_BRIGHTNESS_PAGE + DIGIT_BRIGHTNESS_ADDRESS + INC_Address + WRITE_Command );
	                     //   0x10			  	  1                           0                  0	   


/**********************************************************************************************************/

	STB_Set;	
											
	STB_Clr;


    STLED316s_SPI_SendData ( DIGIT_BRIGHTNESS_PAGE + DIGIT_BRIGHTNESS_ADDRESS + INC_Address + WRITE_Command );
	                        //   0x10		    1                           0                  0	   
	STLED316s_SPI_SendData ( Brightness );					         
	STLED316s_SPI_SendData ( Brightness );							 
	STLED316s_SPI_SendData ( Brightness );							 		

/**********************************************************************************************************/

	STB_Set;	
										
	STB_Clr;

						/* LED灯亮度设置 */	
	STLED316s_SPI_SendData ( LED_BRIGHTNESS_PAGE + LED_BRIGHTNESS_ADDRESS + INC_Address + WRITE_Command );
	                     //   0x11			  0                           0                  0	   

	STB_Set;	
											
	STB_Clr;

    STLED316s_SPI_SendData ( LED_BRIGHTNESS_PAGE + LED_BRIGHTNESS_ADDRESS + INC_Address + WRITE_Command );
	                       //   0x11		  0                           0                  0	   
	STLED316s_SPI_SendData ( Brightness );					         
	STLED316s_SPI_SendData ( Brightness );							 
	STLED316s_SPI_SendData ( Brightness );							 	
	STLED316s_SPI_SendData ( Brightness );				         

/*********************************************************************************************************/

	STB_Set;
}

void STLED316s_Delay(void)
{
//	char i;
// 	
// 	for( i = 0 ; i < 10 ; i++ );
}

void STLED316s_SPI_SendData( unsigned char Data )
{
	char i,temp,t;
	
	for(i = 0; i < 8 ;i ++)
	{
		CLK_Clr;		//SCL=0;		
		temp =  Data & (0x1 << i);		
		if(temp)
			DATA_Set;	//SDATA=1
		else
			DATA_CLr;	//SDATA=0
		for( t = 0 ; t < 5; t++);
		CLK_Set;		
		for( t = 0 ; t < 5 ; t++);					
	}
}	

unsigned char Get7Seg (int Digit)
{
	unsigned char Data;
	
	switch (Digit)
	{
		case 0:
			Data = Seg_0;
			break;
		case 1:
			Data = Seg_1;
			break;
		case 2:
			Data = Seg_2;
			break;
		case 3:
			Data = Seg_3;
			break;
		case 4:
			Data = Seg_4;
			break;
		case 5:
			Data = Seg_5;
			break;
		case 6:
			Data = Seg_6;
			break;
		case 7:
			Data = Seg_7;
			break;
		case 8:
			Data = Seg_8;
			break;
		case 9:
			Data = Seg_9;
			break;
		default:
			Data = Seg_Blank;
			break;
	}
	return (Data);
}	//

void WriteSTLED316SData( int number, char v_mode)
{   
	unsigned char data3, data4;
	int NUM_DIG0, NUM_DIG1;
	
	NUM_DIG0=(int) number%10;
	NUM_DIG1=(int) number/10;

	STB_Clr;
					
	STLED316s_SPI_SendData (0x00 );	
	
	STB_Set;	
												
	STB_Clr;
				
	STLED316s_SPI_SendData (0x00);	

	data4 = Get7Seg(NUM_DIG1);
	data3 = Get7Seg(NUM_DIG0);
	
	if (v_mode)
	{
		if (v_mode == 0xFF)
			data4 |= Seg_Dot;	//display the dot	for version dot
		else
			data3 |= Seg_Dot;	//display the dot
	}
	
    STLED316s_SPI_SendData (data4);
    STLED316s_SPI_SendData (data3);
			 	
	STB_Set;
														
	STB_Clr;
		 										
	STLED316s_SPI_SendData ( DISPLAY_ON );				 /* 发送打开显示命令 */

	STB_Set;		
}	

void WriteSTLED316SMode( char msg)
{   
	STB_Clr;
				
	STLED316s_SPI_SendData (0x00 );	
				  	

	STB_Set;	
												
	STB_Clr;
			
	STLED316s_SPI_SendData (0x00);	
	
	switch(msg)
	{
	  case 'E': //error start button not release
	    STLED316s_SPI_SendData (Seg_E); 
        STLED316s_SPI_SendData (Seg_0); 
      	break; 
      case 'O':
	    STLED316s_SPI_SendData (Seg_0); //0
        STLED316s_SPI_SendData (Seg_P); //P
     	break;
      case 'C':
	    STLED316s_SPI_SendData (Seg_C); //C
        STLED316s_SPI_SendData (Seg_A); //A
     	 break;
      case 'X':
	    STLED316s_SPI_SendData (Seg_Blank); //0
        STLED316s_SPI_SendData (Seg_Blank); //0
      	break;
      case 'G':
	    STLED316s_SPI_SendData (Seg_Blank); //0
        STLED316s_SPI_SendData (Seg_0); //0
        break;
      case 'S':    
	    STLED316s_SPI_SendData (Seg_E); //0
        STLED316s_SPI_SendData (Seg_1); //0
        break;
      case 'A': //Auto - continuous mode    
	    STLED316s_SPI_SendData (Seg_A); //0
        STLED316s_SPI_SendData (Seg_U); //0
        break;
      default:
      	break;
	}

	STB_Set;
															
	STB_Clr;
		 										
	STLED316s_SPI_SendData ( DISPLAY_ON );				
    
	STB_Set;		
}

void WriteSTLED316SErr( char msg)
{   
	STB_Clr;
				
	STLED316s_SPI_SendData (0x00 );	
				  	

	STB_Set;	
												
	STB_Clr;
			
	STLED316s_SPI_SendData (0x00);	
	
	switch(msg)
	{
	  case 'E': //error start button not release
	    STLED316s_SPI_SendData (Seg_E); 
        STLED316s_SPI_SendData (Seg_0); 
      	break; 
      case 'O':
	    STLED316s_SPI_SendData (Seg_0); //0
        STLED316s_SPI_SendData (Seg_P); //P
     	break;
      case 'C':
	    STLED316s_SPI_SendData (Seg_C); //C
        STLED316s_SPI_SendData (Seg_A); //A
     	 break;
      case 'X':
	    STLED316s_SPI_SendData (Seg_Blank); //0
        STLED316s_SPI_SendData (Seg_Blank); //0
      	break;
      case 'G':
	    STLED316s_SPI_SendData (Seg_Blank); //0
        STLED316s_SPI_SendData (Seg_0); //0
        break;
      case 'S':    
	    STLED316s_SPI_SendData (Seg_E); //0
        STLED316s_SPI_SendData (Seg_1); //0
        break;
      case 'A': //Auto - continuous mode    
	    STLED316s_SPI_SendData (Seg_A); //0
        STLED316s_SPI_SendData (Seg_U); //0
        break;
      default:
      	break;
	}

	STB_Set;
															
	STB_Clr;
		 										
	STLED316s_SPI_SendData ( DISPLAY_ON );				
    
	STB_Set;		
}

/****************************************************************************
Function:		IR Sensor Function
******************************************************************************/
void Read_IR(void) 
{
    if(IR_Status == 1)
    {
        IR_SENSORF = 0;
    } 
    else
    {
        IR_SENSORF = 1;
    } 
}

/****************************************************************************
Function:		Initialize Motor startup position
******************************************************************************/
void MotorPosition_Init(void)
{
    IR_ON = 1;
    MotorON_PWM(); // turn ON motor
    __delay_ms(350);
        
    do
    {
        Read_IR();        
    }while(IR_SENSORF != 0);
        
    do
    {
        Read_IR();      
    }while(IR_SENSORF != 1);
    
    delay_1ms(Motor_Stop_Delay_Time);
    MotorBREAK();
    __delay_ms(2000);
    IR_SENSORF=0;
                      
    IR_ON = 0;
    
}

/****************************************************************************
Function:		Toggle vibration variable
******************************************************************************/
void ToggleVIB_Mode(void) 
{
    if (vibration_mode)
        vibration_mode = 0;
    else
        vibration_mode = 1;
    
    if (vibration_mode)
        RED_LED = 1;
    else
        RED_LED = 0;

    //write_i2c(EEPROM_VibMode, vibration_mode);

    WriteSTLED316SData(NUM, vibration_mode); //Update the display
}

/****************************************************************************
Function:		Homing for Manual Mode
******************************************************************************/
void Homing_Again_Manual(void) 
{
    
    IR_SENSORF = 0;
  
    if (vibration_mode == 1) 
    {
        VIB_MOTOR_ON = 1;    
        delay_1ms(Vmotor_Time);
        VIB_MOTOR_ON = 0;
        __delay_ms(300);
    } 
    else 
    {
        VIB_MOTOR_ON = 0;
        __delay_ms(300);
    }
  
    IR_ON = 1; //turn ON IR sensor
    
    while (NUM>0) 
    {
        ClrWdt();
        readWeighingData();
        delay_1ms(Motor_Pause_Time);
        MotorON_PWM(); // Turn ON motor
        __delay_ms(350); //default150

        do
        {
            Read_IR();
        }while(IR_SENSORF != 0);
        
        do
        {
            Read_IR();
        }while(IR_SENSORF != 1);
        
        IR_SENSORF = 0;

        delay_1ms(Motor_Stop_Delay_Time);
        MotorBREAK();

        NUM--;
        WriteSTLED316SData(NUM, vibration_mode);

        if (vibration_mode == 1 && NUM != 0) 
        {
            VIB_MOTOR_ON = 1;                    
            delay_1ms(Vmotor_Time);
            
            VIB_MOTOR_ON = 0;
            __delay_ms(300);
        } 
        else if(vibration_mode == 0 && NUM != 0) 
        {
            VIB_MOTOR_ON = 0;
            if (NUM != 0) 
            {
                delay_1ms(Vmotor_Time);
            } 
            else 
            {
                __delay_ms(500);
            }
        }
      
        if(Stop==1)
            break;
        
    } // end while

    IR_ON = 0; // turn off IR sensor
    
    if (NUM == 0 || Stop==1) 
    {
    
        if (vibration_mode == 1) 
        {
            VIB_MOTOR_ON = 1;
            delay_1ms(Vmotor_Time);
            
            VIB_MOTOR_ON = 0;
            __delay_ms(300);
        }
        
        IR_SENSORF = 0;
        
    }
   
    NUM = 0;
    OpMode = MANUAL_MODE;

}

/****************************************************************************
Function:		Homing for Auto Mode
******************************************************************************/
void Homing_Again_Auto(void) 
{
    
    IR_SENSORF = 0;
    WriteSTLED316SMode('A'); //show AU (auto mode)
  
    if (vibration_mode == 1) 
    {
        VIB_MOTOR_ON = 1;    
        delay_1ms(Vmotor_Time);
        VIB_MOTOR_ON = 0;
        __delay_ms(300);
    } 
    else 
    {
        VIB_MOTOR_ON = 0;
        __delay_ms(300);
    }
  
    IR_ON = 1; //turn ON IR sensor
    
    while ( Stop==0) 
    {
        ClrWdt();
        readWeighingData();
        delay_1ms(Motor_Pause_Time);
        MotorON_PWM(); // Turn ON motor
        __delay_ms(350); //default150

        do
        {
            Read_IR();
        }while(IR_SENSORF != 0);
        
        do
        {
            Read_IR();
        }while(IR_SENSORF != 1);
        
        IR_SENSORF = 0;

        delay_1ms(Motor_Stop_Delay_Time);
        MotorBREAK();

        if (vibration_mode == 1) 
        {
            VIB_MOTOR_ON = 1;                    
            delay_1ms(Vmotor_Time);
            
            VIB_MOTOR_ON = 0;
            __delay_ms(300);
        } 
        else
        {
            VIB_MOTOR_ON = 0; 
             delay_1ms(Vmotor_Time);            
        }
      
        if(Stop==1)
            break;
        
    } // end while

    IR_ON = 0; // turn off IR sensor
    
    if (NUM == 0 || Stop==1) 
    {
    
        if (vibration_mode == 1) 
        {
            VIB_MOTOR_ON = 1;
            delay_1ms(Vmotor_Time);
            
            VIB_MOTOR_ON = 0;
            __delay_ms(300);
        }
        
        IR_SENSORF = 0;
        
    }
   
    NUM = 0;
    OpMode = AUTO_MODE;

}

/****************************************************************************
Function:		Flush serial in buffer
******************************************************************************/
void flush(void)
{
    int i;
    
    for(i=0; i<16; i++)
    {
        Serial_Buffer[i]=0x00;
    }
}

/****************************************************************************
Function:		Flush serial out buffer
******************************************************************************/
void flushOut(void)
{
    int i;
    
    for(i=0; i<16; i++)
    {
        Serial_Buffer_Out[i]=0x00;
    }
}

/****************************************************************************
Function:		Send read weight command to weighing machine
******************************************************************************/
void readWeighingData(void)
{
    int i;
    
    Serial_Buffer_Out[0] = 0xA5;
    Serial_Buffer_Out[1] = 0x45;
    Serial_Buffer_Out[2] = 0x00;
    Serial_Buffer_Out[3] = 0x45;
    Serial_Buffer_Out[4] = Serial_EOT;
    for (i=0; i<5;i++)
        Write1USART(Serial_Buffer_Out[i]);
    
    flushOut();
}