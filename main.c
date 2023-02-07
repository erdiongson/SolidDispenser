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

#define VERSION     33

#define OFF         0
#define ON			1
#define false       0
#define true		1

#define NORMAL   	0x3F //0x3F
#define FAST    	0x00
#define SLOW    	0x7F//0x7F
#define one_sec     0x81
#define two_sec     0x82
#define three_sec   0x83
#define four_sec    0x84
#define five_sec    0x85
#define Serial_SOT	0xA5
#define Serial_EOT	0x5A
#define NAK         0x15
#define CMD_BUSY    0x16

#define errorTime0  30 //IR sensor
#define Time0       20 //Low Power

enum Op_Mode{MANUAL_MODE, IDLE_MODE,AUTO_MODE};

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
unsigned int IR_SENSORF = 0;
unsigned int VOLTAGE=-0;

volatile char OpMode = MANUAL_MODE;
volatile long errorcounter = errorTime0; //IRsensor counter
volatile long LowPowerCounter = Time0;

volatile unsigned char pause_Time;
volatile unsigned char vib_Time;
volatile unsigned char delay_motor_stop_time;
volatile unsigned char PWM_Duty_Cycle;
volatile char TMR1IF_triggered = false;
unsigned char PWM_reg = 0x3F;

void init(void);
void initMotor(void);
void Set_RG3_PWM(void);
void Clr_RG3_PWM(void);
void MotorON_PWM(void);
void MotorBREAK(void);
unsigned int Read_IR(void);
void MotorPosition_Init(void);
void STLED316s_Delay (void);
void STLED316s_SPI_SendData ( unsigned char Data );
void WriteSTLED316SData( int number, char v_mode);
void WriteSTLED316SMode( char msg);
void WriteSTLED316SVibMode( char v_mode);
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
void InitTimer1(void);
void AD_capture_BattVoltage(void);
void Low_Power_Indicator(void);


/****************************************************************************
Function:		Main Loop
******************************************************************************/
void main(void) 
{    
    GIE = 0;
    init();
    InitSTLED316(0x77);
    initMotor();
    i2c_Init();
    initUSART();
    InitTimer1();
    
    VIB_MOTOR_ON = 0;
    IR_ON = 0;
    errorcounter = errorTime0;
  
    
    /* Enable interrupt priority */
  	RCONbits.IPEN = 1;
    
    /* Make receive interrupt high priority */
  	IPR1bits.RCIP = 1;
    
    /* Enable all high priority interrupts */
  	INTCONbits.GIEH = 1;
    INTCONbits.GIE=1;
    //PIE1bits.RCIE=1;
 
    //RCSTA1bits.CREN = 1; // Continuos receiver
    
    WDTCONbits.SWDTEN = OFF; // turn ON watchdog timer
    
    GREEN_LED = 1;
    AMBER_LED = 1;
    
    WriteSTLED316SData(VERSION, 0xFF);
    __delay_ms(500);
    AD_capture_BattVoltage();   
 /****************************************************************************
               Read Parameters
******************************************************************************/
    
     /*************************************************************************
               Read Vibration Mode
    **************************************************************************/
    INTCONbits.GIE=0;
    ETemp = read_i2c(EEPROM_VibMode);
    INTCONbits.GIE=1;
    
    vibration_mode = ETemp & 0x00FF;
    if(vibration_mode>1)
    {
        vibration_mode = 1;
        
        INTCONbits.GIE=0;
        write_i2c(EEPROM_VibMode, vibration_mode);
        INTCONbits.GIE=1;
    }
    
    NUM = 1;
    WriteSTLED316SData(NUM, vibration_mode);
    NUM_REC = 1;
    
    /*************************************************************************
               Read Device ID
    **************************************************************************/
    INTCONbits.GIE=0;
    ETemp = read_i2c(EEPROM_DevID);
    INTCONbits.GIE=1;
    
    Device_ID = ETemp & 0x00FF;
    if(Device_ID<0x31 || Device_ID>0x3A)
    {
        Device_ID=0x31;
        
        INTCONbits.GIE=0;
        write_i2c(EEPROM_DevID, Device_ID);
        INTCONbits.GIE=1;
    }
    
    /*************************************************************************
               Read Motor Pause Time
    **************************************************************************/
    INTCONbits.GIE=0;
    ETemp = read_i2c(EEPROM_MotorPauseTime);
    INTCONbits.GIE=1;
    
    pause_Time = ETemp & 0x00FF;
    if(pause_Time<0x30 || pause_Time>0x35)
    {
        pause_Time = 0x30;
        Motor_Pause_Time=0;
        
        INTCONbits.GIE=0;
        write_i2c(EEPROM_MotorPauseTime, pause_Time);
        INTCONbits.GIE=1;
    }
    else
    {
        switch(pause_Time)
        {
            case 0x30:
            default:
                Motor_Pause_Time = 0;
                pause_Time = 0x30;
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
    
    /*************************************************************************
               Read Motor Stop Position
    **************************************************************************/
    INTCONbits.GIE=0;
    ETemp = read_i2c(EEPROM_MotorStopPosition);
    INTCONbits.GIE=1;
    
    delay_motor_stop_time = ETemp & 0x00FF;
    if(delay_motor_stop_time>0x96)
    {
        delay_motor_stop_time=0x00;
        
        INTCONbits.GIE=0;
        write_i2c(EEPROM_MotorStopPosition, delay_motor_stop_time);
        INTCONbits.GIE=1;
        
        Motor_Stop_Delay_Time=0;
    }
    else
    {
        Motor_Stop_Delay_Time = delay_motor_stop_time;             
    }
    
    /*************************************************************************
               Read Motor Vibration Time
    **************************************************************************/
    INTCONbits.GIE=0;
    ETemp = read_i2c(EEPROM_VibTime);
    INTCONbits.GIE=1
            ;
    vib_Time = ETemp & 0x00FF;   
    if( (vib_Time!=one_sec && vib_Time!=two_sec && vib_Time!=three_sec && vib_Time!=four_sec && vib_Time!=five_sec) )
    {
        Vmotor_Time = 2000;      // default is 2 sec
        vib_Time = two_sec;
        
        INTCONbits.GIE=0;
        write_i2c(EEPROM_VibTime,vib_Time);
        INTCONbits.GIE=1;
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
    
    INTCONbits.GIE=0; 
    ETemp = read_i2c(EEPROM_MotorSpeed);
    INTCONbits.GIE=1;
    
    PWM_reg = ETemp & 0x00FF;
    
    if( (PWM_reg!=FAST && PWM_reg!=NORMAL && PWM_reg!=SLOW) )
    {
        PWM_reg=NORMAL;
        
        INTCONbits.GIE=0;
        write_i2c(EEPROM_MotorSpeed,PWM_reg);
        INTCONbits.GIE=1;
    }
    
    errorcounter = errorTime0;
    MotorPosition_Init();
    AMBER_LED = 0;
    
 /****************************************************************************
                While(1) loop
******************************************************************************/    
    while(1)
    {
        ClrWdt();
        errorcounter = errorTime0;
        AD_capture_BattVoltage();
        
        switch(OpMode)
        {
             /****************************************************************
              Manual Operation mode
            *****************************************************************/
            case MANUAL_MODE:
                
                NUM = NUM_REC;

                if (CENTER == 0) 
                {
                    do
                    {                                              
                        if(MOTOR_ON_BUT == 0)
                        {
                            ToggleVIB_Mode();                           
                        }
                        
                        WriteSTLED316SVibMode(vibration_mode);
                         __delay_ms(100);
                    
                    }while (CENTER == 0);
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

                if (MOTOR_ON_BUT == 0) //MOTOR_ON_BUT
                {
                    Busy = 1;
                    errorcounter = errorTime0;
                    Homing_Again_Manual();
                    Stop = 0;
                    Busy = 0;

                   // do
                   // {
                       // WriteSTLED316SErr('E');
                   // }
                   // while (!MOTOR_ON_BUT); //Loop until the pushbutton release
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
                                
                                errorcounter = errorTime0;                              
                                Homing_Again_Auto();
                                
                            }
                            else if(Serial_Buffer[2] == 0xF2 && Busy == 0) //semi auto
                           {
                                Stop = 0;
                                Busy = 1;
                                NUM = NUM_REC;
                                WriteSTLED316SData(NUM, vibration_mode);
                                
                                errorcounter = errorTime0;
                                Homing_Again_Manual();
                                
                                //send semi-auto dispense completed command to PC
                                if(Stop == 0)
                                {
                                    Serial_Buffer_Out[0] = Serial_SOT;
                                    Serial_Buffer_Out[1] = 0x44;
                                    Serial_Buffer_Out[2] = 0xF9;
                                    Serial_Buffer_Out[3] = 0X3D;
                                    Serial_Buffer_Out[4] = Serial_EOT;
                                    
                                    INTCONbits.GIE=0;
                                    for (i=0; i<5;i++)
                                    {
                                        Write1USART(Serial_Buffer_Out[i]);
                                    }
                                    INTCONbits.GIE=1;
                                }
                                
                            }
                                flush();
                                flushOut();
                                Stop = 0;
                                Busy = 0;    
                                break;
                            
                            
                        case 0x23: //program Pause time
                            
                            if(Busy==0)
                            {
                                Busy = 1;
                                //if(Serial_Buffer[2]>=0x30 && Serial_Buffer[2]<=0x35)
                                //{
                                    //Device_ID = Serial_Buffer[2];
                                    pause_Time = Serial_Buffer[2];
                                    switch(pause_Time)
                                    {
                                        case 0x30:
                                        default:
                                            Motor_Pause_Time = 0;
                                            pause_Time = 0x30;
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
                                //}
                                //write_i2c(EEPROM_DevID, Device_ID);
                                    
                                INTCONbits.GIE=0;
                                write_i2c(EEPROM_MotorPauseTime, pause_Time);
                                INTCONbits.GIE=1;
                                
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
                                    
                                    INTCONbits.GIE=0;
                                    pause_Time = read_i2c(EEPROM_MotorPauseTime);
                                    vib_Time = read_i2c(EEPROM_VibTime);
                                    Motor_Speed = read_i2c(EEPROM_MotorSpeed);
                                    delay_motor_stop_time = read_i2c(EEPROM_MotorStopPosition);
                                    INTCONbits.GIE=1;

                                    Serial_Buffer_Out[0] = 0x51;
                                    Serial_Buffer_Out[1] = pause_Time;
                                    Serial_Buffer_Out[2] = Motor_Speed;
                                    Serial_Buffer_Out[3] = vib_Time;
                                    Serial_Buffer_Out[4] = delay_motor_stop_time;
                                    
                                    __delay_ms(100);
                                    
                                    INTCONbits.GIE=0;
                                    for (i=0; i<5;i++)
                                    {
                                        Write1USART(Serial_Buffer_Out[i]);
                                    }
                                    INTCONbits.GIE=1;
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
                                    INTCONbits.GIE=0;
                                    write_i2c(EEPROM_MotorSpeed,PWM_reg);
                                    INTCONbits.GIE=1;
                                    
                                    Busy = 0;
                            }
                            break;
                            
                        case 0x65: //program motor vibration time
                            
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
                                    INTCONbits.GIE=0;
                                    write_i2c(EEPROM_VibTime,vib_Time);
                                    INTCONbits.GIE=1;
                                    
                                    Busy = 0;
                            }
                            
                        case 0x66: //program motor stop position
                            
                            if(Busy == 0)
                            {
                                Busy = 1;
                                delay_motor_stop_time = Serial_Buffer[2];
                                Motor_Stop_Delay_Time = delay_motor_stop_time; 
                                
                                INTCONbits.GIE=0;
                                write_i2c(EEPROM_MotorStopPosition,delay_motor_stop_time);
                                INTCONbits.GIE=1;
                                
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
                            INTCONbits.GIE=0;
                            for (i=0; i<5; i++)
                            {
                                Write1USART(Serial_Buffer[i]);
                            }
                            INTCONbits.GIE=1;
                        }
                        else
                        {
                            INTCONbits.GIE=0;
                            for (i=0; i<5; i++)
                            {
                                Write1USART(CMD_BUSY);
                            }
                            INTCONbits.GIE=1;
                        }

                        Serial_Flag = 1;
                        Serial_Count = 0;
                        OpMode = AUTO_MODE;

                    }
                    else
                    {
                        Serial_Flag = 0;
                        Serial_Count = 0;
                        
                        INTCONbits.GIE=0;
                        for (i=0; i<5; i++)
                        {
                            Write1USART(NAK);
                        }
                        INTCONbits.GIE=1;
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
    
    if (TMR1IF_triggered == true)
    {
        if(errorcounter >0)
        {
            errorcounter--;
        }
        else 
        {
            errorcounter = 0;
        }
        
        if(LowPowerCounter >0)
        {
            LowPowerCounter--;
        }
        else
        {
            LowPowerCounter = Time0;
            Low_Power_Indicator();
        }
        
        TMR1IF_triggered = false;
    }
    
    if(TMR1IF)
    { 
        TMR1 = 0x9E57; //100ms timer1
        TMR1IF = 0;
        TMR1IF_triggered = true;
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
Function:		IR Sensor Function
******************************************************************************/
unsigned int Read_IR(void) 
{
    //IRFlag black is 1 white is 0
    if(IR_Status == 1)//white
    {
        return 0; 
    } 
    else //black
    {
        return 1;
    }
    
}

/****************************************************************************
Function:		Initialize Motor startup position
******************************************************************************/
void MotorPosition_Init(void)
{
    IR_ON = 1;
    MotorON_PWM(); // turn ON motor
    __delay_ms(350); //default 350
    errorcounter = errorTime0;
        
    do
    {
      IR_SENSORF =  Read_IR();
      if(errorcounter == 0)
      {
          WriteSTLED316SErr('1');
          MotorBREAK();
      }
       
    }while(IR_SENSORF != 0);
    
     __delay_ms(30);
    errorcounter = errorTime0;    
    do
    {
       IR_SENSORF =  Read_IR();
       if(errorcounter == 0)
       {
            WriteSTLED316SErr('2');
           MotorBREAK();
       }
       
    }while(IR_SENSORF != 1);
    
    errorcounter = errorTime0;
    
    delay_1ms(Motor_Stop_Delay_Time);
    MotorBREAK();
    __delay_ms(500);
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
    
    //if (vibration_mode)
        //RED_LED = 1;
    //else
        //RED_LED = 0;
    
    INTCONbits.GIE=0;
    write_i2c(EEPROM_VibMode, vibration_mode);
    INTCONbits.GIE=1;

    //WriteSTLED316SData(NUM, vibration_mode); //Update the display
}

/****************************************************************************
Function:		Homing for Manual Mode
******************************************************************************/
void Homing_Again_Manual(void) 
{
  
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
        AD_capture_BattVoltage();
        delay_1ms(Motor_Pause_Time);
        MotorON_PWM(); // Turn ON motor
        __delay_ms(350); //default 350
        errorcounter = errorTime0;

        do
        {
            IR_SENSORF = Read_IR();
            if(errorcounter == 0)
            {
                WriteSTLED316SErr('1');
                MotorBREAK();
            }
           
        }while(IR_SENSORF != 0);
        
         __delay_ms(30);
         
         errorcounter = errorTime0;
         
        do
        {
            IR_SENSORF = Read_IR();
            if(errorcounter == 0)
            {
                WriteSTLED316SErr('2');
                MotorBREAK();
            }
        }
        while(IR_SENSORF != 1);
        
        errorcounter = errorTime0;
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
        AD_capture_BattVoltage();
        delay_1ms(Motor_Pause_Time);
        MotorON_PWM(); // Turn ON motor
        __delay_ms(350); //default350
        
        errorcounter = errorTime0;

       do
        {
          IR_SENSORF =   Read_IR();
          if(errorcounter == 0)
          {
              WriteSTLED316SErr('1');
              MotorBREAK();
          }
        }while(IR_SENSORF != 0);
        
         __delay_ms(30);
         errorcounter = errorTime0;
        
        do
        {
          IR_SENSORF =   Read_IR();
          if(errorcounter == 0)
          {
              WriteSTLED316SErr('2');
              MotorBREAK();
          }
        }while(IR_SENSORF != 1);
        
        errorcounter = errorTime0;

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
    
    INTCONbits.GIE=0;
    for (i=0; i<5;i++)
    {
        Write1USART(Serial_Buffer_Out[i]);
    }
    INTCONbits.GIE=1;
    
    flushOut();
}

void InitTimer1(void)
{
	T1CON	= 0b00110001;   //prescaler is 8 default is 4tmr
	//TMR1H	= T1_ValueH;	//
	//TMR1L	= T1_ValueL;	//
    TMR1 = 0x9E57; //100ms
	TMR1IF	= 0;	
	TMR1IE	= 1;
    IPR1bits.TMR1IP=1;
    
    TMR1IF_triggered = false;
}

// Check Battery Voltage Level
void AD_capture_BattVoltage(void)
{
	//select channel AN0 and start conversion
    ADCON0bits.CHS0=0;
    ADCON0bits.CHS1=0;
    ADCON0bits.CHS2=0;
    ADCON0bits.CHS3=0;
    
	ADCON0bits.ADON=1;// ON adc
	delay_1ms(5);
	ADCON0bits.GO=1;

	while(ADCON0bits.GO==1){;}
    //VOLTAGE=ADRES;
    VOLTAGE = (ADRESH*256) | (ADRESL);
}

void Low_Power_Indicator(void)
{
    //max 0x0400
    if(VOLTAGE <= 0x0366)//2.7v(0x0346) --> 6.5V //6.1V cut off
    {
                
        AMBER_LED=1;
        
    }
    else if(VOLTAGE >= 0x03C0) // 3.1v --> 7.5V
    {
    	AMBER_LED=0;
    }
    
}