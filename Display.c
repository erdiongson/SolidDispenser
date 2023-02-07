/*
 * File:   Display.c
 * Author: Eric Liu
 *
 * Created on May 18, 2020, 10:32 AM
 */


#include <xc.h>
#include "IO.h"
#include "main.h"
#include "Led_Display.h"

void InitSTLED316(unsigned char Brightness) //default 0x77
{
    int i;

    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(CHIP_CONFIG_ADDRESS + CHIP_CONFIG_PAGE + FIX_Address + WRITE_Command);
    //    0x0                  0x10                 0x20            0 

    STB_Set;

    STB_Clr;


    STLED316s_SPI_SendData(CHIP_CONFIG_ADDRESS + CHIP_CONFIG_PAGE + FIX_Address + WRITE_Command);
    //    0x0                  0x10                 0x20            0 
    STLED316s_SPI_SendData(((Brightness >> 4) << 5) | 0x00 | 0x5);

    STB_Set;

    STB_Clr;


    STLED316s_SPI_SendData(DIGIT_BRIGHTNESS_PAGE + DIGIT_BRIGHTNESS_ADDRESS + INC_Address + WRITE_Command);
    //   0x10			  	  1                           0                  0	   


    /**********************************************************************************************************/

    STB_Set;

    STB_Clr;


    STLED316s_SPI_SendData(DIGIT_BRIGHTNESS_PAGE + DIGIT_BRIGHTNESS_ADDRESS + INC_Address + WRITE_Command);
    //   0x10		    1                           0                  0	   
    STLED316s_SPI_SendData(Brightness);
    STLED316s_SPI_SendData(Brightness);
    STLED316s_SPI_SendData(Brightness);

    /**********************************************************************************************************/

    STB_Set;

    STB_Clr;

    /* LED灯亮度设置 */
    STLED316s_SPI_SendData(LED_BRIGHTNESS_PAGE + LED_BRIGHTNESS_ADDRESS + INC_Address + WRITE_Command);
    //   0x11			  0                           0                  0	   

    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(LED_BRIGHTNESS_PAGE + LED_BRIGHTNESS_ADDRESS + INC_Address + WRITE_Command);
    //   0x11		  0                           0                  0	   
    STLED316s_SPI_SendData(Brightness);
    STLED316s_SPI_SendData(Brightness);
    STLED316s_SPI_SendData(Brightness);
    STLED316s_SPI_SendData(Brightness);

    /*********************************************************************************************************/

    STB_Set;
}

void STLED316s_Delay(void) {
    //	char i;
    // 	
    // 	for( i = 0 ; i < 10 ; i++ );
}

void STLED316s_SPI_SendData(unsigned char Data) {
    char i, temp, t;

    for (i = 0; i < 8; i++) {
        CLK_Clr; //SCL=0;		
        temp = Data & (0x1 << i);
        if (temp)
            DATA_Set; //SDATA=1
        else
            DATA_CLr; //SDATA=0
        for (t = 0; t < 5; t++);
        CLK_Set;
        for (t = 0; t < 5; t++);
    }
}

unsigned char Get7Seg(int Digit) {
    unsigned char Data;

    switch (Digit) {
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
} //

void WriteSTLED316SData(int number, char v_mode) {
    unsigned char data3, data4;
    int NUM_DIG0, NUM_DIG1;

    NUM_DIG0 = (int) number % 10;
    NUM_DIG1 = (int) number / 10;

    STB_Clr;

    STLED316s_SPI_SendData(0x00);

    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(0x00);

    data4 = Get7Seg(NUM_DIG1);
    data3 = Get7Seg(NUM_DIG0);

    if (v_mode) {
        if (v_mode == 0xFF)
            data4 |= Seg_Dot; //display the dot	for version dot
        else
            data3 |= Seg_Dot; //display the dot
    }

    STLED316s_SPI_SendData(data4);
    STLED316s_SPI_SendData(data3);

    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(DISPLAY_ON); /* 发送打开显示命令 */

    STB_Set;
}

void WriteSTLED316SMode(char msg) {
    STB_Clr;

    STLED316s_SPI_SendData(0x00);


    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(0x00);

    switch (msg) {
        case 'E': //error start button not release
            STLED316s_SPI_SendData(Seg_E);
            STLED316s_SPI_SendData(Seg_0);
            break;
        case 'O':
            STLED316s_SPI_SendData(Seg_0); //0
            STLED316s_SPI_SendData(Seg_P); //P
            break;
        case 'C':
            STLED316s_SPI_SendData(Seg_C); //C
            STLED316s_SPI_SendData(Seg_A); //A
            break;
        case 'X':
            STLED316s_SPI_SendData(Seg_Blank); //0
            STLED316s_SPI_SendData(Seg_Blank); //0
            break;
        case 'G':
            STLED316s_SPI_SendData(Seg_Blank); //0
            STLED316s_SPI_SendData(Seg_0); //0
            break;
        case 'S':
            STLED316s_SPI_SendData(Seg_E); //0
            STLED316s_SPI_SendData(Seg_1); //0
            break;
        case 'A': //Auto - continuous mode    
            STLED316s_SPI_SendData(Seg_A); //0
            STLED316s_SPI_SendData(Seg_U); //0
            break;
        case 'V': //vibration mode setting 
            STLED316s_SPI_SendData(Seg_Blank); //0
            STLED316s_SPI_SendData(Seg_U); //0
            break;
        default:
            break;
    }



    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(DISPLAY_ON);

    STB_Set;
}

void WriteSTLED316SVibMode(char v_mode) {

    STB_Clr;

    STLED316s_SPI_SendData(0x00);


    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(0x00);

    //STLED316s_SPI_SendData (Seg_Blank); //0
    //STLED316s_SPI_SendData (Seg_U); //0


    if (v_mode == 1) {
        STLED316s_SPI_SendData(Seg_Blank); //0
        STLED316s_SPI_SendData(Seg_Udot); //0
    } else {
        STLED316s_SPI_SendData(Seg_Blank); //0
        STLED316s_SPI_SendData(Seg_U); //0
    }

    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(DISPLAY_ON);

    STB_Set;
}

void WriteSTLED316SErr(char msg) {
    STB_Clr;

    STLED316s_SPI_SendData(0x00);


    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(0x00);

    switch (msg) {
        case 'E': //error start button not release
            STLED316s_SPI_SendData(Seg_E);
            STLED316s_SPI_SendData(Seg_A);
            break;
        case '0':
            STLED316s_SPI_SendData(Seg_E);
            STLED316s_SPI_SendData(Seg_0);
            break;
        case '1':
            STLED316s_SPI_SendData(Seg_E);
            STLED316s_SPI_SendData(Seg_1);
            break;
        case '2':
            STLED316s_SPI_SendData(Seg_E);
            STLED316s_SPI_SendData(Seg_2);
            break;
        case 'G':
            STLED316s_SPI_SendData(Seg_Blank); //0
            STLED316s_SPI_SendData(Seg_0); //0
            break;
        case 'S':
            STLED316s_SPI_SendData(Seg_E); //0
            STLED316s_SPI_SendData(Seg_1); //0
            break;
        case 'A': //Auto - continuous mode    
            STLED316s_SPI_SendData(Seg_A); //0
            STLED316s_SPI_SendData(Seg_U); //0
            break;
        default:
            break;
    }

    STB_Set;

    STB_Clr;

    STLED316s_SPI_SendData(DISPLAY_ON);

    STB_Set;
}
