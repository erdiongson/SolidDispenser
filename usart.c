/*
 * File:   usart.c
 * Author: Eric Liu
 * 
 *
 * Created on May 18, 2020, 10:48 AM
 * 
 * Created: 2023-04-19, 12:00PM
 * Author: erdiongson
 * Modified: 07-May-2023 //LA
 * Version 3.7_1R : i. Created version dedicated for SDB1R product
 *                  ii. Added UART2 for communication with Arduino AtMega
 */
#include <xc.h>
#include "IO.h"
#include "main.h"

//Added by Leo
#include "UART_PicArduino.h"
#include <stdio.h>
#include <stdbool.h>


#define zero_percent       0x00
#define seventyfive_percent 0x03
#define hundred_percent    0x04


#define errorTime0  30 //IR sensor

/****************************************************************************
Function:		USART Initialization
 ******************************************************************************/
void initUSART(void) {
    // setup USART
    TRISCbits.TRISC6 = 0; // TX1 as output
    TRISCbits.TRISC7 = 1; // RX1 as input

    TXSTA1bits.SYNC = 0; // Async operation
    TXSTA1bits.TX9 = 0; // No tx of 9th bit
    TXSTA1bits.TXEN = 1; // Enable transmitter
    RCSTA1bits.RX9 = 0; // No rx of 9th bit
    RCSTA1bits.CREN = 1; // Enable receiver
    TXSTA1bits.CSRC = 0; //Clock Source Select bit for syn - Asyn is dont care
    RCSTA1bits.ADDEN = 1;
    RCSTA1bits.SPEN = 1; // Enable serial port

    //20230419 (SDB1R) : erdiongson - Added TX2 and RX2 for Arduino -- START --
    TRISGbits.TRISG1 = 0; // TX2 as output
    TRISGbits.TRISG2 = 1; // RX2 as input
    TXSTA2bits.SYNC = 0; // Async operation
    TXSTA2bits.TX9 = 0; // No TX of 9th bit
    TXSTA2bits.TXEN = 1; // Enable transmitter
    RCSTA2bits.RX9 = 0; // No RX of 9th bit
    RCSTA2bits.CREN = 1; // Continuous Receive Enable
    TXSTA2bits.CSRC = 0; // Clock Source Select bit for syn - Asyn is dont care
    RCSTA2bits.ADDEN = 1; // Address Detect Enable
    RCSTA2bits.SPEN = 1; // Enable serial port
    //20230419 (SDB1R) : erdiongson - Added TX2 and RX2 for Arduino -- END --

    // Setting for 19200 BPS
    BAUDCON1bits.BRG16 = 0; // Divisor at 8 bit
    TXSTA1bits.BRGH = 1; // No high-speed baudrate
    PIE1bits.RC1IE = 1; // EUSART1 Receive Interrupt Enable Bit
    PIE1bits.TX1IE = 0; // EUSART1 Transmit Interrupt Enable Bit
    //20230419 : erdiongson - Setting for 19200 -- START --
    BAUDCON2bits.BRG16 = 0; // Divisor at 8 bit    
    TXSTA2bits.BRGH = 1; // No high-speed baudrate
    PIE3bits.RC2IE = 1; // EUSART2 Receive Interrupt Enable Bit
    PIE3bits.TX2IE = 0; // EUSART2 Transmit Interrupt Enable Bit
    //20230419 : erdiongson - Setting for 19200 -- END --

    //(FOSC/baudrate/64)-1 = SPBRG1
    SPBRG1 = 25; //48MHZ = 38; // divisor value for 19200
    SPBRGH1 = 0;
    //20230419 : erdiongson - (FOSC/baudrate/64)-1 = SPBRG2 -- START --
    SPBRG2 = 25;
    SPBRGH2 = 0;
    //20230419 : erdiongson - (FOSC/baudrate/64)-1 = SPBRG2 -- START --
}

/****************************************************************************
Function:		USART Write Function
 ******************************************************************************/
void Write1USART(char data) {
    TXREG1 = data; // Write the data byte to the USART2

    while (Busy1USART());
} // end Write1USART

/****************************************************************************
Function:		USART Read Function
 ******************************************************************************/
char Read1USART(void) {
    char result;

    if (RCSTA1bits.OERR) { //has there been an overrun error?
        RCSTA1bits.CREN = 0; //disable Rx to clear error
        result = RCREG1; //purge receive buffer
        result = RCREG1; //purge receive buffer
        RCSTA1bits.CREN = 1; //re-enable Rx
    } else {
        result = RCREG1;
    }

    return result;
}

/****************************************************************************
Function:		USART Write Function 2
Created:        Apr 19, 2023
Author:         Englebert Diongson
 ******************************************************************************/
void Write2USART(char data) {
    TXREG2 = data; // Write the data byte to the USART2

    while (Busy2USART());
} // end Write1USART

/****************************************************************************
Function:		USART Read Function 2
Created:        Apr 19, 2023
Author:         Englebert Diongson
 ******************************************************************************/
char Read2USART(void) {
    char result;

    //if (RCSTA2bits.OERR) { //has there been an overrun error?
    //RCSTA2bits.CREN = 0; //disable Rx to clear error
    //result = RCREG2; //purge receive buffer
    while (!DataRdy2USART());
    result = RCREG2; //purge receive buffer
    //RCSTA2bits.CREN = 1; //re-enable Rx
    //} else {
    //result = RCREG2;
    //}

    return result;
}

//Added by Leo - 05-May-23

void uart_print(const char *str) {
    while (*str) {
        while (!TX1IF);
        TXREG1 = *str;
        str++;
    }
}

void uart_println(unsigned char num) {
    char buf[5];
    sprintf(buf, "%d", num);
    uart_print(buf);
    uart_print("\r\n");
}

void uart_print_hex(unsigned char num) {
    char buf[5];
    sprintf(buf, "0x%02X", num);
    uart_print(buf);
}

// New function to print the received block of data with  UART1(for testing only)

void print_received_block(unsigned char uart_channel, unsigned char *data, unsigned int size) {
    uart_print("Received data (UART");
    uart_print(uart_channel);
    uart_print("): ");
    for (unsigned int i = 0; i < size; i++) {
        uart_print_hex(data[i]);
        if (i < (size - 1)) {
            uart_print(", ");
        }
    }
    uart_print("\r\n");
}

//void sendData(unsigned char *data, int length, unsigned char uart_channel) {
//    for (int i = 0; i < length; i++) {
//        if (uart_channel == 1) {
//            while (!TX1IF);
//            TXREG1 = data[i];
//            uart_print("Sent data (UART1): ");
//            uart_println(data[i]);
//        } else if (uart_channel == 2) {
//            while (!TX2IF);
//            TXREG2 = data[i];
//        }
//    }
//}

void sendData(unsigned char *data, int length, unsigned char uart_channel) {
    for (int i = 0; i < length; i++) {
        if (uart_channel == 1) {
            while (!TX1IF);
            TXREG1 = data[i];
            uart_print("Sent response (UART1): ");
            uart_print_hex(data[i]);
            if (i < (length - 1)) {
                uart_print(", ");
            } else {
                uart_print("\r\n");
            }
        } else if (uart_channel == 2) {
            while (!TX2IF);
            TXREG2 = data[i];
        }
    }
}

unsigned char receiveData(unsigned char uart_channel) {
    unsigned char receivedData;
    if (uart_channel == 1) {
        while (!PIR1bits.RC1IF);
        receivedData = RCREG1;
    } else if (uart_channel == 2) {
        while (!PIR3bits.RC2IF);
        receivedData = RCREG2;
    }
    return receivedData; // Add this line to return the received data
}

void sendResponse(unsigned char sot, unsigned char response, unsigned char data1, unsigned char data2, unsigned char eot) {
    unsigned char responseData[5] = {sot, response, data1, data2, eot};

    //        sendData(responseData, 5, 1);
    //        uart_print("\r\n"); // Add a newline after sending data to the Serial Monitor
    //
    //    __delay_ms(100);
    sendData(responseData, 5, 2);


}

void delay2_1ms(unsigned int time) {
    while (time > 0) {
        __delay_ms(1);
        time--;
    }
}

void handle_uart_communication(unsigned int Motor_Stop_Delay_Time, volatile long errorcounter) {

    unsigned int IR_SENSORF = 0;

    IR_ON = 1; //turn ON IR sensor

    //volatile long errorcounter = errorTime0; //IRsensor counter

    while (1) {
        unsigned char receivedBytes[5];

        // Wait for start of transmission byte
        while ((receivedBytes[0] = receiveData(2)) != 0xA5);

        // Receive the remaining bytes
        for (int i = 1; i < 5; i++) {
            receivedBytes[i] = receiveData(2);
        }

        // Print the received data in one line as a block
        //        print_received_block(2, receivedBytes, 5);

        unsigned char command = receivedBytes[1];
        unsigned char data1 = receivedBytes[2];
        unsigned char data2 = receivedBytes[3];
        unsigned char eot = receivedBytes[4];

        if (receivedBytes[0] == 0xA5 && eot == 0x5A) {
            // Process the command and send the appropriate response
            switch (command) {
                case Handshake: // Handshake
                    sendResponse(0xA5, 0x60, 0x00, 0x00, 0x5A); // Acknowledge
                    break;
                case Vibrate_Mode_ON:
                    //to do: logic of powder dispenser
                    //                    vibration_mode == 1;

                    VIB_MOTOR_ON = 1;

                    // Turn the motor on with a duty cycle of 100 percent
                    //                    PWM1_Start();
                    vibrationMotorControl(true, seventyfive_percent); // Assuming fifty_percent is defined appropriately
                    //                    MotorON();
                    sendResponse(0xA5, 0x60, 0x1C, 0x00, 0x5A); // Vibration_Turned_ON
                    break;
                case Vibrate_Mode_OFF:
                    // to do: logic of powder dispenser
                    //                    vibration_mode == 0;
                    VIB_MOTOR_ON = 0;
                    //                    PWM1_Stop();
                    // When you need to turn off the motor
                    vibrationMotorControl(false, zero_percent); // PWM message may not matter here as the motor is being turned off
                    //                    MotorBREAK();
                    sendResponse(0xA5, 0x60, 0x2C, 0x00, 0x5A); // Vibration_Turned_OFF
                    break;
                case SDB_Dispense_START:
                    // Implement SDB dispense start
                    //to do: logic of powder dispenser
                    //                    Homing_Again_Auto(); // Turn ON motor
                    IR_ON = 1;
                    MotorON();
                    //                    Homing_Again_Manual; //Adding this will stop the Arduino comms
                    //                    __delay_ms(6000);
                    __delay_ms(150); //20230922 - ediongson change in rotation
                    errorcounter = errorTime0;
                    do {
                        IR_SENSORF = Read_IR();
                        if (errorcounter == 0) {
                            MotorBREAK();
                        }
                    } while (IR_SENSORF != 0);   
                     __delay_ms(30);
                    errorcounter = errorTime0;
                    
                    do {
                        IR_SENSORF = Read_IR();
                        if (errorcounter == 0) {
                            MotorBREAK();
                        }
                    } while (IR_SENSORF != 1);
                    errorcounter = errorTime0;
                    delay2_1ms(Motor_Stop_Delay_Time);
                    
                    MotorBREAK();                    
                    sendResponse(0xA5, 0x60, 0x3C, 0x00, 0x5A); // Dispense_DONE
                    break;
                case SDB_Dispense_PAUSE:
                    // Implement SDB dispense pause
                    //to do: logic of powder dispenser
                    MotorBREAK();
                    sendResponse(0xA5, 0x60, 0x3C, 0x00, 0x5A); // Dispense_DONE
                    break;
                case SDB_Dispense_STOP:
                    // Implement SDB dispense stop
                    //to do: logic of powder dispenser
                    MotorBREAK();
                    sendResponse(0xA5, 0x60, 0x3C, 0x00, 0x5A); // Dispense_DONE
                    break;
                case IR_Censor_Failure:

                    do {
                        IR_SENSORF = Read_IR();
                        if (errorcounter == 0) {
                            sendResponse(0xA5, 0x60, 0xE1, 0x00, 0x5A); // Error 1: IR Censor Failure
                            MotorBREAK();
                        }
                    } while (IR_SENSORF != 0);
                    break;
                case Marker_Not_Detected:

                    do {
                        IR_SENSORF = Read_IR();
                        if (errorcounter == 0) {
                            sendResponse(0xA5, 0x60, 0xE2, 0x00, 0x5A); // Error 2: Marker not detected
                            MotorBREAK();
                        }
                    } while (IR_SENSORF != 1);
                    break;
                default:
                    // Error handling or sending an error response
                    break;
            }
        }
    }
}

// UART configuration function

void uart_config(unsigned int uart_num) {
    if (uart_num == 1) {
        //TRISCbits.RC7 = 1; // Set RX1 as input
        //TRISCbits.RC6 = 0; // Set TX1 as output

        PIE1bits.RC1IE = 1; // Enable UART1 receive interrupt
        PIE1bits.TX1IE = 1; // Enable UART1 transmit interrupt

        TRISCbits.TRISC7 = 1; // Set RX1 as input
        TRISCbits.TRISC6 = 0; // Set TX1 as output

        SPBRG1 = BRG; // Set baud rate generator

        //TXSTA1bits.CSRC = 0; // Clock Source(bit 7 of TXSTA1 register, don't care for Asynchronous
        TXSTA1bits.TX9 = 0; // 9-bit transmission Enable bit(bit 6 of TXSTA1)
        TXSTA1bits.TXEN = 1; // Transmit Enable bit-Set high(bit 5 of TXSTA1)
        TXSTA1bits.SYNC = 0; // EUSART Mode Select bit(bit 4 of TXSTA1)
        TXSTA1bits.SENDB = 0; // Send Break Character bit(bit 3 of TXSTA1)
        TXSTA1bits.BRGH = 1; // High Baud Rate Select bit-Set high(bit 2 of TXSTA1)
        TXSTA1bits.TRMT = 0; // Transmit Shift Register Status bit(bit 1 of TXSTA1)
        TXSTA1bits.TX9D = 0; // 9th bit of Transmit Data(bit 0 of TXSTA1)
        //TXSTA1 = 0x24;	 // Equivalent hex value of 8-bit TXSTA1 register

        RCSTA1bits.SPEN = 1; // Serial Port Enable bit- Enable UART1(bit 7 of RCSTA1)
        RCSTA1bits.RX9 = 0; // 9-Bit Receive Enable bit - Set low for 8 bit(bit 6 of RCSTA1)
        RCSTA1bits.SREN = 0; // Single Receive Enable bit(bit 5 of RCSTA1)
        RCSTA1bits.CREN = 1; // Continuous Receive Enable bit(bit 4 of RCSTA1)
        RCSTA1bits.ADDEN = 0; // Address Detect Enable bit(bit 3 of RCSTA1), Don't care when RX9 is 0.
        RCSTA1bits.FERR = 0; // Framing Error bit - Disable framing error(bit 2 of RCSTA1)
        RCSTA1bits.FERR = 0; // Overrun Error bit - Disable overrun error(bit 1 of RCSTA1)
        RCSTA1bits.RX9D = 0; //  9th bit of Received Data(bit 0 of RCSTA1)
        //RCSTA1 = 0x90;		  //Equivalent hex value of 8-bits RCSTA1

    } else if (uart_num == 2) {
        //TRISGbits.RG2 = 1; // Set RX2 as input
        //TRISGbits.RG1 = 0; // Set TX2 as output

        PIE3bits.RC2IE = 1; // Enable UART2 receive interrupt
        PIE3bits.TX2IE = 1; // Enable UART2 transmit interrupt

        TRISGbits.TRISG2 = 1; // Set RX1 as input
        TRISGbits.TRISG1 = 0; // Set TX1 as output

        SPBRG2 = BRG; // Set baud rate generator

        //TXSTA2bits.CSRC = 0; // Clock Source(bit 7 of TXSTA2 register, don't care for Asynchronous
        TXSTA2bits.TX9 = 0; // 9-bit transmission Enable bit(bit 6 of TXSTA2)
        TXSTA2bits.TXEN = 1; // Transmit Enable bit-Set high(bit 5 of TXSTA2)
        TXSTA2bits.SYNC = 0; // EUSART Mode Select bit(bit 4 of TXSTA2)
        TXSTA2bits.SENDB = 0; // Send Break Character bit(bit 3 of TXSTA2)
        TXSTA2bits.BRGH = 1; // High Baud Rate Select bit-Set high(bit 2 of TXSTA2)
        TXSTA2bits.TRMT = 0; // Transmit Shift Register Status bit(bit 1 of TXSTA2)
        TXSTA2bits.TX9D = 0; // 9th bit of Transmit Data(bit 0 of TXSTA2)
        //TXSTA2 = 0x24;	 // Equivalent hex value of 8-bit TXSTA2 register

        RCSTA2bits.SPEN = 1; // Serial Port Enable bit- Enable UART1(bit 7 of RCSTA2)
        RCSTA2bits.RX9 = 0; // 9-Bit Receive Enable bit - Set low for 8 bit(bit 6 of RCSTA2)
        RCSTA2bits.SREN = 0; // Single Receive Enable bit(bit 5 of RCSTA2)
        RCSTA2bits.CREN = 1; // Continuous Receive Enable bit(bit 4 of RCSTA2)
        RCSTA2bits.ADDEN = 0; // Address Detect Enable bit(bit 3 of RCSTA2), Don't care when RX9 is 0.
        RCSTA2bits.FERR = 0; // Framing Error bit - Disable framing error(bit 2 of RCSTA2)
        RCSTA2bits.FERR = 0; // Overrun Error bit - Disable overrun error(bit 1 of RCSTA2)
        RCSTA2bits.RX9D = 0; //  9th bit of Received Data(bit 0 of RCSTA2)
        //RCSTA2 = 0x90;		  //Equivalent hex value of 8-bits RCSTA2

    } else {
        // Return error if invalid UART number is passed
        printf("Error: Invalid UART number.\n");
    }
}

// Function to send data through UART

void uart_send(unsigned int uart_num, unsigned char data) {
    if (uart_num == 1) {
        while (!TXSTA1bits.TRMT1);
        TXREG1 = data;
    } else if (uart_num == 2) {
        while (!TXSTA2bits.TRMT2);
        TXREG2 = data;
    } else {
        // Return error if invalid UART number is passed
        printf("Error: Invalid UART number.\n");
    }
}

//Function to receive data through UART

unsigned char uart_receive(unsigned int uart_num) {
    if (uart_num == 1) {
        while (!PIR1bits.RC1IF);
        return RCREG1;
    } else if (uart_num == 2) {
        while (!PIR3bits.RC2IF);
        return RCREG2;
    } else {
        // Return error if invalid UART number is passed
        printf("Error: Invalid UART number.\n");
        return 0;
    }
}

// Function to receive data through UART with error handling

unsigned char uart_receive_err(unsigned int uart_num) {
    if (uart_num == 1) {
        while (!PIR1bits.RC1IF) {
            if (RCSTA1bits.FERR || RCSTA1bits.OERR) {
                // Handle framing error or overrun error
                RCSTA1bits.FERR = 0;
                RCSTA1bits.OERR = 0;
                return 0xFF; // Return a special value to indicate an error has occurred
            }
        }
        return RCREG1;
    } else if (uart_num == 2) {
        while (!PIR3bits.RC2IF) {
            if (RCSTA2bits.FERR || RCSTA2bits.OERR) {
                // Handle framing error or overrun error
                RCSTA2bits.FERR = 0;
                RCSTA2bits.OERR = 0;
                return 0xFF; // Return a special value to indicate an error has occurred
            }
        }
        return RCREG2;
    } else {
        // Return error if invalid UART number is passed
        printf("Error: Invalid UART number.\n");
        return 0;
    }
}

//Function to receive character through

void uart_send_char(unsigned int uart_num, char data) {
    uart_send(uart_num, data);
}

//Function to receive character with error handling

void uart_send_char_err(unsigned int uart_num, char data) {
    if (uart_num == 1 || uart_num == 2) {
        uart_send(uart_num, data);
    } else {
        printf("Error: Invalid UART number.\n");
    }
}

//Function to receive character through UART

char uart_receive_char(unsigned int uart_num) {
    return (char) uart_receive(uart_num);
}

//Function to receive character through UART with error handling

char uart_receive_char_err(unsigned int uart_num) {
    if (uart_num == 1 || uart_num == 2) {
        return (char) uart_receive(uart_num);
    } else {
        printf("Error: Invalid UART number.\n");
        return 0;
    }
}

// Function to send data string through UART

void uart_send_string(unsigned int uart_num, const char *data) {
    int i;
    for (i = 0; i < strlen(data); i++) {
        uart_send(uart_num, data[i]);
    }
}

// Function to send data string through UART with error handling

void uart_send_string_err(unsigned int uart_num, const char *data) {
    if (uart_num == 1 || uart_num == 2) {
        int i;
        for (i = 0; i < strlen(data); i++) {
            uart_send(uart_num, data[i]);
        }
    } else {
        printf("Error: Invalid UART number.\n");
    }
}

// Function to receive data string through UART

void uart_receive_string(unsigned int uart_num, char *data) {
    int i = 0;
    char c = uart_receive(uart_num);
    while (c != '\0') {
        data[i++] = c;
        c = uart_receive(uart_num);
    }
    data[i] = '\0';
}

// Function to receive data string through UART with error handling

void uart_receive_string_err(unsigned int uart_num, char *data) {
    if (uart_num == 1 || uart_num == 2) {
        int i = 0;
        char c = uart_receive(uart_num);
        while (c != '\r') {
            data[i++] = c;
            c = uart_receive(uart_num);
        }
        data[i] = '\0';
    } else {
        printf("Error: Invalid UART number.\n");
    }
}