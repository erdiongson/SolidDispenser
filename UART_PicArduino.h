#include "main.h"
#include <xc.h>
#include <stdio.h>
#include <string.h>


/*/ / Calculate baud rate generator
BRG = (_XTAL_FREQ - baudrate * 16) / (baudrate * 16);
BRG = (8000000 - 9600 * 16) / (9600 * 16);
BRG = (8000000 - 153600) / 153600;

BRG = 468.75

        baud rate = _XTAL_FREQ / (16 * (SPBRG + 1))
baud rate = 8000000 / (16 * (468 + 1))
baud rate = 8000000 / (16 * 469)
baud rate = 9600
 */

#define _XTAL_FREQ 8000000 // Define crystal frequency
#define BAUD 19200 // Define baud rate
#define BRG ((_XTAL_FREQ/16/BAUD) - 1) //Formula for high speed calculation if baud rate is greater than 255

//Enum added by Leo - 05May23

typedef enum {
    Handshake = 0x06,
    Vibrate_Mode_ON = 0xC1,
    Vibrate_Mode_OFF = 0xC2,
    SDB_Dispense_START = 0xC3,
    SDB_Dispense_PAUSE = 0xC4,
    SDB_Dispense_STOP = 0xC5,
    IR_Censor_Failure = 0xE1,
    Marker_Not_Detected = 0xE2
} Command;

typedef enum {
    WAIT_FOR_START,
    RECEIVE_BYTES,
    PROCESS_COMMAND
} uart_state_t;

//Added by Leo from original(can be remove later)
void initUSART(void);
void Write1USART(char data);
char Read1USART(void);
void Write2USART(char data);
char Read2USART(void);

//Function prototype
void uart_config(unsigned int uart_num);
void uart_send(unsigned int uart_num, unsigned char data);
unsigned char uart_receive(unsigned int uart_num);
unsigned char uart_receive_err(unsigned int uart_num);
void uart_send_char(unsigned int uart_num, char data);
void uart_send_char_err(unsigned int uart_num, char data);
char uart_receive_char(unsigned int uart_num);
char uart_receive_char_err(unsigned int uart_num);
void uart_send_string(unsigned int uart_num, const char *data);
void uart_send_string_err(unsigned int uart_num, const char *data);
void uart_receive_string(unsigned int uart_num, char *data);
void uart_receive_string_err(unsigned int uart_num, char *data);

//Added by Leo - 05May23
void uart_print(const char *str);
void uart_println(unsigned char num);
//void sendResponse(unsigned char response, unsigned char data1, unsigned char data2);
void sendResponse(unsigned char sot, unsigned char response, unsigned char data1, unsigned char data2, unsigned char eot);
void sendData(unsigned char *data, int length, unsigned char uart_channel);
void uart_print(const char *str);
void uart_println(unsigned char num);
void uart_print_hex(unsigned char num);
void print_received_block(unsigned char uart_channel, unsigned char *data, unsigned int size);
void handle_uart_communication(unsigned int Motor_Stop_Delay_Time, volatile long errorcounter);
unsigned char receiveData(unsigned char uart_channel);
void delay2_1ms(unsigned int time);

///******************************************************************************
// * PWM Duty Cycle Selection
// ******************************************************************************/
//#define zero_percent       0x00
//#define twentyfive_percent 0x01
//#define fifty_percent      0x02
//#define hundred_percent    0x03
///******************************************************************************
// 