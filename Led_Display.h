/*******************************************
 *   7 Segment Program
 * Header file for STLED316 7-Seg LED driver
 *******************************************/

#define Mem_Add_00	0x00	//0b0000 0000
#define Mem_Add_01	0x01	//0b0000 0001
#define Mem_Add_02	0x02	//0b0000 0010
#define Mem_Add_03	0x03	//0b0000 0011
#define Mem_Add_04	0x04	//0b0000 0100
#define Mem_Add_05	0x05	//0b0000 0101
#define Mem_Add_06	0x06	//0b0000 0110
#define Mem_Add_07	0x07	//0b0000 0111

#define Mem_Page_0	0x00	//0b0000 0000
#define Mem_Page_1	0x08	//0b0000 1000
#define Mem_Page_2	0x10	//0b0001 0000
#define Mem_Page_3	0x18	//0b0001 1000

#define INC_Address		0x00	// Default
#define FIX_Address 	0x20
#define WRITE_Command	0x00	// Default
#define READ_Command  	0x40

/*********************************************/
#define CHIP_CONFIG_PAGE 		(0x02 << 3)
#define CHIP_CONFIG_ADDRESS 	0x00
#define CHIP_CONFIGURATION 		0xe5		//0b1110 0101

#define DIGIT_BRIGHTNESS_PAGE 		(0x02 << 3)	//0b0001 0000
#define DIGIT_BRIGHTNESS_ADDRESS 	0x01		//0x1-0x3
#define DIGIT_BRIGHTNESS 			0x77

#define LED_BRIGHTNESS_PAGE 	(0x03 << 3)		//0b0001 1000
#define LED_BRIGHTNESS_ADDRESS 	0x00			//0x1-0X3
#define LED_BRIGHTNESS 			0x77

#define SEGMENT_DATA_PAGE 		(0x00 << 3)		//0b0000 0000
#define SEGMENT1_DATA_ADDRESS 	0x00			//0x0-0x5
#define SEGMENT2_DATA_ADDRESS 	0x01
#define SEGMENT3_DATA_ADDRESS 	0x02
#define SEGMENT4_DATA_ADDRESS 	0x03
#define SEGMENT5_DATA_ADDRESS 	0x04
#define SEGMENT6_DATA_ADDRESS 	0x05

#define LED_DATA_PAGE 		(0x01 << 3)		//0b0000 1000
#define LED_DATA_ADDRESS 	0x00

#define DISPLAY_ON 		0x0D	//0b0000 1101 --- Mem_Page_1+Mem_Add_05
#define DISPLAY_OFF 	0x0E	//0b0000 1110 --- Mem_Page_1+Mem_Add_06

/*********************************************************/
#define SET		1
#define CLR		0

#define STB_Set		SEN = 1
#define STB_Clr		SEN = 0
#define	CLK_Set		SCL = 1
#define	CLK_Clr		SCL = 0
#define	DATA_Set	SDATA = 1
#define	DATA_CLr	SDATA = 0

/*********************************************/
//					  hgfedcba
#define Led_Seg_a	0b00000001
#define Led_Seg_b	0b00000010
#define Led_Seg_c	0b00000100
#define Led_Seg_d	0b00001000
#define Led_Seg_e	0b00010000
#define Led_Seg_f	0b00100000
#define Led_Seg_g	0b01000000
#define Led_Seg_h	0b10000000

//            --- a ---
//           /         /
//          f         b
//         /         /
//         --- g ---
//       /         /
//      e         c
//     /         /
//     --- d ---    h 	    hgfe dcba
#define Seg_La	0x01	//0b0000 0001
#define Seg_Lb	0x02	//0b0000 0010
#define Seg_Lc	0x04	//0b0000 0100
#define Seg_Ld	0x08	//0b0000 1000
#define Seg_Le	0x10	//0b0001 0000
#define Seg_Lf	0x20	//0b0010 0000
#define Seg_Lg	0x40	//0b0100 0000
#define Seg_Lh	0x80	//0b1000 0000

//						    hgfe dcba
#define Seg_0	0x3F	//0b0011 1111
#define Seg_1	0x06	//0b0000 0110
#define Seg_2	0x5B	//0b0101 1011
#define Seg_3 	0x4F	//0b0100 1111
#define Seg_4	0x66	//0b0110 0110
#define Seg_5	0x6D	//0b0110 1100
#define Seg_6	0x7D	//0b0111 1101
#define Seg_7	0x07	//0b0000 0111
#define Seg_8 	0x7F	//0b0111 1111
#define Seg_9	0x6F	//0b0110 1111

#define Seg_A	0x77	//0b0111 0111
#define Seg_b	0x7C	//0b0111 1100
#define Seg_c	0x54	//0b0101 1000	
#define Seg_d	0x5E	//0b0101 1110
#define Seg_E	0x79	//0b0111 1001
#define Seg_F	0x71	//0b0111 0001
#define Seg_U	0x3E	//0b0011 1110
#define Seg_Udot	0xBE	//0b1011 1110
#define Seg_0dot	0xBF	//0b1011 1111
#define Seg_1dot	0x86	//0b1000 0110
#define Seg_2dot	0xDB	//0b1101 1011
#define Seg_3dot 	0xCF	//0b1100 1111
#define Seg_4dot 	0xE6	//0b1110 0110

#define Seg_C	0x39	//0b0011 1001
#define Seg_P	0x73	//0b0111 0011

//	Special code			    hgfe dcba
#define Seg_Blank	0x00	//0b0000 0000
//#define Seg_dC		0x8F	//0b1000 0111
#define Seg_cdeg 	0x5C	//0b0101 1100	
#define Seg_bgfa 	0x63	//0b0110 0011
#define Seg_Dot		0x80	//0b1000 0000

/*********************************************/
//function declaration
/*void STLED316s_Delay (void);
void STLED316s_SPI_SendData ( unsigned char Data );
void WriteSTLED316SData( int number, char v_mode);
void WriteSTLED316SErr( char msg);
void InitSTLED316( unsigned char Brightness );
unsigned char Get7Seg (int Digit);*/
