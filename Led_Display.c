#include "IO.h"
#include "LedDisplay.h"

void STLED316s_Delay(void)
{
//	char i;
// 	
// 	for( i = 0 ; i < 10 ; i++ );
}	// end STLED316s_Delay(void)

/****************************************************************************
Function:		STLED316s_SPI_SendData
Description: 		通过I/O口向STLED316S发送一个字节的数据	
Arguments:		Data 
Returns:		无
Notes:		
******************************************************************************/
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
}	//

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

/****************************************************************************
Function:		WriteSTLED316Strings
Description: 	向STLED316S的7'LED显存写字符串	
Arguments:		unsigned char DataAddress 显存地址, unsigned char *SrcString 字符串首地址,unsigned char nBytes 字符串长
Returns:		无
Notes:			显存地址不能大于5, 否则会溢出, 造成显示错误
******************************************************************************/
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
}	//

//
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
Function:		InitSTLED316
Description: 		初始化STLED316S,主要是对亮度,数码管个数的设置	
Arguments:		unsigned char Brightness 
Returns:		无
Notes:			
******************************************************************************/
void InitSTLED316( unsigned char Brightness )
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
}	//
