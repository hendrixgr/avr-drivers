/*******************************************************************************************************
 Title  :   C  file for the HD44780U LCD library (lcd_io.c)
 Author:    Chris efstathiou hendrix@vivodinet.gr
 Date:      6/July/2018
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device
 Comments:  This software is FREE.

LCD MODULE INFORMATION!
       The maximum time required by each instruction for the slowest lcd units is
       4.1 msecs for clearing the display or moving the cursor/display to the "home position",
       160 usecs for all other commands. (i use 50 because i use good units)  

       Usual HD44780 Pins connections 
       1 = Ground 
       2 = Vcc 
       3 = Contrast Voltage 
       4 = "R/S" _Instruction/Register Select 
       5 = "R/W" _Read/Write LCD Registers (See the connection table for additional information). 
       6 = "E" Clock 
       7 - 14 = Data I/O Pins 0 to 7 (0=7, 1=8,... 7=14 )
       15 - Vcc for the BACKLIGHTING (Check to be sure because some units have it for Gnd)
       16 - Gnd for the BACKLIGHTING (Check to be sure because some units have it for Vcc)

TYPICAL CONNECTION TABLE AS USED IN THIS lcd_io.h FILE.
ACTUALLY YOU CAN USE ANY PIN AS LONG AS THE PIN CARRIES THE CORRECT SIGNAL TO THE CORRECT LCD PIN.       

port_pin0  -> lcd_D4_pin11 
port_pin1  -> lcd_D5_pin12 
port_pin2  -> lcd_D6_pin13 
port_pin3  -> lcd_D7_pin14 
port_pin4  -> lcd_RS_pin4  
port_pin5  -> lcd_E_pin6   
GND        -> lcd_RW_pin5 if lcd reading is not needed.
port_pin6  -> lcd_RW_pin5 if lcd reading is needed


CAUTION!!!  FOR MODES WHERE LCD READING IS NOT POSSIBLE OR NOT ENABLED READ THE FOLLOWING.
            FOR SLOW LCD UNITS INCREASE THE "LCD_DELAY_TIME_US"  VALUE!!!  
            The driver does not read anything back from the lcd because that way i only need 6
            I/O lines and furthermore i can use the output only port found in MEGA103.
            Since i dont read anything back, i just wait for commands to be executed
            so below i define the time to wait for the two categories of commands of HD44780 .    
            The two CATEGORIES are:
            1) 2ms for clear screen and return home commands (nominal value=1,52 ms)
               and it is derived from "LCD_DELAY_TIME_US" .  
            2) 50us for all other commands (nominal value=37 us) 
  
            Even the slowest units will work reliably with LCD_DELAY_TIME_US=160. 
            The Longer delay needed for CATEGORY 1 is calculated from the "LCD_DELAY_TIME_US" value.
            The specifications for HD44780 @ 270KHZ are 37 us for CATEGORY 2 and 1,52 ms for CATEGORY 1 

USER DEFINED CHARS ARRAY's 

static const unsigned char user_char1[8]__attribute__((progmem)) = {0x0E,0x11,0x0E,0x04,0x1F,0x04,0x0A,0x11}; 

static const unsigned char user_char2[8]__attribute__((progmem)) = {0x11,0x0A,0x04,0x1F,0x04,0x0E,0x11,0x0E};

You could also define a char array like this:
static const unsigned char user_chars[2][8]__attribute__((progmem))={
                                                                      {0x0E,0x11,0x0E,0x04,0x1F,0x04,0x0A,0x11},
                                                                      {0x11,0x0A,0x04,0x1F,0x04,0x0E,0x11,0x0E}
                                                                    };
or                                                 

static const unsigned char user_chars[]__attribute__((progmem))={
                                                                  0x0E,0x11,0x0E,0x04,0x1F,0x04,0x0A,0x11,
                                                                  0x11,0x0A,0x04,0x1F,0x04,0x0E,0x11,0x0E
                                                                };
    
    and use the macro "lcd_fill_cgram(user_chars);"  instead of uploading each char separately.
    The array can have from 1 up to 8 used defined chars (max 64 bytes) but the macro will upload only
    as much as the array size so in the above example it will fill user defined char positions 0 and 1
    and will not overwrite user defined chars that might exist located at position 2 to 7.
    However it will always start uploading at position 0.

 
*******************************************************************************************************/
#ifndef LCD_HD44780_AVR_H
#define LCD_HD44780_AVR_H

/*######################################################################################################*/
/*           CONFIGURATION BLOCK STARTS HERE. Change these definitions to adapt setting                 */
/*######################################################################################################*/

/***********************************************************************************************/
/*                GLOBAL SETTINGS (settings described here are applyied everywhere)            */
/***********************************************************************************************/
/*  */

#ifndef F_CPU
#define F_CPU				16000000	//CPU CLOCK FREQUENCY
#endif

#define LCD_PROTOCOL			1       // 0 = 4 bit, 1 = I2C / 4 BIT
#define LCD_I2C_ADDRESS			0x4E    // 0x20 to 0x27, OR 4E for PCF8574 or 7E for PCF8574A

#define LCD_AUTO_LINE_FEED		0	//1 = Auto line feed, 0 = no Auto line feed 
#define LCD_DELAY_TIME_US		200	// THE TYPICAL TIME THE LCD NEEDS TO COMPLETE A COMMAND  
#define LCD_E_PULSE_WIDTH_US		20	// THE E PULSE WIDTH IN MICROSECONDS (Timing is accurate)
#define LCD_DECIMAL_POINT		','	// The decimal point punctuation mark char for lcd_get_i()

#define LCD_CHARS_PER_LINE		20	//visible chars per lcd line
#define LCD_LINES			4	//visible lines 
#define LCD_READ_REQUIRED		1	//0=write only, 1 = read from lcd unit (7 pins needed)
#define LCD_GROUND_RW_PIN_ALSO		0	// Ground the RW pin, valid only if LCD_READ_REQUIRED == 0.
#define LCD_ERROR_DETECTION		0	// 1= Read back and verify every character written.
#define READ_LCD_BUSY_FLAG		1	// 0=use delay, 1=check busy flag, valid only if LCD_READ_REQUIRED == 1
               
#define LCD_BACKUP_LOCATION		0       // 1=save lcd screen in RAM, 0=save in EEPROM              

#if LCD_PROTOCOL == 0

#define LCD_PORT			D   

#define LCD_DATA4_PORT			LCD_PORT	//port for data 0 pin
#define LCD_D4_PIN			0		//AVR port pin number

#define LCD_DATA5_PORT			LCD_PORT	//port for data 1 pin 
#define LCD_D5_PIN			1		//AVR port pin number

#define LCD_DATA6_PORT			LCD_PORT	//port for data 2 pin
#define LCD_D6_PIN			2		//AVR port pin number       

#define LCD_DATA7_PORT			LCD_PORT	//port for data 3 pin
#define LCD_D7_PIN			3		//AVR port pin number

#define LCD_RS_SIGNAL_PORT		LCD_PORT	//port for RS line
#define LCD_RS_PIN			4		//AVR port pin number

#define LCD_E_SIGNAL_PORT		LCD_PORT	//port for Enable line
#define LCD_E_PIN			5		//AVR port pin number

/* YOU NEED TO EDIT  "LCD_RW_SIGNAL_PORT" AND "LCD_RW_PIN" ONLY IF "LCD_READ_REQUIRED == 1" */
#if LCD_READ_REQUIRED == 1 || LCD_GROUND_RW_PIN_ALSO == 1
#define LCD_RW_SIGNAL_PORT		LCD_PORT	//port for R/W line
#define LCD_RW_PIN			6		//AVR port pin number
#endif

#elif LCD_PROTOCOL == 1

#define LCD_RS_PIN			0		
#define LCD_RW_PIN			1		
#define LCD_E_PIN			2		
#define LCD_BT_PIN			3		


#define LCD_D4_PIN			4		
#define LCD_D5_PIN			5		
#define LCD_D6_PIN			6		       
#define LCD_D7_PIN			7		

#endif



/*######################################################################################################*/
/*                             CONFIGURATION BLOCK ENDS HERE.                                           */
/*######################################################################################################*/

/* you shouldn't need to change anything below this line */

/********************************************************************************************************/
/*                             HD44780 DDRAM PARAMETERS                                                 */
/********************************************************************************************************/

#define LCD_LINE_LENGTH          0x40     /* internal line length */
#define LCD_START_LINE1          0x00     /* DDRAM address of first char of line 1 */
#define LCD_START_LINE2          0x40     /* DDRAM address of first char of line 2 */
#define LCD_START_LINE3          0x14     /* DDRAM address of first char of line 3 */
#define LCD_START_LINE4          0x54     /* DDRAM address of first char of line 4 */

/********************************************************************************************************/
/*                    INSTRUCTION REGISTER BIT POSITIONS AND COMBINATIONS                               */
/********************************************************************************************************/
#define LCD_CLR                  0      /* DB0: clear display */

#define LCD_HOME                 1      /* DB1: return to home position */

#define LCD_ENTRY_MODE           2      /* DB2: set entry mode */
#define LCD_ENTRY_INC            1      /*   DB1: 1=increment, 0=decrement  */
#define LCD_ENTRY_SHIFT          0      /*   DB2: 1=display shift on        */

#define LCD_ON                   3      /* DB3: turn lcd/cursor on */
#define LCD_ON_DISPLAY           2      /*   DB2: turn display on */
#define LCD_ON_CURSOR            1      /*   DB1: turn cursor on */
#define LCD_ON_BLINK             0      /*     DB0: blinking cursor ? */

#define LCD_MOVE                 4      /* DB4: move cursor/display */
#define LCD_MOVE_DISP            3      /*   DB3: move display (0-> cursor) ? */
#define LCD_MOVE_RIGHT           2      /*   DB2: move right (0-> left) ? */

#define LCD_FUNCTION             5      /* DB5: function set */
#define LCD_FUNCTION_8BIT        4      /*   DB4: set 8BIT mode (0->4BIT mode) */
#define LCD_FUNCTION_2LINES      3      /*   DB3: two lines (0->one line) */
#define LCD_FUNCTION_10DOTS      2      /*   DB2: 5x10 font (0->5x7 font) */

#define LCD_CGRAM                6      /* DB6: set CG RAM address */
     
#define LCD_DDRAM                7      /* DB7: set DD RAM address */

#define LCD_BUSY                 7      /* DB7: LCD is busy */

/* function set: set interface data length and number of display lines */
#define LCD_FUNCTION_4BIT_1LINE  0x20   /* 4-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_4BIT_2LINES 0x28   /* 4-bit interface, dual line,   5x7 dots */
#define LCD_FUNCTION_8BIT_1LINE  0x30   /* 8-bit interface, single line, 5x7 dots */
#define LCD_FUNCTION_8BIT_2LINES 0x38   /* 8-bit interface, dual line,   5x7 dots */

/* Lcd default mode used in this driver */
#define LCD_MODE_DEFAULT         ((1<<LCD_ENTRY_MODE) | (1<<LCD_ENTRY_INC) )                                         

/********************************************************************************************************/
/*                     LCD COMMANDS (CAN BE USED WITH "lcd_command(cmd);")                              */
/********************************************************************************************************/
/* set entry mode: display shift on/off, dec/inc cursor move direction */
#define LCD_ENTRY_DEC            0x04   /* display shift off, dec cursor move dir */
#define LCD_ENTRY_DEC_SHIFT      0x05   /* display shift on,  dec cursor move dir */
#define LCD_ENTRY_INC_           0x06   /* display shift off, inc cursor move dir */
#define LCD_ENTRY_INC_SHIFT      0x07   /* display shift on,  inc cursor move dir */

/* display on/off, cursor on/off, blinking char at cursor position */
#define LCD_DISP_OFF             0x08   /* display off                            */
#define LCD_DISP_ON              0x0C   /* display on, cursor off                 */
#define LCD_DISP_ON_BLINK        0x0D   /* display on, cursor off, blink char     */
#define LCD_DISP_ON_CURSOR       0x0E   /* display on, cursor on                  */
#define LCD_DISP_ON_CURSOR_BLINK 0x0F   /* display on, cursor on, blink char      */
#define LCD_CLEAR_SCREEN         (1<<LCD_CLR)
#define LCD_RETURN_HOME          (1<<LCD_HOME)

/* move cursor/shift display */
#define LCD_MOVE_CURSOR_LEFT     0x10   /* move cursor left  (decrement)          */
#define LCD_MOVE_CURSOR_RIGHT    0x14   /* move cursor right (increment)          */
#define LCD_MOVE_DISP_LEFT       0x18   /* shift display left                     */
#define LCD_MOVE_DISP_RIGHT      0x1C   /* shift display right                    */



/********************************************************************************************************/
/*                             LCD USEFULL DEFINITIONS                                                  */
/********************************************************************************************************/

#ifndef LOCATION_IS_RAM
#define LOCATION_IS_RAM          1
#endif
#ifndef LOCATION_IS_EEPROM
#define LOCATION_IS_EEPROM       2
#endif 
#ifndef LOCATION_IS_FLASH
#define LOCATION_IS_FLASH        3
#endif


/********************************************************************************************************/
/*                             PUBLIC FUNCTION PROTOTYPES                                               */
/********************************************************************************************************/

/* 
   MANUAL LCD INITIALIZATION IS NOT NEEDED ANYMORE. IT IS DONE AUTOMATICALLY!
   An exception is when "LCD_SAVE_MORE_CODE_SPACE == 1". Then a call to "lcd_init()" is needed.
*/ 
/* HIGH level functions */
extern void          lcd_init(void);
extern void          lcd_command(unsigned char cmd);
extern void          lcd_gotoxy(unsigned char lcd_x, unsigned char lcd_y);
extern void          lcd_putc(unsigned char c);
// user char is an array containing the charackters pixels, char position = 0,1,2,3,4,5,6,7,8
extern void	    lcd_putc_cgram(const unsigned char *user_char, unsigned char char_position);
extern void          lcd_puts(const unsigned char *s);
extern void          lcd_put_i(int value, unsigned char dot_position, unsigned char number_of_chars);
extern void          lcd_puts_p(const unsigned char *progmem_s);
extern void          lcd_puts_e(unsigned char *eeprom_s); 
extern void          lcd_putc_cgram(const unsigned char *user_char, unsigned char char_position);
extern void          lcd_clrline(unsigned char line);
#if LCD_PROTOCOL == 1
extern void	     lcd_backlight_off(void);
extern void	     lcd_backlight_on(void);
#else
#warning THERE IS NO BACKLIGHT CONTROL IN 4 BIT MODE.
#define lcd_backlight_on()	1
#define lcd_backlight_off()	0
#endif

/*
    The return value of the "lcd_getxy()" function is an integer,
    with the high byte containing the current line number (y) and the low byte 
    containing the char position in that line (x).
    If the lower byte has a value of 20 that means that you filled that line.
    This position result can only happen when no lcd reading is available.
    When lcd reading is available the maximum x == 19. 
*/
extern unsigned int  lcd_getxy(void);
#if  LCD_READ_REQUIRED == 1
extern unsigned char lcd_getc(void);
extern unsigned char lcd_get_error(void);
extern void lcd_get_line(unsigned char line, unsigned char* s);
extern void          lcd_backup_scr(void);
extern void          lcd_restore_scr(void);
#endif

extern unsigned char lcd_error_detected;

/********************************************************************************************************/
/*                                 FUNCTION RESEMBLING MACROS                                           */
/********************************************************************************************************/


#define lcd_clrscr()            lcd_command(LCD_CLEAR_SCREEN)
#define lcd_home()              lcd_command(LCD_RETURN_HOME)
//#define lcd_puti(x,y)           lcd_put_i(x,y,0xFF)
#define lcd_fill_cgram(array)   lcd_putc_cgram((const unsigned char*)array, sizeof(array))


/********************************************************************************************************/
/*                                 USEFULL MACROS                                                       */
/********************************************************************************************************/
/* macros for automatically storing string constant in program memory */
#ifndef P
#define P(s) ({static const unsigned char c[] __attribute__ ((progmem)) = s;c;})
#endif
#define lcd_puts_P(__s)         lcd_puts_p(P(__s))

#define LCD_RJUST		(1<<7)
#define LCD_LJUST		(0<<7)
#define LCD_ADD_ZEROS		(1<<6)


#endif //LCD_IO_H
/*######################################################################################################*/
/*                                         T H E   E N D                                                */
/*######################################################################################################*/


