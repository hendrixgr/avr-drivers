/****************************************************************************
 Title  :   C Test program for the LCD FUNCTIONS library (test_lcd.c)
 Author:    Chris efstathiou hendrix@otenet.gr
 Date:      19/Jan/2003
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device
 Comments:  This software is FREE.

*****************************************************************************/
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#include "lcd_hd44780_avr.h"


#ifndef DELAY_L_ACCURATE
/* 6 cpu cycles per loop + 3 overhead when a constant is passed. */
#define DELAY_L_ACCURATE(x)  ({ unsigned long number_of_loops=(unsigned long)x;   \
                                __asm__ volatile ( "L_%=: \n\t"                   \
                                                   "subi %A0,lo8(-(-1)) \n\t"     \
                                                   "sbci %B0,hi8(-(-1)) \n\t"     \
                                                   "sbci %C0,hlo8(-(-1)) \n\t"    \
                                                   "sbci %D0,hhi8(-(-1)) \n\t"    \
                                                   "brne L_%= \n\t"               \
                                                   : /* NO OUTPUT */              \
                                                   : "w" (number_of_loops)        \
                                                 );                               \
                             })                                      




#define TIME_L1_MS      ( 1*(F_CPU/6000) )     /* 1 MILLISECOND  */

#define TIME_L100_US    ( 1*(F_CPU/60000) )    /* 100 MICROSECONDS  */

#define DELAY_MS(ms)    DELAY_L_ACCURATE( (TIME_L1_MS*(ms)) )

#define DELAY_L_US(us)  DELAY_L_ACCURATE( (TIME_L100_US*((us)/100))+1 )

#endif /* #ifndef DELAY_L_ACCURATE */


  /* USER DEFINED CHARS ARRAY's */
static const unsigned char user_char1[8]__attribute__((progmem)) = {0x0E,0x11,0x0E,0x04,0x1F,0x04,0x0A,0x11}; 

static const unsigned char user_char2[8]__attribute__((progmem)) = {0x11,0x0A,0x04,0x1F,0x04,0x0E,0x11,0x0E};

/*
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

*/

void main(void)
{

/*
    Since we also test the eeprom DONT! forget to write the test_lcd.eep to AVR's eeprom 
    otherwise the lcd will not show the eeprom_string
*/
unsigned char ram_string[]={"DONE TESTING..."};

static  unsigned char eeprom_string[]__attribute__((section(".eeprom")))={"lcd_puts_e testing"};

/*11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
/*11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
/* It may seems strange that i use a union but this approach saves a lot of code space */
union double_byte_union {
                           unsigned int  integer; 
                           unsigned char byte[2];
                         } word;

/* 
    MANUAL LCD INITIALIZATION IS NOT NEEDED ANYMORE. IT IS DONE AUTOMATICALLY! 
    lcd(s) is(are) automatically initialized and if more than 1, lcd unit 0 gets active.
    You can always call "lcd_init()" if you wish.
    An exception is when "LCD_ABSOLUTELY_MINIMUM == 1". Then a call to "lcd_init()" is needed.
 
*/

/* 
   UPLOAD USER DEFINED CHARS TO LCD's CGRAM AND THEN RETURN TO THE PREVIUS DDRAM CURSOR POSITION
   (It can be made anywhere as long you upload them before you use them)
*/


   lcd_putc_cgram(user_char1,0);
   lcd_putc_cgram(user_char2,1);

   lcd_gotoxy(0,1);
   lcd_clrscr(); lcd_puts_P( "Hello World\nthis is line 2 " );


   /* DISPLAY USER DEFINED CHARS */
   lcd_putc(0);
   lcd_putc(1);

/*
    The return value of the "lcd_getxy()" function is an integer,
    with the high byte containing the current line number (y) and the low byte 
    containing the char position in that line (x).
    If the lower byte has a value of 20 that means that you filled that line.
    This position result can only happen when no lcd reading is available.
    When lcd reading is available the maximum x == 19. 
*/

   lcd_gotoxy(19,3); 
   word.integer=lcd_getxy(); 
   lcd_gotoxy(16,3);
   lcd_putc(',');
   lcd_put_i(*(word.byte),0,(2|LCD_LJUST));


#if LCD_READ_REQUIRED == 1 && LCD_BACKUP_REQUIRED == 1
   /* Take a snapshot of the display and save it to EEPROM (defined in the lcd_io.h) */
   lcd_backup_scr();
#endif
/*---------------------------------------------------------------------------------------------------*/

#if LCD_AUTO_LINE_FEED == 1
  lcd_puts_e(eeprom_string); lcd_puts(ram_string);  
#elif LCD_AUTO_LINE_FEED == 0
/*---------------------------------------------------------------------------------------------------*/
/*
   I positioned this command here to test the correct saving of the lcd x,y coordinates
   between lcd unit switching, if you select mode 7 
*/


   lcd_gotoxy(0,2); lcd_puts_P("NOT VISIBLE LINE");
   lcd_clrline(2);  lcd_puts_e(eeprom_string);
   lcd_gotoxy(0,3); lcd_puts(ram_string);  
#endif /* #elif LCD_AUTO_LINE_FEED == 0 */

#if LCD_READ_REQUIRED == 1 

/*
   Read a char from the lcd DDRAM at the selected position and then display the read char to another location.
   When using the lcd_getc() function the Address Counter (AC) is auto incremented or decremented
   according to the ENTRY MODE selected during initialization, just like the lcd_putc() function works, 
   so when doing a sequential read, there is no need to reposition the cursor each time a read is performed.
   EXAMPLE: To backup line 1 only of the lcd, the below code works fine.

            unsigned char lcd_backup[LCD_CHARS_PER_LINE];
            unsigned char x=0;
            lcd_gotoxy(0,1); 
            for(x=0; x<LCD_CHARS_PER_LINE; x++)
              {
                *(lcd_backup+x)=lcd_getc();
              }
*/


unsigned char  x=0;
lcd_gotoxy(0,0);
x=lcd_getc();
lcd_gotoxy(19,1); lcd_putc(x);




#endif

#if LCD_READ_REQUIRED == 1 && LCD_BACKUP_REQUIRED == 1
/* Delay in order to see the lcd screen before the restore function alter it. */
DELAY_MS(5000);

/* Restore the lcd display using the earlier saved image. */
lcd_restore_scr(); 
#endif




return;
}
/*######################################################################################################*/
/*                                         T H E   E N D                                                */
/*######################################################################################################*/

