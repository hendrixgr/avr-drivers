
/****************************************************************************
 Title  :   C  file for the HD44780U LCD library (lcd_io.c)
 Author:    Chris efstathiou hendrix@vivodinet.gr
 Date:      6/July/2018
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device
 Comments:  This software is FREE.
 
*****************************************************************************/

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include "i2c.h"
#include "lcd_hd44780_avr.h"


/*******************************************************************************************************/
#if LCD_PROTOCOL == 0
#warning LCD IN 4  BIT MODE
#elif LCD_PROTOCOL == 1
#warning LCD IN I2C MODE
#else
#error CHOOSE LCD MODE (4bit OR I2C)
#endif

// SOME MISCONFIGURATION CHECKING
/*
#if defined(LCD_ERROR_DETECTION) && LCD_ERROR_DETECTION == 1
#if defined(LCD_READ_REQUIRED)) && LCD_READ_REQUIRED == 0
#undef LCD_READ_REQUIRED
#define LCD_READ_REQUIRED	1
#endif
#endif
*/
#if  LCD_READ_REQUIRED == 0 && LCD_ERROR_DETECTION == 1
#warning LCD ERROR DETECTION NEEDS LCD READING ACTIVE
#warning LCD ERROR DETECTION IS TURNED OFF!
#undef LCD_ERROR_DETECTION
#define LCD_ERROR_DETECTION	0
#endif
/*
#if  LCD_READ_REQUIRED == 0 && READ_LCD_BUSY_FLAG == 1
#warning READ_LCD_BUSY_FLAG NEEDS LCD READING ACTIVE
#warning READ_LCD_BUSY_FLAG IS TURNED OFF!
#undef READ_LCD_BUSY_FLAG
#define READ_LCD_BUSY_FLAG	0
#endif
*/
#if defined(LCD_READ_REQUIRED) && LCD_READ_REQUIRED == 1
#if defined(LCD_GROUND_RW_PIN_ALSO) &&	LCD_GROUND_RW_PIN_ALSO == 1
#undef LCD_GROUND_RW_PIN_ALSO
#define LCD_GROUND_RW_PIN_ALSO	0
#endif
#endif

#define INIT_MODE		2
#define DATA_MODE		1
#define CMD_MODE		0

#if LCD_READ_REQUIRED == 1  
#define BUSY_FLAG		0
#define ADDRESS_COUNTER		0
#define DATA			1
#endif

/* constants/macros */
#ifndef CONCAT1
#define CONCAT1(a, b) CONCAT2(a, b)
#endif

#ifndef CONCAT2
#define CONCAT2(a, b) a ## b
#endif

/* Calculation of the LCD_E_PULSE duration in cpu cycles. sbi() and cbi() are taken in to account. */

#define LCD_E_PULSE_CYCLES          ( (LCD_E_PULSE_WIDTH_US*1000L)/(1000000000/F_CPU) )

#if   LCD_E_PULSE_CYCLES <= 0
#warning " LCD_E_PULSE_WIDTH_US in lcd_io.h too small!!"
#define LCD_E_DELAY()             { }

#elif LCD_E_PULSE_CYCLES == 1
#define LCD_E_DELAY()             __asm__ volatile("nop")

#elif LCD_E_PULSE_CYCLES == 2 
#define LCD_E_DELAY()             __asm__ volatile("nop \n\t nop \n\t" ::)

#elif LCD_E_PULSE_CYCLES == 3
#define LCD_E_DELAY()             __asm__ volatile("nop \n\t nop \n\t nop \n\t" ::)

#elif LCD_E_PULSE_CYCLES >= 4


#define LCD_E_DELAY_LOOPS          (LCD_E_PULSE_CYCLES/4)

#define  LCD_E_DELAY()            { delay_int(LCD_E_DELAY_LOOPS); }


#endif 
                                 

// CONVERSION OF LCD_DELAY_TIME_US TO TRUE MICROSECONDS/
#define LCD_DELAY_LOOPS    ( ((LCD_DELAY_TIME_US*1000L)/(1000000000/F_CPU))/4 )
#if LCD_DELAY_LOOPS <=0   
#undef  LCD_DELAY_LOOPS
#define LCD_DELAY_LOOPS    1
#warning " LCD_DELAY_TIME_US in lcd_io.h too small!!"
#elif LCD_DELAY_LOOPS > 65535
#undef  LCD_DELAY_LOOPS
#define LCD_DELAY_LOOPS    65535
#warning " LCD_DELAY_TIME_US in lcd_io.h too long!!"
#endif

// LCD_DELAY_TIME_US (about 100 microseconds)
#define LCD_FUNCTION_DELAY_TYP()  { delay_int(LCD_DELAY_LOOPS);  }  

// LCD_DELAY_TIME_US * 40 (about 4 ms)
#define LCD_FUNCTION_DELAY_MAX()  { unsigned int delay_multiplier = 40;  \
                                    while(delay_multiplier) { LCD_FUNCTION_DELAY_TYP(); delay_multiplier--; } \
                                  }      

// LCD_DELAY_TIME_US * 500 (about 50 ms)
#define LCD_POWER_ON_DELAY()  { unsigned int delay_multiplier = 500;  \
                                while(delay_multiplier) { LCD_FUNCTION_DELAY_TYP(); delay_multiplier--; } \
                              }    
// LCD_DELAY_TIME_US * 100 (about 10 ms) 
#define LCD_INIT_DELAY()      { unsigned int delay_multiplier = 100;  \
                                while(delay_multiplier) { LCD_FUNCTION_DELAY_TYP(); delay_multiplier--; } \
                              }    

#define lcd_e_high()		{ LCD_E_OUT_REG |= (1<<LCD_E_PIN); LCD_E_DELAY(); }
#define lcd_e_low()		{ LCD_E_OUT_REG &= (~(1<<LCD_E_PIN)); LCD_E_DELAY(); }
#define lcd_toggle_e()		{ lcd_e_high(); lcd_e_low(); }

#if LCD_LINES == 1 || LCD_LINES == 2 || LCD_LINES == 4
#else
#error THE LCD LINES MUST BE 1 OR 2 OR 4 !
#endif

#define LCD_D4_OUT_REG       CONCAT1(PORT, LCD_DATA4_PORT)
#define LCD_D5_OUT_REG       CONCAT1(PORT, LCD_DATA5_PORT)
#define LCD_D6_OUT_REG       CONCAT1(PORT, LCD_DATA6_PORT)
#define LCD_D7_OUT_REG       CONCAT1(PORT, LCD_DATA7_PORT)
#define LCD_RS_OUT_REG       CONCAT1(PORT, LCD_RS_SIGNAL_PORT)
#define LCD_RW_OUT_REG       CONCAT1(PORT, LCD_RW_SIGNAL_PORT)
#define LCD_E_OUT_REG        CONCAT1(PORT, LCD_E_SIGNAL_PORT)

#define LCD_D4_DDR_REG       CONCAT1(DDR, LCD_DATA4_PORT)
#define LCD_D5_DDR_REG       CONCAT1(DDR, LCD_DATA5_PORT)
#define LCD_D6_DDR_REG       CONCAT1(DDR, LCD_DATA6_PORT)
#define LCD_D7_DDR_REG       CONCAT1(DDR, LCD_DATA7_PORT)
#define LCD_RS_DDR_REG       CONCAT1(DDR, LCD_RS_SIGNAL_PORT)
#define LCD_RW_DDR_REG       CONCAT1(DDR, LCD_RW_SIGNAL_PORT)
#define LCD_E_DDR_REG        CONCAT1(DDR, LCD_E_SIGNAL_PORT)

#if   LCD_READ_REQUIRED == 1

#define LCD_D4_PIN_REG        CONCAT1(PIN, LCD_DATA4_PORT)
#define LCD_D5_PIN_REG        CONCAT1(PIN, LCD_DATA5_PORT)
#define LCD_D6_PIN_REG        CONCAT1(PIN, LCD_DATA6_PORT)
#define LCD_D7_PIN_REG        CONCAT1(PIN, LCD_DATA7_PORT)

#endif


/*######################################################################################################*/
/*                                        TYPE DEFINITIONS                                              */
/*######################################################################################################*/


typedef unsigned char  u08;
typedef unsigned short u16;


/*######################################################################################################*/
/*                                    LOCAL FUNCTION PROTOTYPES                                         */
/*######################################################################################################*/

static inline void delay_int(uint16_t __count) __attribute__((always_inline)); 
static inline void lcd_write(unsigned char data, unsigned char rs);
#if  LCD_READ_REQUIRED == 1
static inline void          wait_until_busy_flag_is_clear(void);
static inline unsigned char lcd_read(unsigned char rs);
#endif
 

/*######################################################################################################*/
/*                                      GLOBAL VARIABLES                                                */
/*######################################################################################################*/

static unsigned char x,y;
static unsigned char lcd_line_start[4]={ LCD_START_LINE1,LCD_START_LINE2,
                                         LCD_START_LINE3,LCD_START_LINE4
                                       };
static unsigned char putc_lock=0;
//static unsigned char lcd_init_lock=0;
static unsigned char lcd_is_busy = 0;
unsigned char lcd_error_detected = 0;

#if   LCD_BACKUP_LOCATION == 0 

static unsigned char lcd_backup[LCD_LINES][LCD_CHARS_PER_LINE]__attribute__((section(".eeprom")));

#elif LCD_BACKUP_LOCATION == 1

static unsigned char lcd_backup[LCD_LINES][LCD_CHARS_PER_LINE];

#endif   /* #if   LCD_BACKUP_LOCATION == 0 -> #elif LCD_BACKUP_LOCATION == 1 */

#if LCD_PROTOCOL == 1
unsigned char lcd_i2c_address  =  LCD_I2C_ADDRESS;
unsigned char backlight = 1;
#endif	


/*######################################################################################################*/
/*                                  LOCAL FUNCTION DEFINITIONS                                          */
/*######################################################################################################*/

static inline void delay_int(uint16_t __count){

	__asm__ volatile (
		"1: sbiw %0,1" "\n\t"
		"brne 1b"
		: "=w" (__count)
		: "0" (__count)
	);
} 


#if LCD_READ_REQUIRED == 1 
#if LCD_PROTOCOL == 0
/*######################################################################################################*/
/* loops while lcd is busy, reads address counter */
static inline void wait_until_busy_flag_is_clear(void){

#if  READ_LCD_BUSY_FLAG == 1    
do{
    LCD_RS_OUT_REG &= ~(1<<LCD_RS_PIN);     
    // configure data pins as input 
    LCD_D7_DDR_REG &= ~(1<<LCD_D7_PIN);
    LCD_D6_DDR_REG &= ~(1<<LCD_D6_PIN);
    LCD_D5_DDR_REG &= ~(1<<LCD_D5_PIN);
    LCD_D4_DDR_REG &= ~(1<<LCD_D4_PIN);
    // set R/W pin for reading from LCD 
    LCD_RW_OUT_REG |= (1<<LCD_RW_PIN);

    lcd_e_high();
    //Only bit 7 must be checked.
    if(LCD_D7_PIN_REG & (1<<LCD_D7_PIN)) { lcd_is_busy = 1; }else{ lcd_is_busy = 0; }

    lcd_e_low();
    lcd_e_high();
    lcd_e_low();


}while(lcd_is_busy);
#else

LCD_FUNCTION_DELAY_TYP();

#endif

return;  
}

/*######################################################################################################*/

static inline unsigned char lcd_read(unsigned char rs){

unsigned char data=0;
/* RS=1: read data, RS=0: read busy flag and address counter, RW=1  read mode */  
    
    if(rs){ LCD_RS_OUT_REG |= (1<<LCD_RS_PIN); }else{ LCD_RS_OUT_REG &= ~(1<<LCD_RS_PIN); }     
    // configure data pins as input 
    LCD_D7_DDR_REG &= ~(1<<LCD_D7_PIN);
    LCD_D6_DDR_REG &= ~(1<<LCD_D6_PIN);
    LCD_D5_DDR_REG &= ~(1<<LCD_D5_PIN);
    LCD_D4_DDR_REG &= ~(1<<LCD_D4_PIN);
    // set R/W pin for reading from LCD 
    LCD_RW_OUT_REG |= (1<<LCD_RW_PIN);

    lcd_e_high();

    if(LCD_D7_PIN_REG & (1<<LCD_D7_PIN)) { data|=(1<<7); }
    if(LCD_D6_PIN_REG & (1<<LCD_D6_PIN)) { data|=(1<<6); }
    if(LCD_D5_PIN_REG & (1<<LCD_D5_PIN)) { data|=(1<<5); }
    if(LCD_D4_PIN_REG & (1<<LCD_D4_PIN)) { data|=(1<<4); }
   
    lcd_e_low();
    lcd_e_high();

    if(LCD_D7_PIN_REG & (1<<LCD_D7_PIN)) { data|=(1<<3); }
    if(LCD_D6_PIN_REG & (1<<LCD_D6_PIN)) { data|=(1<<2); }
    if(LCD_D5_PIN_REG & (1<<LCD_D5_PIN)) { data|=(1<<1); }
    if(LCD_D4_PIN_REG & (1<<LCD_D4_PIN)) { data|=(1<<0); }
    
    lcd_e_low();

    if(rs) { wait_until_busy_flag_is_clear(); }  

return(data);
}

#elif LCD_PROTOCOL == 1
/*######################################################################################################*/

/* loops while lcd is busy, reads address counter */
static inline void wait_until_busy_flag_is_clear(void){

#if  READ_LCD_BUSY_FLAG == 1    
#warning LCD READ BUSY FLAG ACTIVE
unsigned char i2c_tx_data = 0, i2c_rx_data = 0;

//i2c_stop();
// Configure the i2c data byte.
i2c_tx_data = 0;
i2c_tx_data &= (~(1<<LCD_RS_PIN)); //COMMAND TO BE SEND: READ BUSY FLAG 
i2c_tx_data |= (1<<LCD_RW_PIN);    // LCD IN READ MODE
i2c_tx_data |= (backlight<<LCD_BT_PIN);  
// PUT THE pca8574 4 bits in input mode.
i2c_tx_data |= ((1<<LCD_D4_PIN)|(1<<LCD_D5_PIN)|(1<<LCD_D6_PIN)|(1<<LCD_D7_PIN)); 
I2C_START_TX(lcd_i2c_address);
i2c_put_byte( i2c_tx_data );

do{
       I2C_START_TX(lcd_i2c_address);
       i2c_tx_data |= (1<<LCD_E_PIN); // lcd_e_high()
       i2c_put_byte( i2c_tx_data );
       I2C_START_RX(lcd_i2c_address);
       i2c_rx_data=i2c_get_byte(I2C_QUIT);
       if(i2c_rx_data & (1<<LCD_D7_PIN)) { lcd_is_busy = 1; }else{ lcd_is_busy = 0; }        
       I2C_START_TX(lcd_i2c_address);
       i2c_tx_data |= ((1<<LCD_D4_PIN)|(1<<LCD_D5_PIN)|(1<<LCD_D6_PIN)|(1<<LCD_D7_PIN)); 
       i2c_tx_data &= ~(1<<LCD_E_PIN); // lcd_e_low()
       i2c_put_byte( i2c_tx_data );
       i2c_tx_data |= (1<<LCD_E_PIN); // lcd_e_high()
       i2c_put_byte( i2c_tx_data );
       i2c_tx_data &= ~(1<<LCD_E_PIN); // lcd_e_low()
       i2c_put_byte( i2c_tx_data );


}while(lcd_is_busy);

#else

LCD_FUNCTION_DELAY_TYP();

#endif
i2c_stop();

return;  
}

/*######################################################################################################*/

/* RS=1: read data, RS=0: read busy flag and address counter, RW=1  read mode */
static inline unsigned char lcd_read(unsigned char rs){

unsigned char i2c_tx_data = 0, i2c_rx_data = 0, data = 0;

i2c_tx_data = 0; // reset the i2c data byte.  

if(rs) { wait_until_busy_flag_is_clear(); i2c_tx_data |= (1<<LCD_RS_PIN);  } 

i2c_tx_data |= 1<<LCD_RW_PIN; 
i2c_tx_data |= (backlight<<LCD_BT_PIN);
i2c_tx_data |= ((1<<LCD_D4_PIN)|(1<<LCD_D5_PIN)|(1<<LCD_D6_PIN)|(1<<LCD_D7_PIN));
I2C_START_TX(lcd_i2c_address);
i2c_put_byte( i2c_tx_data);
i2c_tx_data |= (1<<LCD_E_PIN); // lcd_e_high() 
i2c_put_byte( i2c_tx_data);

I2C_START_RX(lcd_i2c_address);
i2c_rx_data=i2c_get_byte(I2C_QUIT);
if(i2c_rx_data&(1<<LCD_D7_PIN)) { data|=(1<<7); }
if(i2c_rx_data&(1<<LCD_D6_PIN)) { data|=(1<<6); }
if(i2c_rx_data&(1<<LCD_D5_PIN)) { data|=(1<<5); }
if(i2c_rx_data&(1<<LCD_D4_PIN)) { data|=(1<<4); }

I2C_START_TX(lcd_i2c_address);
i2c_tx_data &= (~(1<<LCD_E_PIN)); // lcd_e_low()
i2c_put_byte( i2c_tx_data ); 
i2c_tx_data |= (1<<LCD_E_PIN); // lcd_e_high()
i2c_put_byte( i2c_tx_data );    

I2C_START_RX(lcd_i2c_address);
i2c_rx_data=i2c_get_byte(I2C_QUIT);
if(i2c_rx_data&(1<<LCD_D7_PIN)) { data|=(1<<3); }
if(i2c_rx_data&(1<<LCD_D6_PIN)) { data|=(1<<2); }
if(i2c_rx_data&(1<<LCD_D5_PIN)) { data|=(1<<1); }
if(i2c_rx_data&(1<<LCD_D4_PIN)) { data|=(1<<0); }

i2c_tx_data &= (~(1<<LCD_E_PIN)); // lcd_e_low()
I2C_START_TX(lcd_i2c_address);
i2c_put_byte( i2c_tx_data ); 

if(rs) { wait_until_busy_flag_is_clear(); }  
i2c_stop();



return(data);
}
#endif   // #if LCD_PROTOCOL == 0 , #elif LCD_PROTOCOL == 1

#endif  /* #if LCD_READ_REQUIRED == 1  */

/*######################################################################################################*/
#if LCD_PROTOCOL == 0
static inline void lcd_write(unsigned char data, unsigned char rs){

//if(lcd_init_lock == 0) { lcd_init(); }

#if LCD_READ_REQUIRED == 1 
       LCD_RW_OUT_REG &= ~(1<<LCD_RW_PIN);  
       LCD_D4_DDR_REG |= (1<<LCD_D4_PIN);
       LCD_D5_DDR_REG |= (1<<LCD_D5_PIN);
       LCD_D6_DDR_REG |= (1<<LCD_D6_PIN);
       LCD_D7_DDR_REG |= (1<<LCD_D7_PIN);
       //LCD_E_DELAY();
#endif
       /* output high nibble first */ 
       /* Make all data pins & rs pin go low */
       if(rs==DATA_MODE){ LCD_RS_OUT_REG |= (1<<LCD_RS_PIN); }else{ LCD_RS_OUT_REG &= ~(1<<LCD_RS_PIN); } 
       //LCD_E_DELAY();
       lcd_e_high();
       if(data&0x10){ LCD_D4_OUT_REG |= (1<<LCD_D4_PIN); }else{ LCD_D4_OUT_REG &= ~(1<<LCD_D4_PIN); } 
       if(data&0x20){ LCD_D5_OUT_REG |= (1<<LCD_D5_PIN); }else{ LCD_D5_OUT_REG &= ~(1<<LCD_D5_PIN); }
       if(data&0x40){ LCD_D6_OUT_REG |= (1<<LCD_D6_PIN); }else{ LCD_D6_OUT_REG &= ~(1<<LCD_D6_PIN); }
       if(data&0x80){ LCD_D7_OUT_REG |= (1<<LCD_D7_PIN); }else{ LCD_D7_OUT_REG &= ~(1<<LCD_D7_PIN); }

       /* Strobe E pin (Time is defined in lcd_io.h) */
       lcd_e_low();
       lcd_e_high()

       /* output low nibble */
       /* if INIT_MODE skip this section else execute it */
       if (rs != INIT_MODE){
          if(data&0x01){ LCD_D4_OUT_REG |= (1<<LCD_D4_PIN); }else{ LCD_D4_OUT_REG &= ~(1<<LCD_D4_PIN); } 
          if(data&0x02){ LCD_D5_OUT_REG |= (1<<LCD_D5_PIN); }else{ LCD_D5_OUT_REG &= ~(1<<LCD_D5_PIN); }
          if(data&0x04){ LCD_D6_OUT_REG |= (1<<LCD_D6_PIN); }else{ LCD_D6_OUT_REG &= ~(1<<LCD_D6_PIN); }
          if(data&0x08){ LCD_D7_OUT_REG |= (1<<LCD_D7_PIN); }else{ LCD_D7_OUT_REG &= ~(1<<LCD_D7_PIN); }
          /* Strobe E pin (Time is defined in lcd_io.h) */
          lcd_e_low();
#if LCD_READ_REQUIRED == 1
          wait_until_busy_flag_is_clear(); /* Check Busy Flag. */
       }
       else{ LCD_FUNCTION_DELAY_TYP(); }   //when INIT_MODE add a delay since busy flag cannot be checked
#elif LCD_READ_REQUIRED == 0
       }
       LCD_FUNCTION_DELAY_TYP(); 
#endif

return;
}

#elif LCD_PROTOCOL == 1
static inline void lcd_write(unsigned char data, unsigned char rs){

unsigned char i2c_tx_data=0;

       /* INITIALIZATION */
       /* Set clock and data pins as outputs, at low state and set rs value */ 
       i2c_tx_data = 0; // reset the i2c data byte.
       i2c_tx_data |= (1<<LCD_E_PIN); // lcd_e_high() 
       i2c_tx_data |= (backlight<<LCD_BT_PIN);
       if(rs==DATA_MODE) { i2c_tx_data |= (1<<LCD_RS_PIN); } 
       if(data&0x10) i2c_tx_data |= (1<<LCD_D4_PIN);  
       if(data&0x20) i2c_tx_data |= (1<<LCD_D5_PIN);   
       if(data&0x40) i2c_tx_data |= (1<<LCD_D6_PIN);   
       if(data&0x80) i2c_tx_data |= (1<<LCD_D7_PIN);  
        
       //i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) ); // THIS DOES NOT WORK, IT NEEDS TWO SEPARATE CODE LINES!!!
       I2C_START_TX(lcd_i2c_address);
       i2c_put_byte( i2c_tx_data);
       //i2c_tx_data &= (~(1<<LCD_E_PIN)); // lcd_e_low()
       i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); 
       //i2c_tx_data |= (1<<LCD_E_PIN); // lcd_e_high() 
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) );
       //LCD_E_DELAY();
 
    
       /* IF MODE IS NOT "INIT_MODE" SEND LOW NIBBLE AND THE APPROPRIATE CONTROL SIGNALS */
       if(rs != INIT_MODE)
        {       
           /* Reset the data bits */
           i2c_tx_data &= (~((1<<LCD_D4_PIN)|(1<<LCD_D5_PIN)|(1<<LCD_D6_PIN)|(1<<LCD_D7_PIN)));
           if(data&0x01) i2c_tx_data |= (1<<LCD_D4_PIN);  
           if(data&0x02) i2c_tx_data |= (1<<LCD_D5_PIN);   
           if(data&0x04) i2c_tx_data |= (1<<LCD_D6_PIN);   
           if(data&0x08) i2c_tx_data |= (1<<LCD_D7_PIN);    

           i2c_put_byte(i2c_tx_data);                    
           i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); // lcd_e_low()
#if LCD_READ_REQUIRED == 1
           wait_until_busy_flag_is_clear(); /* Check Busy Flag. */
#else
           LCD_FUNCTION_DELAY_TYP(); 
#endif
       }
       else{ LCD_FUNCTION_DELAY_TYP(); }   //when INIT_MODE add a delay since busy flag cannot be checked

i2c_stop();



return;      
}

#endif


/*######################################################################################################*/
/*                                 PUBLIC FUNCTION DEFINITIONS                                          */
/*######################################################################################################*/

#if LCD_PROTOCOL == 1
inline void  lcd_backlight_off(void){
uint16_t address = 0;
uint8_t x_column = 0, y_row = 0;

address = lcd_getxy();
y_row = address /100;
x_column = address - (y*100);
backlight = 0;
lcd_gotoxy(x_column, y_row);

return;
}

inline void  lcd_backlight_on(void){
uint16_t address = 0;
uint8_t x_column = 0, y_row = 0;

address = lcd_getxy();
y_row = address /100;
x_column = address - (y*100);
backlight = 1;
lcd_gotoxy(x_column, y_row);

return;
}

#endif

// send command <cmd> to LCD 
void lcd_command(unsigned char cmd){

   lcd_write(cmd,CMD_MODE);
   //this command needs more waiting time to execute
   if (cmd==(1<<LCD_CLR) || cmd==(1<<LCD_HOME)){ 
//#if LCD_READ_REQUIRED == 0 || READ_LCD_BUSY_FLAG == 0
      LCD_FUNCTION_DELAY_MAX();
//#endif
      x=0;
      y=0;
   }       

return;    
}
/*######################################################################################################*/
/* goto position (x,y) */
void lcd_gotoxy(unsigned char lcd_x, unsigned char lcd_y){

   /* If out of range values for the lcd coordinates are given, lock lcd_putc() and return */
   putc_lock=0;
   if(lcd_x >= LCD_CHARS_PER_LINE || lcd_y >= LCD_LINES) { putc_lock=1; return; }
   lcd_command((1<<LCD_DDRAM)+lcd_line_start[lcd_y]+lcd_x);
   x=lcd_x;
   y=lcd_y;

return;
}
/*######################################################################################################*/
/* print character at current cursor position */
void lcd_putc(unsigned char c){
#if  LCD_READ_REQUIRED == 1 && LCD_ERROR_DETECTION == 1
static unsigned char cnt1 = 0;
#endif

 if (!putc_lock)
    {
#if LCD_LINES!=1
       if (c=='\n'){
          if(y<LCD_LINES-1){ lcd_gotoxy(0,(y+1)); }
  
       }else if (x<LCD_CHARS_PER_LINE){
                 lcd_write(c, DATA_MODE);
#if  LCD_READ_REQUIRED == 1 && LCD_ERROR_DETECTION == 1
#warning LCD SCREEN VERIFICATION IS ACTIVE
                 lcd_gotoxy(x, y);
                 if(lcd_read(DATA) != c){ lcd_error_detected = 1; }else{ lcd_error_detected = 0; cnt1 = 0; }
#endif
                 x++;
              }
#if LCD_AUTO_LINE_FEED == 1
               else if (y<LCD_LINES-1){ 
                        lcd_gotoxy(0,(y+1)); 
                        lcd_write(c, DATA_MODE);
#if  LCD_READ_REQUIRED == 1 && LCD_ERROR_DETECTION == 1
                        lcd_gotoxy(x, y);
                        if(lcd_read(DATA) != c){ lcd_error_detected = 1; }else{ lcd_error_detected = 0; cnt1 = 0; } 
#endif
                        x++;
                   
                     }else { 
                               lcd_gotoxy(0,0); 
                               lcd_write(c, DATA_MODE); 
#if  LCD_READ_REQUIRED == 1 && LCD_ERROR_DETECTION == 1
                               lcd_gotoxy(x, y);
                               if(lcd_read(DATA) != c){ lcd_error_detected = 1; }else{lcd_error_detected = 0; cnt1 = 0;}
#endif
                               x++;
                            }
#endif

#else
       if(c=='\n') { return; }
       if(x<LCD_CHARS_PER_LINE) { lcd_write(c, DATA_MODE); x++; }
#endif
  }
#if  LCD_READ_REQUIRED == 1 && LCD_ERROR_DETECTION == 1
#if LCD_PROTOCOL == 0
if (lcd_error_detected && cnt1 <= 4){
   lcd_toggle_e();
   lcd_init();
   cnt1++;
}
#endif
#endif

return;
}
/*######################################################################################################*/
void lcd_putc_cgram(const unsigned char *user_char, unsigned char char_position){

unsigned char x1=0;

if(char_position<=7)
 {    
   lcd_command((1<<LCD_CGRAM)+(char_position*8));
   for(x1=0; x1<=7; x1++)
     {
        lcd_write(pgm_read_byte(user_char++), DATA_MODE); 
     }
 }
else{
       lcd_command((1<<LCD_CGRAM));
       for(x1=0; x1<char_position; x1++)
         { 
            if(x1>=64) { break; } 
            lcd_write(pgm_read_byte(user_char++), DATA_MODE); 
         }
    }     

lcd_gotoxy(x,y);

return;
}




/* print signed integer on lcd with or without comma (no auto linefeed) */
void lcd_put_i(int value, unsigned char dot_position, unsigned char number_of_chars){

   unsigned char lcd_data[9]={' ',' ',' ',' ',' ',' ',' ',' ',' ' }; 
   unsigned char negative=0, position=sizeof(lcd_data), radix=10, digits_needed = 0, zeros_needed = 0; 
   unsigned char options = 0;

   // The three lower bits are for declaring how many digits to reserve on the lcd screen.
   // The remaining bits are reserved for options
   options = number_of_chars;
   //options &= (~(7<<0));
   number_of_chars &= (7<<0);       

   /* Some adjustments if the number is negative */
   if(value<0) { value=-value; *(lcd_data)='-'; negative=1; digits_needed++; } 

/*
   Convert 16 bit positive now made number to ascii algorithm. Position starts at array position 8
   so in the extreme case of lcd_puti(-32767,5,8); the display should show -0,32767 (8 chars) which
   means that the position variable will count down to 2.
   Since the sign is always in array position 0 there is always an empty slot in position 1 at least.
   This empty slot is essential for left allignment.
   Also note that position 8 gets the least significant digit.
   I have tried varius code schemes but this one yields the smaller code. 
*/
   do{
        if (dot_position){
           if ((sizeof(lcd_data)-position)>=dot_position){
              position--;
              digits_needed++;
              *(lcd_data+position)=',';
              dot_position=0;
           }
        } 
        position--;
        digits_needed++;
        *(lcd_data+position)=(value%radix)+'0';
        value/=radix;

   }while((value || dot_position) && dot_position <= 5); 

   /* Move the array contents to the start of the array filling the rest with spaces (Left align data) */
   /* radix has no meaning from here on, it is just used as a counter. */
   if (options & LCD_ADD_ZEROS){
      if (number_of_chars > digits_needed){ 
         zeros_needed = number_of_chars-digits_needed; 
         do{
              position--; 
              zeros_needed--;
              *(lcd_data+position)='0';

         }while(zeros_needed > 0);
      }
   }

   if ((options & LCD_RJUST) == 0){   // LEFT JUSTIFIED
      for (radix=negative; position<sizeof(lcd_data); radix++,position++){
          *(lcd_data+radix)=*(lcd_data+position);
          *(lcd_data+position)=' ';
      }
      // Adjust the number of reserved lcd char positions 
      if(number_of_chars<sizeof(lcd_data)) { radix=number_of_chars; }  
      // Display the number on the LCD screen 
      for(position=0; position<radix; position++){ lcd_putc(lcd_data[position]); }

   }else{   // RIGHT JUSTIFIED
            if( (int8_t)x-number_of_chars >= 0){ lcd_gotoxy(x-number_of_chars, y); }else{ x = 0; }
            // Adjust the number of reserved lcd char positions 
            if (digits_needed <= sizeof(lcd_data)){
               for(; digits_needed > 0; digits_needed--, position++){ lcd_putc(lcd_data[position]); }
            }  
         }

return;
}


/*######################################################################################################*/
/* clear a specific lcd line and set cursor at the start of the line */
void lcd_clrline(unsigned char line){
#if LCD_LINES == 1
  lcd_gotoxy(0,0);
  while(x<LCD_CHARS_PER_LINE) { lcd_putc(' '); }
  lcd_gotoxy(0,0);
#elif (LCD_LINES == 2 || LCD_LINES == 4)  
  if(line<LCD_LINES)
   {
      lcd_gotoxy(0,line);
      while(x<LCD_CHARS_PER_LINE) { lcd_putc(' '); }
      lcd_gotoxy(0,line);
   }   
#endif  /*  #if LCD_LINES == 1 -> #elif (LCD_LINES == 2 || LCD_LINES == 4) */

return;
}

/*######################################################################################################*/

unsigned int lcd_getxy(void)
{
/*
    The return value of the "lcd_getxy()" function is an integer,
    with the high byte containing the current line number (y) and the low byte 
    containing the char position in that line (x).
    If the lower byte has a value of 20 that means that you filled that line.
    This position result can only happen when no lcd reading is available.
    When lcd reading is available the maximum x == 19. 
*/

unsigned char lcd_address_x;
unsigned char lcd_address_y;
unsigned int  lcd_address = 0;

#if LCD_READ_REQUIRED == 1   

unsigned char address_counter=0;

// Get the lcd's AC //
address_counter=lcd_read(ADDRESS_COUNTER);
// Turn off the busy flag's bit so that the address only remains. 
address_counter &= (~(1<<LCD_BUSY)); 

if(address_counter<(LCD_START_LINE1+0x14)) { lcd_address_y=0; } 
else if(address_counter<(LCD_START_LINE3+0x14)) { lcd_address_y=2; } 
else if(address_counter<(LCD_START_LINE2+0x14)) { lcd_address_y=1; }
else lcd_address_y=3;

lcd_address_x=(address_counter-lcd_line_start[lcd_address_y]);

#else

lcd_address_y = y;
lcd_address_x = x;

#endif   // #if LCD_READ_REQUIRED == 0 -> #elif LCD_READ_REQUIRED == 1 

lcd_address = (lcd_address_y * 100)+lcd_address_x;
//lcd_address = lcd_address_y;
//lcd_address = (lcd_address<<8);
//lcd_address += lcd_address_x; 

return(lcd_address);
}

/*######################################################################################################*/
// print string on lcd (no auto linefeed)
void lcd_puts(const unsigned char *s){

unsigned char c;

    while( (c = *s++) ) { lcd_putc(c);  }

return;
}

/*######################################################################################################*/
// print string from program memory on lcd (no auto linefeed)
void lcd_puts_p(const unsigned char *progmem_s){

unsigned char c;

    while( (c = pgm_read_byte(progmem_s++)) ) { lcd_putc(c);  }


return;
}

/*######################################################################################################*/

void lcd_puts_e(unsigned char *eeprom_s)
/* print string from eeprom on lcd (no auto linefeed) */
{
    unsigned char c;
   
    while( (c=eeprom_read_byte(eeprom_s++))&& c!=0xFF ) { lcd_putc(c);  }

return;               
}

/*######################################################################################################*/

#if LCD_READ_REQUIRED == 1

unsigned char lcd_getc(void){


return(lcd_read(DATA));
}

/*######################################################################################################*/

unsigned char lcd_get_error(void){


return(lcd_error_detected);
}

/*######################################################################################################*/
void lcd_get_line(unsigned char line, unsigned char* s)
{
unsigned char x1=0, lcd_x=0, lcd_y=0;

lcd_x = x; 
lcd_y = y;
lcd_gotoxy(0,line); 
for(x1=0; x1<LCD_CHARS_PER_LINE; x1++){ *(s+x1)=lcd_read(DATA); }
x=lcd_x;
y=lcd_y;
lcd_gotoxy(x,y);

return;
}


/*######################################################################################################*/
void lcd_backup_scr(void)
{
unsigned char x1=0, x2=0, lcd_x=x, lcd_y=y;

for(x1=0; x1<LCD_LINES; x1++)
  {
     lcd_gotoxy(0,x1); 
     for(x2=0; x2<LCD_CHARS_PER_LINE; x2++)
       {

#if   LCD_BACKUP_LOCATION == 0
         eeprom_write_byte((lcd_backup[x1]+x2),lcd_read(DATA));
#elif LCD_BACKUP_LOCATION == 1
          *(lcd_backup[x1]+x2)=lcd_read(DATA);
#endif
       }
  }

x=lcd_x;
y=lcd_y;
lcd_gotoxy(x,y);

return;
}

/*######################################################################################################*/
void lcd_restore_scr(void)
{
unsigned char x1=0, x2=0, lcd_x=x, lcd_y=y;

lcd_clrscr();   
lcd_gotoxy(0,0);

for(x1=0; x1<LCD_LINES; x1++)
  {

     lcd_gotoxy(0,x1); 
     for(x2=0; x2<LCD_CHARS_PER_LINE; x2++)
       {
#if   LCD_BACKUP_LOCATION == 0
          lcd_putc( eeprom_read_byte((lcd_backup[x1]+x2)) );
#elif LCD_BACKUP_LOCATION == 1
          lcd_putc(*(lcd_backup[x1]+x2));
#endif
       }
  }

x=lcd_x;
y=lcd_y;
lcd_gotoxy(x,y);

return;
}

#endif  /* #if LCD_READ_REQUIRED == 1 */

/*######################################################################################################*/
/*                                LCD INITIALIZATION FUNCTION                                           */
/*######################################################################################################*/

void lcd_init(void)
{
/* initialize display and select type of cursor */
/* dispAttr: LCD_DISP_OFF, LCD_DISP_ON, LCD_DISP_ON_CURSOR, LCD_DISP_CURSOR_BLINK */

#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_1LINE 
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 
#endif 

#if LCD_PROTOCOL == 0
//lcd_init_lock = 1;
#if LCD_READ_REQUIRED == 1 || LCD_GROUND_RW_PIN_ALSO == 1 
LCD_RW_DDR_REG |= (1<<LCD_RW_PIN);
LCD_RW_OUT_REG &= ~(1<<LCD_RW_PIN);
#endif

LCD_E_DDR_REG |= (1<<LCD_E_PIN);
LCD_E_OUT_REG &= ~(1<<LCD_E_PIN);
LCD_RS_DDR_REG |= (1<<LCD_RS_PIN);
LCD_RS_OUT_REG &= (~(1<<LCD_RS_PIN));



LCD_D4_DDR_REG |= (1<<LCD_D4_PIN);
LCD_D5_DDR_REG |= (1<<LCD_D5_PIN);
LCD_D6_DDR_REG |= (1<<LCD_D6_PIN);
LCD_D7_DDR_REG |= (1<<LCD_D7_PIN);

LCD_D4_OUT_REG &= ~(1<<LCD_D4_PIN);
LCD_D5_OUT_REG &= ~(1<<LCD_D5_PIN);
LCD_D6_OUT_REG &= ~(1<<LCD_D6_PIN);
LCD_D7_OUT_REG &= ~(1<<LCD_D7_PIN);
#else

       I2C_START_TX(lcd_i2c_address);
       i2c_put_byte(0);    // lcd_e_low()
       i2c_stop();
#endif

/*------ Initialize lcd to 4 bit i/o mode -------*/
/* initial write to lcd is 8bit using delay since busy flag can't be checked here anyway */

    LCD_POWER_ON_DELAY();                          /* Wait 20 milliseconds  */
     
    lcd_write(LCD_FUNCTION_8BIT_1LINE, INIT_MODE);
    LCD_INIT_DELAY();                              /* Wait 5 milliseconds */
    
    lcd_write(LCD_FUNCTION_8BIT_1LINE, INIT_MODE);
    lcd_write(LCD_FUNCTION_8BIT_1LINE, INIT_MODE);
    lcd_write(LCD_FUNCTION_4BIT_1LINE, INIT_MODE);      /* set IO mode to 4bit */
 
    /* from now on the lcd accepts only 4 bit I/O, so we can use lcd_command() */    
    lcd_command(LCD_FUNCTION_DEFAULT);      /* function set: display lines  */
    lcd_command(LCD_DISP_OFF);              /* display off                  */
    lcd_command(LCD_CLEAR_SCREEN);          /* display clear                */
    lcd_command(LCD_MODE_DEFAULT);          /* set entry mode               */
    lcd_command(LCD_DISP_ON);               /* LCD DISPLAY ON (DEFAULT)     */

return;
}

/*
void lcd_init(void)
{


#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_1LINE 
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 
#endif 

#if LCD_PROTOCOL == 0
//lcd_init_lock = 1;
#if LCD_READ_REQUIRED == 1 || LCD_GROUND_RW_PIN_ALSO == 1 
LCD_RW_DDR_REG |= (1<<LCD_RW_PIN);
LCD_RW_OUT_REG &= ~(1<<LCD_RW_PIN);
#endif

LCD_E_DDR_REG |= (1<<LCD_E_PIN);
//LCD_E_OUT_REG |= (1<<LCD_E_PIN); 
LCD_E_OUT_REG &= ~(1<<LCD_E_PIN);
LCD_RS_DDR_REG |= (1<<LCD_RS_PIN);
LCD_RS_OUT_REG &= (~(1<<LCD_RS_PIN));



LCD_D4_DDR_REG |= (1<<LCD_D4_PIN);
LCD_D5_DDR_REG |= (1<<LCD_D5_PIN);
LCD_D6_DDR_REG |= (1<<LCD_D6_PIN);
LCD_D7_DDR_REG |= (1<<LCD_D7_PIN);

LCD_D4_OUT_REG &= ~(1<<LCD_D4_PIN);
LCD_D5_OUT_REG &= ~(1<<LCD_D5_PIN);
LCD_D6_OUT_REG &= ~(1<<LCD_D6_PIN);
LCD_D7_OUT_REG &= ~(1<<LCD_D7_PIN);

//------ Initialize lcd to 4 bit i/o mode -------
// initial write to lcd is 8bit using delay since busy flag can't be checked here anyway 

    LCD_POWER_ON_DELAY();                          // Wait 20 milliseconds  
     
    lcd_write(LCD_FUNCTION_8BIT_1LINE, INIT_MODE);
    LCD_INIT_DELAY();                              // Wait 5 milliseconds 
    
    lcd_write(LCD_FUNCTION_8BIT_1LINE, INIT_MODE);
    lcd_write(LCD_FUNCTION_8BIT_1LINE, INIT_MODE);
    lcd_write(LCD_FUNCTION_4BIT_1LINE, INIT_MODE);      // set IO mode to 4bit 
 
    // from now on the lcd accepts only 4 bit I/O, so we can use lcd_command()     
    lcd_command(LCD_FUNCTION_DEFAULT);      // function set: display lines 
    lcd_command(LCD_DISP_OFF);              // display off                 
    lcd_command(LCD_CLEAR_SCREEN);          // display clear               
    lcd_command(LCD_MODE_DEFAULT);          // set entry mode              
    lcd_command(LCD_DISP_ON);               // LCD DISPLAY ON (DEFAULT)    

#elif LCD_PROTOCOL == 1
       // INITIALIZATION 
       // Set clock and data pins as outputs, at low state and set rs value  
       LCD_POWER_ON_DELAY();  

       volatile uint8_t i2c_tx_data = 0; // reset the i2c data byte.
       i2c_tx_data |= (backlight<<LCD_BT_PIN);
       I2C_START_TX(lcd_i2c_address);
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) ); // lcd_e_high()
       i2c_tx_data |= LCD_FUNCTION_8BIT_1LINE;
       i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); // lcd_e_low()
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) );    // lcd_e_high()
       LCD_INIT_DELAY();
       i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); // lcd_e_low()
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) );    // lcd_e_high()
       LCD_INIT_DELAY();
       i2c_tx_data |= LCD_FUNCTION_4BIT_1LINE;
       i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); // lcd_e_low()
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) );    // lcd_e_high()
       LCD_INIT_DELAY();

       i2c_tx_data = LCD_FUNCTION_DEFAULT;
       i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); // lcd_e_low()
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) );    // lcd_e_high()
       LCD_INIT_DELAY();

       i2c_tx_data = LCD_DISP_OFF;
       i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); // lcd_e_low()
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) );    // lcd_e_high()
       LCD_INIT_DELAY();

       i2c_tx_data = LCD_CLEAR_SCREEN;
       i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); // lcd_e_low()
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) );    // lcd_e_high()
       LCD_INIT_DELAY();

       i2c_tx_data = LCD_MODE_DEFAULT;
       i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); // lcd_e_low()
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) );    // lcd_e_high()
       LCD_INIT_DELAY(); 

      i2c_tx_data = LCD_DISP_ON;
       i2c_put_byte( i2c_tx_data & (~(1<<LCD_E_PIN)) ); // lcd_e_low()
       i2c_put_byte( i2c_tx_data | (1<<LCD_E_PIN) );    // lcd_e_high()
       LCD_INIT_DELAY();


 
#endif

return;
}
*/

/*######################################################################################################*/
/*                                         T H E   E N D                                                */
/*######################################################################################################*/

