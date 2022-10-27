
/*
 * uSDX Triband Radio, OLED version.
 * 
 * Main feature of this software version is interrupt driven I2C routines. We can calculate the next Si5351 transmit information while
 * the I2C bus is busy.
 * 
 * !!! Hardware mod.  Jumpered pin 27 to 4 and 28 to 5 to get I2C signals to the OLED.  Arduino D2, D3 should not be enabled
 * 
 * My process:  compile, AVRDudess, file in user/ron/local/appdata/temp/arduino_build_xxxxxx/, pickit 2 as programmer
 * power up with pickit app to test?   !!! does pickit apply vpp when searching for a device ID?
 * 
 * 
 * Strip menus
 * Select moves through menu, makes editable with encoder ( entry highlighted ), double tap select next menu, exit back to normal
 * Strip menu area for cw decode text also.
 */

#include <Arduino.h>
#include <avr/interrupt.h>
#include <OLED1306_Basic.h>

#define ENC_A  6
#define ENC_B  7
#define SW_ADC A3

// enable the fonts used
extern unsigned char SmallFont[];
extern unsigned char MediumNumbers[];
extern unsigned char BigNumbers[];

#define ROW0 0
#define ROW1 8
#define ROW2  16
#define ROW3  24
#define ROW4  32
#define ROW5  40
#define ROW6  48
#define ROW7  56                  // my OLED is a little low in the faceplate window, this is a good row for diag messages

  OLED1306 LCD;


#define I2TBUFSIZE 32              // size power of 2.  max 256 as using 8 bit index
#define I2RBUFSIZE 2               // set to expected #of reads, power of 2.  Won't be doing any reads in this program.
#define I2INT_ENABLED 1            // 0 for polling in loop or timer, 1 for TWI interrupts

//  I2C buffers and indexes
unsigned int i2buf[I2TBUFSIZE];   // writes
uint8_t i2rbuf[I2RBUFSIZE];       // reads
volatile uint8_t i2in,i2out;
volatile uint8_t i2rin,i2rout;
volatile uint8_t  gi2state;
uint8_t i2done = 1;
uint8_t polling;

uint8_t rit_enabled;
uint32_t freq = 14100000UL;
uint16_t divider = 50;            // 40 meters 100, below 7 126, 30m 70,  
// int step_ = 100;               // trying fixed tuning steps per mode.  Maybe long press on select and exit to move 100kc
uint8_t transmitting;
int sw_adc;                    // request flag and result of ADC of the switches

//#define I2STATS                // see if the i2c interrupts are functioning and being useful in freeing up cpu time
#ifdef I2STATS
  uint16_t i2polls;
  uint16_t i2ints;
  uint16_t i2stalls;
#endif

 /* switch states */
#define NOTACTIVE 0
#define ARM 1
#define DTDELAY 2
#define FINI 3
#define TAP  4
#define DTAP 5
#define LONGPRESS 6
#define DBOUNCE 60

int sw_state[3] = {NOTACTIVE,NOTACTIVE,NOTACTIVE};   // state of the switches 

uint8_t volume =20;
uint8_t band = 1;
uint8_t mode = 2;
uint8_t kspeed = 12;
uint8_t attn = 0;

#define FREQ_U 0
#define MENU_U 1
uint8_t encoder_user;


#include "si5351_usdx.cpp"     // the si5351 code from the uSDX project, modified slightly
SI5351 si5351;


void setup() {

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(SW_ADC, INPUT);

  i2init();
  delay(1);
  LCD.InitLCD();
  LCD.clrScr();
  LCD.setFont(SmallFont);

  si5351.freq( freq, 0, 90, divider );
  display_freq();

   // some sign on info/help screen
  //LCD.print(F("K1URC wb2cba pe1nnz"), LEFT, ROW4);
  //LCD.print(F("TriBander SDX"), LEFT, ROW3); // 7944 flash, 169 bytes using print from flash, diffs +1374 flash  -24 ram
  LCD.gotoRowCol( 0,0 );                       // 6570 flash, 193 bytes using puts, diffs don't seem logical
  LCD.puts("SDX TriBander");
  LCD.gotoRowCol( 1,0 );
  LCD.puts("K1URC wb2cba pe1nnz");


}

void loop() {
static unsigned long tm;
static unsigned int sec;
int t;

  if( polling ){               // may need to finish an I2C transfer if the buffer became full and we needed to poll in i2send.
     if( i2done == 0 ){
        noInterrupts();        // interrupts and twi done flags may get out of sync causing a hang condition.
        polling = i2poll();    // polling will be active until the buffer is empty and done flag is set.
        interrupts();
     }
     else polling = 0;         // looks like the interrupts finished the buffer, we can just clear the polling flag.
  }

  t = encoder();
  if( t ){
     if( encoder_user == FREQ_U ) qsy( t );
     if( encoder_user == MENU_U ) strip_menu( 3+t );    // menu commands 2 and 4
  }

  if( sw_adc == -1 ) fake_it();   

  if( tm != millis()){          // 4/5 ms routines ( 20meg clock vs 16meg )
     tm = millis();
     button_state();
     sel_button();
     exit_button();
     enc_button();
     // button_testing();     

     if( ++sec == 60000 ){     // 4/5 minutes counter
        sec = 0;
        #ifdef I2STATS
           uint16_t a,b,c;
           noInterrupts();
           a = i2ints;  b = i2polls;  c = i2stalls;
           i2ints = i2polls = i2stalls = 0;
           interrupts();
           LCD.clrRow( 7 );
           LCD.printNumI( a, LEFT, ROW7 );       // should be biggest number, interrupts working
           LCD.printNumI( b, CENTER, ROW7 );     // some OLED functions may overfill our buffer, polling counts here
           LCD.printNumI( c, RIGHT, ROW7 );      // polling may cause int flags out of sync with i2state, counts here
        #endif
     }
  }

}




void fake_it(){        // !!! test code.   analog read of the switches, fake it for now
  
    sw_adc = analogRead( SW_ADC );
    LCD.printNumI( sw_adc, RIGHT, ROW6 );   // readings are 772, 932, 1023 for the 3 buttons
}

char read_buttons(){                        // 3 switches on an analog pin
int val;
static char last;

    if( digitalRead( SW_ADC ) == LOW ){     // no switch pressed
        last = 0;
        return 0;
    }
    if( sw_adc == -1 ) return last;         // no adc results yet, guess it is same as last time
    val = sw_adc;                           // save value, queue another read
    sw_adc = -1;
                                            // 860 983 are usdx values
    last = 0;                                        
    if( val > 680 ) last = 1;               // select sw
    if( val > 830 ) last = 2;               // 852   set lower as any sw bounce may false detect select instead of exit
    if( val > 977 ) last = 4;               // encoder sw
    return last;
}


void button_state(){                /* state machine running at 1ms rate */
int sw,st,i;
static int press_,nopress;

      sw = read_buttons();                 // only one button detected at a time    
      if( sw ) ++press_, nopress = 0;
      else ++nopress, press_= 0;
      
      /* switch state machine */
      for( i= 0; i < 3; ++i ){
         st= sw_state[i];      /* temp the array value to save typing */

         if( st == NOTACTIVE && (sw & 0x1) && press_ >= DBOUNCE ) st= ARM;
         if( st == FINI && nopress >= DBOUNCE ) st = NOTACTIVE;   /* reset state */

         /* double tap detect */
         if( st == ARM && nopress >= DBOUNCE/2 )     st= DTDELAY;
         if( st == ARM && (sw & 0x1 ) && press_ >= 10*DBOUNCE )  st= LONGPRESS; 
         if( st == DTDELAY && nopress >= 4*DBOUNCE ) st= TAP;
         if( st == DTDELAY && ( sw & 0x1 ) && press_ >= DBOUNCE )   st= DTAP;
         
         sw_state[i]= st;      
         sw >>= 1;   /* next switch */
      }        
}


/***************
void button_testing(){    // !!! test code
uint8_t i;

   for( i = 0; i < 3; ++i){
      if( sw_state[i] < TAP ) continue;
      LCD.gotoRowCol( 6,0 );
      LCD.putch( i + 0x30 );  LCD.putch(' '); 
      if( sw_state[i] == TAP ) LCD.puts("TAP "), strip_menu(0);
      if( sw_state[i] == DTAP ) LCD.puts("DTAP");
      if( sw_state[i] == LONGPRESS) LCD.puts("LONG");
      sw_state[i] = FINI;
   }  
}
********************/

void sel_button(){                    // strip menu entry
  
  if( sw_state[0] < TAP ) return;
  switch( sw_state[0] ){
     case DTAP:   strip_menu( 1 );     // no break;  coded so two quick taps will enter edit mode on a var whether detected as
     case TAP:    strip_menu( 1 );    break;      // a double tap or two single taps on the switch
     case LONGPRESS:  freq -= 100000; qsy(0); break;
  }
  sw_state[0] = FINI;
}

void exit_button(){                   // strip menu exit
  
  if( sw_state[1] < TAP ) return;
  switch( sw_state[1] ){
     case DTAP:   strip_menu( 3 );     // no break;
     case TAP:    strip_menu( 3 );    break;
     case LONGPRESS:  freq += 100000; qsy(0); break;   // 100k freq jumps on long press sel, exit
  }
  sw_state[1] = FINI;
}

void enc_button(){
  
  if( sw_state[2] < TAP ) return;
    switch( sw_state[2] ){
     case DTAP:        break;
     case TAP:         break;
     case LONGPRESS:               // quick entry to edit volume
        strip_menu(0); strip_menu(1); strip_menu(1);
     break;
  }  
  sw_state[2] = FINI;
}


int encoder(){         /* read encoder, return 1, 0, or -1 */
// static char mod;        /* encoder is divided by 4 because it has detents */
static char dir;        /* need same direction as last time, effective debounce */
static char last;       /* save the previous reading */
char new_;              /* this reading */
char b;

   if( transmitting ) return 0;
   
   new_ = (digitalRead(ENC_B) << 1 ) | digitalRead(ENC_A);
   if( new_ == last ) return 0;       /* no change */

   b = ( (last << 1) ^ new_ ) & 2;    /* direction 2 or 0 from xor of last shifted and new data */
   last = new_;
   if( b != dir ){
      dir = b;
      return 0;      /* require two in the same direction serves as debounce */
   }
  // mod = (mod + 1) & 3;       /* divide by 4 for encoder with detents */
  // if( mod != 0 ) return 0;

   return ( (dir == 2 ) ? 1: -1 );   /* swap defines ENC_A, ENC_B if it works backwards */
}


void qsy( int8_t f ){
int stp;

   stp = 100;      // !!! use 1000 for USB,LSB  100 for cw, 10 for RIT.  To tune ssb to odd freq will need to switch to cw, tune 
   freq = (int32_t)freq + f * stp;
   si5351.freq( freq, 0, 90, divider );
   display_freq();
   
}

void display_freq(){
int rem;
//char priv[2];

  // priv[0] = band_priv( freq );
  // priv[1] = 0;
   rem = freq % 1000;

   /****    Freq displayed on top 2 lines in yellow area of the OLED
    LCD.setFont(MediumNumbers);
    LCD.printNumI(freq/1000,4*12,ROW0,5,'/');
    LCD.setFont(SmallFont);
    LCD.printNumI(rem,9*12,ROW1,3,'0');
    //LCD.print( priv, RIGHT, ROW0 );
    if( rit_enabled ){
       LCD.clrRow( 0, 4*12, 4*12+6*4 );
       LCD.clrRow( 1, 4*12, 4*12+6*4 );
       LCD.print((char *)"RIT",4*12,ROW1 );      
    }
    ****/

    // display big numbers in the blue area of the screen
    // font widths/height are: small 6 x 8, Medium 12 x 16, Big 14 x 24
    LCD.setFont(BigNumbers);
    LCD.printNumI(freq/1000,5,ROW2,5,'/');
    LCD.setFont(MediumNumbers);
    LCD.printNumI(rem,5*14 + 5 + 3,ROW3,3,'0');
    LCD.setFont( SmallFont );                     // keep OLED in small text as the default font
    if( rit_enabled ){
       LCD.clrRow( 0, 4*12, 4*12+6*4 );       // !!!! fixup needed for this, or another way to show RIT is active
       LCD.clrRow( 1, 4*12, 4*12+6*4 );
       LCD.print((char *)"RIT",4*12,ROW1 );      
    }

    
}

// TWI interrupt version
// if twi interrupts enabled, then need a handler 
ISR(TWI_vect){
  i2poll();
  if( gi2state == 0 ) i2poll();   // needed to get out of state zero.
  #ifdef I2STATS
     ++i2ints;
  #endif
}

/*****  Non-blocking  I2C  functions   ******/

void i2init(){
  TWSR = 0;
  TWBR = 6;    //8  500k, 12 400k, 72 100k   for 16 meg clock. ((F_CPU/freq)-16)/2
               //12 500k, 17 400k, 72 125k   for 20 meg clock.  8 625k 6 700k
  TWDR = 0xFF;       // ?? why
  // PRR = 0;        // bit 7 should be cleared  &= 7F
  PRR &= 0x7F;
  TWSR = 1<<TWEN;
  i2done = 1;
}
// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200


// wait for previous transfer to finish if any, queue a start condition
void i2start( unsigned char adr ){
unsigned int dat;
uint8_t t;

  while( i2done == 0 ){             // wait for finish of previous transfer
     noInterrupts();
     t = i2out;                     // !!! what about reads on i2c, needs work here i2rin changing if reading
     interrupts();
     delayMicroseconds( 40 );       // i2out should be changing if have interrupts happening 400k 
     noInterrupts();
     if( t == i2out ){
        i2poll();                   // kick if stuck
        #ifdef I2STATS
          ++i2stalls;
        #endif  
     }
     interrupts();
  }
  dat = ( adr << 1 ) | ISTART;      // shift the address over and add the start flag
  i2send( dat );
  
}


void i2send( unsigned int data ){   // just save stuff in the buffer
uint8_t  next;
uint8_t  t;

  // check for buffer full
  next = (i2in + 1) & (I2TBUFSIZE-1);
  noInterrupts();
  t = i2out;
  interrupts();
  if( t == next ){
      polling = 1;          // may need to finish this transfer via polling as tranfer is bigger than our buffer
      #ifdef I2STATS
        ++i2polls;
      #endif
  }
  
  while( t == next ){ 
         noInterrupts(); 
         i2poll();                  // wait for i2out to move
         t = i2out;
         interrupts();
  }
  
  i2buf[i2in++] = data;
  i2in &= (I2TBUFSIZE - 1);
}

void i2stop( ){

   i2send( ISTOP );   // que a stop condition
   i2done = 0;
   noInterrupts();    // kick off sending this buffer
   i2poll();
   interrupts();
}


void i2flush(){  // call flush to empty out the buffer, waits on I2C transactions completed 
uint8_t  ex;

  ex = 1;
  while(ex){
     noInterrupts();
     ex = i2poll(); 
     interrupts();
  }
}

/***********    save some flash space, not doing I2C reads for this radio
// queue a read that will complete later
void i2queue_read( unsigned char adr, unsigned char reg, unsigned char qty ){
unsigned int dat;

   i2start( adr );
   i2send( reg );
   // i2stop();     // or repeated start
   dat = ((unsigned int)qty << 10) | ( adr << 1 ) | ISTART | 1;   // a start with the read bit set
   i2send( dat );
   i2stop();        // stop to complete the transaction
}

int i2read_int(){     // returns 2 values in i2c read queue as a signed integer
int data;

      if( i2rout == i2rin ) return 0;
      data = i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      data <<= 8;
      if( i2rout == i2rin ) return data;
      data |= i2rbuf[i2rout++];
      i2rout &= (I2RBUFSIZE-1);
      return data;
}

uint8_t i2available(){
uint8_t qty;

     qty = i2rin - i2rout;
     if( qty > I2RBUFSIZE ) qty += I2RBUFSIZE;    // some funky unsigned math
     return qty;
}
*********************/

// everything happens here.  Call this from loop or interrupt.  Only state that does not return immediately is the i2stop.

uint8_t i2poll(){    
static  uint8_t state = 0;
static unsigned int data;
static unsigned int read_qty;
static uint8_t first_read;

   
   switch( state ){
     
      case 0:      // idle state or between characters
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2TBUFSIZE - 1 );
         
           if( data & ISTART ){   // start
              if( data & 1 ) read_qty = data >> 10;     // read queued
              data &= 0xff;
              // set start condition
              TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (I2INT_ENABLED); 
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) | (I2INT_ENABLED);
              state = 3;
           }
           else{   // just data to send
              TWDR = data;
              TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
              state = 2;
           }
        }
        else TWCR = (1<<TWEN);      // stop interrupts, keep enabled
      // break;                     // no break, check for stop completion
      case 3:  // wait here for stop to clear. TWINT does not return to high and no interrupts happen on stop.
         if( state != 3 ) break;
         while( 1 ){
            if( (TWCR & (1<<TWSTO)) == 0 ){
               state = 0;
               i2done = 1;           // i2c transaction complete, buffer is free for next one
               break;
            }
         }
      break;
      
      case 1:  // test for start to clear, send saved data which has the device address
         if( (TWCR & (1<<TWINT)) ){
            state = ( data & 1 ) ? 4 : 2;    // read or write pending?
            first_read = 1;
            TWDR = data;
            TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);
         }
      break;
      case 2:  // test for done
         if( (TWCR & (1<<TWINT)) ){  
            state = 0;
         }
      break;
      case 4:  // non blocking read until count has expired
         if( (TWCR & (1<<TWINT)) ){
            // read data
            if( first_read ) first_read = 0;       // discard the 1st read, need 8 more clocks for real data
            else{
               i2rbuf[i2rin++] = TWDR;
               i2rin &= ( I2RBUFSIZE - 1 );
               if( --read_qty == 0 ) state = 0;    // done
            }
            
            if( read_qty ){                        // any left ?
               if( read_qty > 1 ) TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (I2INT_ENABLED);  // not the last read
               else TWCR = (1<<TWINT) | (1<<TWEN) | (I2INT_ENABLED);                            // nack the last read
            }
         }
      break;    
   }
   
   gi2state = state;
   if( i2in != i2out ) return (state + 8);
   else return state;
}


// strcpy_P
// simple menu with parallel arrays. Any values that do not fit in 0 to 255 will need to be scaled when used.
// example map( var,0,255,-128,127)
#define NUM_MENU 5
const char smenu[] PROGMEM = "Vol AttnBandModeKSpd            ";      // pad out to multiple of 4 fields
uint8_t *svar[NUM_MENU] = {&volume,&attn,&band,&mode,&kspeed};
uint8_t  smax[NUM_MENU] = {  255,   3,    2,     2,     25 } ;

// 2,4 encoder, 0 reset entry, 1 select, 3 exit
void strip_menu( int8_t command ){        
static uint8_t sel;
static uint8_t ed;
static uint8_t mode;   // 0 not active, 1 select var, 2 edit var

   if( command == 0 ) mode = sel = ed = 0;    // menu reset.  Maybe make this open volume for edit ?
   switch( mode ){
       case 0:
          LCD.clrRow(0);  LCD.clrRow(1);      // entry condition, clear area
       break;
       case 1:                                // changing selected var to edit
          if( command == 2 && sel != 0 ) --sel;
          if( command == 4 && sel < NUM_MENU-1 ) ++sel;
       break;
       case 2:                                // editing *svar[sel]
          if( command == 2 && *svar[sel] != 0 ) --*svar[sel];
          if( command == 4 && *svar[sel] < smax[sel] ) ++*svar[sel];
       break;
   }

   // commands common to all modes
   if( command == 1 && mode < 2 ) ++mode;
   if( command == 3 && mode != 0 ) --mode;
   ed = ( mode == 2 ) ? 1 : 0;

   if( mode ){
       strip_display(sel>>2, sel, ed );    // menu active
       encoder_user == MENU_U;
   }
   else{
       strip_display( sel>>2, 0xff, 0 );   // display menu with no inverse text
       encoder_user == FREQ_U;             // encoder now changes frequency
   }
  
}

// print 4 strings and values from somewhere in the strip menu text, offset will be a page number or group of 4
const char mode_str[] PROGMEM = "CW USBLSB";
void strip_display( uint8_t offset, uint8_t sel, uint8_t ed ){
int i,k;
uint8_t val;

   LCD.gotoRowCol( 0, 0 );
   for( i = 0; i < 4; ++i ){
       for(k = 0; k < 4; ++k ){
          if( sel == ( 4*offset + i ) ) LCD.invertText(1);
          else LCD.invertText(0);
          LCD.putch( pgm_read_byte( &smenu[4*i+k+16*offset]));     //? 16*offset
       }      
       LCD.putch(' ');
   }

   // LCD.clrRow( 1 );
   for( i = 0; i < 4; ++i ){
       val = *svar[i+4*offset];                  // offset for 2nd page 4 * offset
       if( ed && sel == ( 4*offset + i ) ) LCD.invertText(1);
       else LCD.invertText(0);
       // special cases
       if( i == 2 && offset == 0 ) val += 1;     // band display, 0,1,2 becomes band 1 2 and 3 
       if( i == 3 && offset == 0 ){              // text for mode
          LCD.gotoRowCol(1,i*6*5);
          for( k = 0; k < 3; ++k )LCD.putch( pgm_read_byte( &mode_str[3*mode+k] ));
       }
       else LCD.printNumI( val, i*6*5, ROW1,3,' ' );
   }

   LCD.invertText(0);
  
}
