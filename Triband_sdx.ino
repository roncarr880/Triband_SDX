
/*
 * uSDX Triband Radio, OLED version.
 * 
 * Main features of this software version: 
 *     Interrupt driven I2C routines. We can calculate the next Si5351 transmit information while the I2C bus is busy.
 *        In theory we may achieve a higher tx sampling rate than otherwise.
 *     Distributed receive processing with pipelined data flow.    
 *        In theory we will have more time to process more complicated receive algorithms.
 *     Good looking display layout.
 * 
 * !!! Hardware mod.  The Si5351 module was NOT converted to run 3.3 volts I2C signals. 
 *     Jumpered pin 27 to 4 and 28 to 5 to get I2C signals to the OLED.  Arduino D2, D3 should not be enabled
 *     I am running 5 volt I2C signals !!!  Standard is to run 3.3 volt I2C.  DO NOT load and give this program a try 
 *     without using a 5 volt Si5351 module and adding the jumper wires OR add level converters as follows:
 *     
 *     !!! Do not use jumpers if your Si5351 is running with 3.3 volt I2C, Si5351 soldered to the board or Modified module.  
 *     Add level converters instead for SDA, SCL, 3.3 side is ATMega328, 5 volt side is OLED. 
 *     Pin 27 to level converter to pin 4.   Pin 28 to level converter to pin 5.
 *     The Mega328 will now run 3.3 volt I2C, the OLED will be at 5 volt, the Si5351 stays at 3.3 volt
 *     This is an untested idea. Should work, OLED I2C baud is 125k.
 * 
 * My process:  compile, AVRDudess, file in user/ron/local/appdata/temp/arduino_build_xxxxxx/, pickit 2 as programmer
 *  ? power up with pickit app to test ?  
 *  does pickit app apply 12v vpp when searching for a device ID?  ( it applies 5v a few times, 9 volts, then 12 volts ).
 *          !!!!!!!!!!  Yes, so don't power the radio with the Pickit2 App. !!!!!!!!!!!
 * 
 * Button Functions,  buttons respond to TAP, Double TAP, and Long Press
 *   Select: Tap opens menu, another Tap enables edit , DTap opens menu in edit mode, Long Press qsy's -100kc
 *   Exit  : Tap backs out of menu one step, DTap backs out all the way, Long Press qsy's +100kc
 *   Encoder: Long Press opens Volume directly for edit.
 *            Tap changes tuning step 
 *            Dtap  ( unassigned )
 *            Any press to enable TX after band change, CHECK the switches first!, clears message on screen.
 *            Any press to turn off RIT
 *   
 */
/**********
 MIT License

Copyright (c) 2022 Ron Carr

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*******************/


#include <Arduino.h>
#include <avr/interrupt.h>
#include <OLED1306_Basic.h>
#include <EEPROM.h>
#include "sine_cosine.h"

#define ENC_A  6
#define ENC_B  7
#define SW_ADC A3
#define AUDIO_PIN 9             // PWM pins
#define TXMOD_PIN 10
#define AUDIO  0b10000000       // TCCR1A values
#define TXMOD  0b00100000
#define RX     2                // TIMSK2 interrupt enable bits, for timer2 setup
#define TX     4

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

#define CW  0
#define USB 1
#define LSB 2  

#define I2TBUFSIZE 32              // size power of 2.  max 256 as using 8 bit index
#define I2RBUFSIZE 2               // set to expected #of reads, power of 2.  Won't be doing any reads in this program.
#define I2INT_ENABLED 1            // 0 for polling in loop or timer, 1 for TWI interrupts

//  I2C buffers and indexes
unsigned int i2buf[I2TBUFSIZE];   // writes
uint8_t i2rbuf[I2RBUFSIZE];       // reads
volatile uint8_t i2in,i2out;
volatile uint8_t i2rin,i2rout;
//volatile uint8_t  gi2state;
uint8_t i2done = 1;
uint8_t polling;

uint8_t rit_enabled;
uint32_t freq;
uint16_t divider;  
uint8_t transmitting;
int sw_adc;                    // request flag and result of ADC of the switches

uint32_t bandstack[3] = { 21100000UL, 10106000UL, 7100000UL };

#define I2STATS                // see if the i2c interrupts are functioning and being useful in freeing up cpu time
#ifdef I2STATS
  uint16_t i2polls;
  uint16_t i2ints;
  uint16_t i2stalls;
  uint16_t rx_overruns;       //  Added other debug counters non-i2c related
  uint16_t rx_underruns;  
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

//   variables in the strip menu
uint8_t volume = 20;
uint8_t attn = 0;
uint8_t band = 2;        // bands here are numbered 0,1,2    Display as 1,2,3   2 == band 3
uint8_t mode = 1;
uint8_t kspeed = 12;
uint8_t kmode = 1;
uint8_t side_vol = 22;
uint8_t vox;
uint8_t cal = 128;         // final frequency calibration +-500hz
uint8_t Pmin = 0;
uint8_t Pmax = 30;
//uint8_t dv;              // dummy var menu placeholder

uint8_t ad_ref;                    // some bits to merge with the A/D mux bits to select the reference voltage
uint8_t oldband;                   // for saving freq in bandstack on band change
uint16_t eeprom_write_pending;     // delay eeprom writes while the user fiddles the values

// receiver vars
#define RXPIPE  32                                 // power of 2 buffer size.
volatile int Qr[RXPIPE], Ir[RXPIPE];               // post processing sample delay pipeline
volatile uint8_t rx_val[RXPIPE];                   // PWM data buffer
volatile uint8_t rx_process_flag;                  // receiver audio lags behind the sampling function up to RXPIPE size.
volatile uint8_t rx_ready_flag;                    // currently it runs about 8 to 10 samples behind

#define FREQ_U 0               // encoder users, tuning and menu
#define MENU_U 1
uint8_t encoder_user;
int     step_ = 1000;
uint8_t tx_inhibit = 1;        // start with a band switches check message before transmit


#include "si5351_usdx.cpp"     // the si5351 code from the uSDX project, modified slightly for calibrate and RIT
SI5351 si5351;

const char msg1[] PROGMEM = "Check Band Switches";
const char msg2[] PROGMEM = "RIT Enabled";
const char msg3[] PROGMEM = "SDX TriBander";
const char msg4[] PROGMEM = "K1URC wb2cba pe1nnz";
const char msg5[] PROGMEM = "EEPROM ? Check vars";


void setup() {

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(SW_ADC, INPUT);
  pinMode(TXMOD_PIN, OUTPUT );
  digitalWrite(TXMOD_PIN, LOW );           // ? vaguely remember something about leaving this high for better cw shaping
  pinMode( AUDIO_PIN, OUTPUT );
  digitalWrite( AUDIO_PIN, LOW );          // if thumps try the floating the pin as in timer1 startup notes

  i2init();
  LCD.InitLCD();
  LCD.clrScr();
  LCD.setFont(SmallFont);

  oldband = band;
  strip_restore();                         // get var data from eeprom
  freq = bandstack[band];                  // simple bandstack, only freq is saved for each band
  qsy(0);                                  // set Si5351, display freq

  p_msg( msg3,0 );                         // sign on messages
  p_msg( msg4,1 );
  p_msg( msg1,6 );                         // display band switch check message, manual switches for Tribander
  
  analogRead( SW_ADC );                    // enable the ADC the easy way?
  // PRR &= ~( 1 << PRADC );               // adc power reduction bit should be clear

  set_timer1( AUDIO );                     // enable pwm audio out
  set_timer2( RX );                        // start receiver

}


// would something like this work, pointers to PROGMEM
// using print with flash helper (F("Signon message")) adds 1600 bytes to program usage
void p_msg( const char *ptr, int row ){
char c;

   LCD.clrRow( row );
   LCD.gotoRowCol( row, 0 );
   while( ( c = pgm_read_byte(ptr++) ) ) LCD.putch(c);
   LCD.putch(' ');                      // make sure at least one write after gotoRowCol
  
}

void loop() {
static unsigned long tm;
static unsigned int sec;
static uint8_t robin;          // round robin some processing, not sure this is needed
int t;

 // high priority.
  while( rx_process_flag ) rx_process2();
 
  if( polling ){               // may need to finish an I2C transfer if the buffer became full and we needed to poll in i2send.
     if( i2done == 0 ){        // a bigger buffer may help,  but screen writes send a bunch of data
        noInterrupts();        // interrupts and twi done flags may get out of sync causing a hang condition.
        polling = i2poll();    // polling will be active until the buffer is empty and done flag is set.
        interrupts();
     }
     else polling = 0;         // looks like the interrupts finished the buffer, we can just clear the polling flag.
  }

  t = encoder();               // returns -1,0,1
  if( t ){
     if( encoder_user == FREQ_U ) qsy( t );
     if( encoder_user == MENU_U ) strip_menu( 3+t );    // menu commands 2 and 4
  }

  if( transmitting && mode == CW ) sidetone();

  // round robin 1ms processing. Process each on a different loop
  if( robin == 1 ) button_state(), ++robin;
  if( robin == 2 ) sel_button(), ++robin;
  if( robin == 3 ) exit_button(), ++robin;
  if( robin == 4 ) enc_button(), robin = 0;


  if( tm != millis()){          // 1 ms routines ( a quick 1ms - 20meg clock vs 16meg )
     tm = millis();
     robin = 1;

     if( eeprom_write_pending ){
        if( --eeprom_write_pending == 0 ) strip_save();
     }

     if( ++sec == 15000 ){
        sec = 0;
        #ifdef I2STATS
          print_i2stats();
        #endif
     }
  }           /** end one ms routines **/

}


char read_buttons(){                        // 3 switches on an analog pin
int val;
static char last;

    if( digitalRead( SW_ADC ) == LOW ){     // no switch pressed
        last = 0;
        return 0;
    }
    last = 8;                               // get the vref up to 5 volts, 8 == invalid switch
    if( sw_adc == -1 ) return last;         // no adc results yet, guess it is same as last time
    val = sw_adc;                           // save value, queue another read
    sw_adc = -1;
                                            // 860 983 are usdx values for breakpoints in voltage read
    // last = 0;                                        
    if( val > 680 ) last = 1;               // select sw
    if( val > 830 ) last = 2;               // 852   set lower as any sw bounce may false detect select instead of exit
    if( val > 977 ) last = 4;               // encoder sw
    return last;
}


void button_state(){                /* state machine running at 1ms rate */
int sw,st,i;
static int press_,nopress;
int8_t sw_true;

      sw = read_buttons();                 // only one button detected at a time    
      if( sw & 7 ) ++press_, nopress = 0;  // check only valid switch
      else ++nopress, press_= 0;
      sw_true = sw;                        // value 8 or valid switch will make vref 5 volts
      
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
         sw_true |= st;                     // a switch is active or was active
         sw >>= 1;   /* next switch */
      }
      // maybe a funny place to set the A/D reference voltage, but we need 5 volts when the switches are active
      ad_ref = 0x40 | 0x80;                                // assume 1.1 volt
      if( sw_true ) ad_ref = 0x40;                         // need 5 volt reference to read the switches
      else if( vox == 0 && ( attn & 1 ) ) ad_ref = 0x40;   // attn setting if vox inactive, vox uses 1.1 volt

      //ad_ref = 0x40;    // !!!! debug, force 5 volts
          
}


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

    if( tx_inhibit ){                               // any encoder press to enable transmit after band change
        tx_inhibit = 0;
        LCD.clrRow(6);                              // clear help message                    
    }
    else if( rit_enabled ){                         // any encoder press to cancel RIT
       rit_enabled = 0;                             // returns to rx freq as we don't save the tx freq, just in si5351 registers
       freq = freq - freq % step_;                  // clear out LSDigits
       qsy(0);
       LCD.clrRow(6);
    }
    else{
       switch( sw_state[2] ){
        case DTAP:        break;                                     // !!! unused function available
        case TAP:  step_ = ( step_ == 100 ) ? 1000 : 100;  break;    // toggle tuning step size
        case LONGPRESS:                                              // quick entry to edit volume
           strip_menu(0); strip_menu(1); strip_menu(1);
        break;
       }
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

   if( rit_enabled ) freq = (int32_t)freq + f * 10;        // tune by 10 hz
   else freq = (int32_t)freq + f * step_;
   calc_divider();
   if( mode == LSB ) si5351.freq( freq, 0, 90, divider );
   else si5351.freq( freq, 90, 0, divider );
   display_freq();
   
}

void calc_divider(){
uint32_t  f;

   f = freq/1000000;
   divider = 126;
   if( f > 6 ) divider = 100;
   if( f > 9 ) divider = 66;
   if( f > 12 ) divider = 50;
   if( f > 17 ) divider = 36;
   if( f > 25 ) divider = 28;
}


void display_freq(){
int rem;
//char priv[2];

  // priv[0] = band_priv( freq );
  // priv[1] = 0;
   rem = freq % 1000;

    // display big numbers in the blue area of the screen
    // font widths/height are: small 6 x 8, Medium 12 x 16, Big 14 x 24
    LCD.setFont(BigNumbers);
    LCD.printNumI(freq/1000,5,ROW2,5,'/');
    LCD.setFont(MediumNumbers);
    LCD.printNumI(rem,5*14 + 5 + 3,ROW3,3,'0');
    LCD.setFont( SmallFont );                     // keep OLED in small text as the default font
    if( rit_enabled ) p_msg( msg2, 6 );           // RIT message
 
}


/*****  Non-blocking  I2C  functions, interrupt driven  ******/

// TWI interrupt version
// if twi interrupts enabled, then need a handler 
ISR(TWI_vect){
  i2poll();
  //if( gi2state == 0 ) i2poll();   // needed to get out of state zero. Fixed this issue I think.
  #ifdef I2STATS
     ++i2ints;
  #endif
}


//  found that a slower baud allows the receive process to run better.  Switch to a high baud when transmitting.
void i2init(){
  TWSR = 0;
  TWBR = 72;    //8  500k, 12 400k, 72 100k   for 16 meg clock. ((F_CPU/freq)-16)/2
                //12 500k, 17 400k, 72 125k   for 20 meg clock.  8 625k 6 700k  ( 4 seems to work ok 833k )
  TWDR = 0xFF;       // ?? why
  PRR &= 0x7F;
  TWSR = 1<<TWEN;
  i2done = 1;
}
// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200


// single buffered, wait for previous transfer to finish if any, queue a start condition
void i2start( unsigned char adr ){
unsigned int dat;
uint8_t t;

  while( i2done == 0 ){             // wait for finish of previous transfer
     noInterrupts();
     t = i2out;                     // see if interrupts are clearing the buffer, quickest way
     interrupts();
     my_delay(40);                  //40us for 400k baud
     noInterrupts();
     if( t == i2out ){              // should be changing
        i2poll();                   // kick if stuck
        #ifdef I2STATS
          ++i2stalls;
        #endif  
     }
     interrupts();
  }
  dat = ( adr << 1 ) | ISTART;      // shift the address over and add the start flag
  i2send( dat );
  // change baud rate here for si5351 vs OLED ?  17 or 6
  // may be better to just use the transmitting variable or both
  // lets try it.  And find using 125k does free up processing time for the receiver, less interrupt overhead probably
  //TWBR = ( adr == 0x60 ) ? 4 : 72;  // oled lower priority, we shouldn't write oled while transmitting
   TWBR = ( transmitting ) ? 3 : 72;  // oled lower priority, we shouldn't be writing the oled while transmitting
  
}

void my_delay( uint32_t val ){                  // delay micros with yield type of feature
uint32_t tm;

    if( transmitting ) return;                  // rx process not running
    val *= 4;                                   // for 125k baud delay
    tm = micros();
    while( micros() - tm < val ){
        if( rx_process_flag ) rx_process2();    // run receive processing while waiting on the I2C bus
    }

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
      polling = 1;          // may need to finish this transfer via polling as transfer is bigger than our buffer
      #ifdef I2STATS
        ++i2polls;
      #endif
  }
  
  while( t == next ){       // the buffer is full, call poll to send some of the data. Some OLED writes are bigger than the buffer.
         noInterrupts(); 
         i2poll();                  // wait for i2out to move
         t = i2out;
         interrupts();
         if( rx_process_flag ) rx_process2();    // keep the receiver going
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

// everything happens here.  Call this from loop or interrupt.
// Only state that does not return immediately is the i2stop.  It does not produce an interrupt.
// modified to take some of the cases out of the switch construct so they always get parsed.
// Fixes the need to double poll when in state 0. 

uint8_t i2poll(){    
static  uint8_t state = 0;
static unsigned int data;
static unsigned int read_qty;
static uint8_t first_read;

   
   switch( state ){
           
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

   // always test these conditions
   //      case 0:      // idle state or between characters
   if( state == 0 ){
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
   }

   if( state == 3 ){
      //case 3:  // wait here for stop to clear. TWINT does not return to high and no interrupts happen on stop.
      //   if( state != 3 ) break;
         while( 1 ){
            if( (TWCR & (1<<TWSTO)) == 0 ){
               state = 0;
               i2done = 1;           // i2c transaction complete, buffer is free for next one
               break;
            }
         }
      //break;
   }
   
   if( i2in != i2out ) return (state + 8);
   else return state;
}


/****************************  
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
***********************************/

/*********** end I2C functions  ************/


// simple menu with parallel arrays. Any values that do not fit in 0 to 255 will need to be scaled when used.
// example map( var,0,255,-128,127)
#define NUM_MENU 11
const char smenu[] PROGMEM = "Vol AttnBandModeKSpdKmdeSvolVox Cal PminPmax    ";      // pad spaces out to multiple of 4 fields
uint8_t *svar[NUM_MENU] = {&volume,&attn,&band,&mode,&kspeed,&kmode,&side_vol,&vox,&cal,&Pmin,&Pmax};
uint8_t  smax[NUM_MENU] = {  63,   3,    2,     2,     25,    3,     63,       1,   254,  20,    63 };
uint32_t stripee = 0b00000000000000000000011100100000;                         // bit set for strip menu items to save in eeprom

// note: a smax value of 255 will not be restored from eeprom, it looks like erased data.
//       smax of 1 allows two values for the variable.  Max for band (2) is 3 bands 0,1,2
//       the zero value for cal is 128 
// commands: 2,4 encoder, 0 reset entry, 1 select, 3 exit
void strip_menu( int8_t command ){        
static uint8_t sel;
static uint8_t ed;
static uint8_t mode;   // 0 not active, 1 select var, 2 edit var
static uint8_t hyper;

   // slow down encoder inputs
   hyper ^= 1;
   if( hyper && (command == 2 || command == 4) ) return;   // half speed encoder

   if( command == 0 ) mode = sel = ed = 0;    // menu reset
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

   // commands common to all modes.  mode here is menu mode and not the radio mode.
   if( command == 1 && mode < 2 ) ++mode;
   if( command == 3 && mode != 0 ) --mode;
   ed = ( mode == 2 ) ? 1 : 0;

   if( mode ){
       strip_display(sel>>2, sel, ed );    // menu active
       encoder_user = MENU_U;
   }
   else{
       strip_display( sel>>2, 0xff, 0 );   // display menu with no inverse text
       encoder_user = FREQ_U;              // encoder now changes frequency
   }

   if( (command == 2 || command == 4) && ed ) strip_post( sel );
  
}

// print 4 strings and values from somewhere in the strip menu text, offset will be a page number or group of 4
const char mode_str[] PROGMEM = "CW USBLSB";
void strip_display( uint8_t offset, uint8_t sel, uint8_t ed ){
int i,k;
uint8_t val;

   if( ed == 0 ){                                  // skip extra OLED writes, ed mode only changes numbers
      LCD.gotoRowCol( 0, 0 );                      // print header row
      for( i = 0; i < 4; ++i ){
          if( sel == ( 4*offset + i ) ) LCD.invertText(1);
          for(k = 0; k < 4; ++k ) LCD.putch( pgm_read_byte( &smenu[4*i+k+16*offset]));
          LCD.invertText(0);
          LCD.putch(' ');
      }
   }  

   for( i = 0; i < 4; ++i ){                         // print data row
       if( ( i+4*offset ) >= NUM_MENU ) val = 0;     // unused variable placeholder to show as zero's
       else val = *svar[i+4*offset];                 // offset for 2nd page 4 * offset
       if( ed && sel == ( 4*offset + i ) ) LCD.invertText(1);
       // special cases
       if( offset == 0 && i == 2 ) val += 1;     // band display, 0,1,2 becomes band 1 2 and 3 
       if( offset == 0 && i == 3 ){              // text for mode
          LCD.gotoRowCol(1,i*6*5);
          for( k = 0; k < 3; ++k )LCD.putch( pgm_read_byte( &mode_str[3*mode+k] ));
       }
       else LCD.printNumI( val, i*6*5, ROW1,3,' ' );
       LCD.invertText(0);
   }
  
}


void strip_post( uint8_t sel ){    // do any post processing needed
int i;

   // LCD.clrRow( 6 );             // help message area
   // LCD.gotoRowCol( 6,0 );       // having this here with no writes to follow causes a program hang
                                   // OLED left in command mode ?  goto is one of my "enhancements" to the library.
   
   switch( sel ){
      case 2:                      // band change
        bandstack[oldband] = freq;
        freq = bandstack[band];
        oldband = band;
        p_msg( msg1, 6 );          // Check band switches message
        tx_inhibit = 1;
      //break;
      case 3: rit_enabled = 0;  qsy(0);  break;    // mode change.  clear RIT or Si5351 will not be reset to new dividers
      case 8:  qsy(0);  break;                     // calibrate, implement freq change for user observation
   }
   if( stripee & ( 1 << sel ) ) eeprom_write_pending = 62000;   // 62 quick seconds until eeprom write
  
}

void strip_save(){             // save items to eeprom.  Docs say if item hasn't changed it won't wear out the eeprom
int i;                         // so we will just save all flagged as eeprom data

   for( i = 0; i < NUM_MENU; ++i ){
      if( stripee & ( 1 << i )){           // item flagged as one to save
         EEPROM.put(i,*svar[i]);
      }
   }
}

void strip_restore(){          // called from setup(),  restore items from eeprom.  If data is 0xff we ignore.
int i;
uint8_t val;

   for( i = 0; i < NUM_MENU; ++i ){
      if( stripee & ( 1 << i )){
         EEPROM.get( i, val );
         if( val != 0xff ){
            if( val > smax[i] ){      // eeprom not valid, maybe some changes were made to the menu
                p_msg( msg5, 6 );
                delay(5000);          // message will be overwritten in 5 seconds
                return;               // abort restore
            }
            *svar[i] = val;
         }
      }
   }
}

/*******************  end menu *********************/

/*****    Radio processing   *****/

void set_timer1( uint8_t clk ){                // timer 1 is set for 8 bit fixed mode 78k PWM
                                               // enable both PWM's for CW so can hear sidetone
   noInterrupts();
   if( clk == 0 ){                             // turn off
      // pinMode( AUDIO_PIN, INPUT_PULLUP );     // ? useful at all to float the audio pin or should we just leave it as
       TCCR1A = 0;                             // an output pin.  It would make more sense to do this if audio is muted.
       TCCR1B = 0;
   }
   else{

      // no prescale, function arg clk controls what clock is on, WGM13-10 = 0101 fast 8 bit mode 5  ( usdx uses mode 14 )
      // OCR1AL controls audio,  OCR1BL controls transmit pwm.
   PRR &= ~(1 << PRTIM1 );
   OCR1AH = 0;                               // the temp high byte register is saved for these next low byte writes
   OCR1AL = 128;  OCR1BL = 0;  TCNT1L = 0;   // or should OCR1BL = Pmin ?
   TCCR1A = clk | 1;
   TCCR1B = 0b00001001;
  // pinMode( AUDIO_PIN, OUTPUT );
   }
   interrupts();
}

void set_timer2( int8_t mode){

  noInterrupts();
  TCCR2B = 0;                        // stop
  TCNT2 = 0;                         // clear counter
  PRR &= ~(1 << PRTIM2 );            // powered up
  TCCR2A = 2;                        // CTC mode
  TIMSK2 = mode;                     // enable interrupts A match or B match, RX TX defines
                                     // OCR2A = (F_CPU / pre-scaler / fs) - 1;
  if( mode == RX ) OCR2A = 51;       // count to value, resets, interrupts, fs = 47786
  if( mode == TX ) OCR2A = 226;      // fs = 11000
  OCR2B = OCR2A - 1;                 // use the 2nd interrupt for transmit
  TCCR2B = 2;                        // start clocks, divide by 8 clock prescale
  interrupts();
}


ISR(TIMER2_COMPA_vect){               // Timer2 COMPA interrupt

   rx_process();  
}

ISR(TIMER2_COMPB_vect){               // Timer2 COMPB interrupt

   if( mode == CW ) ;                 // goto cw functions
   else tx_process();                  
}

/*******   tx functions   *******/

// a 31 tap classic hilbert every other constant is zero, kaiser window
int16_t valq, vali;                                 // results of hilbert filter are global
void tx_hilbert( int16_t val ){
static int16_t wi[31];                              // delay terms
int16_t t;

const int16_t k0 = (int16_t)( 512.5 * 0.002972769320862211 );
const int16_t k1 = (int16_t)( 512.5 * 0.008171666650726522 );
const int16_t k2 = (int16_t)( 512.5 * 0.017465643081957562 );
const int16_t k3 = (int16_t)( 512.5 * 0.032878923709314147 );
const int16_t k4 = (int16_t)( 512.5 * 0.058021930268698417 );
const int16_t k5 = (int16_t)( 512.5 * 0.101629404192315698 );
const int16_t k6 = (int16_t)( 512.5 * 0.195583262432201366 );
const int16_t k7 = (int16_t)( 512.5 * 0.629544595185021816 );

   // val = constrain(val,-64,63);                     // avoid overflow, tx clipper
   // think it will handle 10 bits in with the adjustment on k7 multiply
   for( int i = 0; i < 30; ++i )  wi[i] = wi[i+1];
   wi[30] = val;

   valq = wi[15];
   vali =  k0 * ( wi[0] - wi[30] ) + k1 * ( wi[2] - wi[28] ) + k2 * ( wi[4] - wi[26] ) + k3 * ( wi[6] - wi[24] );
   vali += k4 * ( wi[8] - wi[22] ) + k5 * ( wi[10] - wi[20]) + k6 * ( wi[12] - wi[18]);  // + k7 * ( wi[14] - wi[16]);
   vali >>= 9;
   t = k7/4 * ((wi[14] - wi[16]) >> 2);
   vali += (t >> 7);
}




#define TX_REF (0x40 | 0x80)                 // 1.1 volt reference
#define _UA   5500                           // can use arbitrary number, but make same as sample rate

void tx_process(){
int cin;
static uint8_t R;
static int16_t y1;
static int16_t y2, y1d;
static int16_t mag;
static int prev_phase;
int dp, ph;

       cin = ADC - 512;
       ADMUX = TX_REF | 2;                   // I think we can ignore all button inputs during tx
       ADCSRA = 0xc0 + 5;                    // next start conversion

       // one stage of CIC, final rate is half of interrupt rate
       y1 = y1 + cin;                        // integrator
       R ^= 1;
       if( R ){                              // precalc mag on this interrupt
          mag *= Pmax;                       // slope, Pmax is 0 to 63 to stay in 16 bits
          mag >>= 6;
          mag += Pmin;                       // y intercept
          mag = constrain( mag, 0, 255 );    // clip to 8 bits
          return;
       }

       OCR1BL = mag;                         // write mag here for 1 sample delay ?
       
       y2 = y1 - y1d;   y1d = y1;            // comb length 2
       y2 >>= 2;
       tx_hilbert( y2 );                     // get valq and vali
       mag = fastAM( vali, valq ) >> 2;      // to 8 bits
       ph  = arctan3( valq, vali );          // phase

       dp = ph - prev_phase;                 // delta phase
       prev_phase = ph;
       if( dp < -_UA/2 ) dp += _UA;
       if( mode == LSB ) dp = -dp;
       dp = constrain(dp,-3000,3000);

       si5351.freq_calc_fast(dp);
       while( i2done == 0 ){                 // should be ready but need to wait if not
          noInterrupts();
          i2poll();
          interrupts();
       }
       si5351.SendPLLBRegisterBulk();
}

int16_t fastAM( int16_t i, int16_t q ){           // estimation of sqrt( i^2 + q^2 )

  /*
    // quicker version   max + min/4
       i = abs(i);   q = abs(q);  
       if( i > q ) q >>= 2;
       else i >>= 2;
       return ( i + q );
  */
     // better accuracy version   max or 7/8 max + 1/2 min
int16_t mb4; 

    i = abs(i), q = abs(q);
    if( i > q ){
       mb4 = i;   i = q;  q = mb4;
    }
    mb4 = q >> 2;
    if( i < mb4 ) return q;
    q -= ( mb4 >> 1 );                     // 7/8 * q == q - 1/8q
    i = i >> 1;
    
    return ( i + q );  
}


int16_t arctan3( int16_t q, int16_t i ){           // from QCX-SSB code

  #define _atan2(z)  (((_UA/8 + _UA/22) - _UA/22 * z ) * z)     //uSDX original derived from equation 5 [1].
  
  int16_t r;
  int16_t ai, aq;

  ai = abs(i);  aq = abs(q);
  if( aq > ai )   r = _UA / 4 - _atan2( ai / aq );  // arctan(z) = 90-arctan(1/z)
  else r = (i == 0) ? 0 : _atan2( aq / ai );        // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                    // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                          // arctan(-z) = -arctan(z)
}



 

/****
void audio_out( int16_t val ){        // clip/saturate values to 8 bits
 
   OCR1AL = constrain( val + 128, 0, 255 );
}
***/


void sidetone(){                     // can we generate an ok sidetone from just loop
static uint32_t tm;
static uint8_t  phase;
int16_t s;

   if( micros() - tm < 416 ) return;     // 3000 hz sample time, 333 for 16 mhz but 416 for 20 mhz clock
   
   tm = micros();                        //  sin table is 64 entries for a full wave
   phase =  (phase+13) & 63;             //  phase update is tone *  64 / freq_samp .   600 * 64/3000 = 12.8
                                 
   //s = pgm_read_word( &sin_cos[phase] ); //  sin table is in q12 format
   //s >>= 5;           // divide q12 number by 32, keep numbers in range of int16 or switch to long data types
                        // have -128 to 128 as sine value
                        
   s = pgm_read_byte( &sin_cos[phase] );   // q6 format, have -64 to 64, half volume, can shift left if want full volume
   s *= side_vol;
   s >>= 6;                                // divide by 64, max volume
   OCR1AL = (uint8_t)(s + 128);
}



/*********   RX functions ***********/

// thoughts on sin table phase updates and final freq sampling rate.  Thinking for Weaver mode RX.
// I think it would not be good to have a phase update that is a divisor of 64, only a couple of values would be used from the table
//  pretend A/D result Q7.3 -128 + 127  Q7.3 is 10 bits? or Q7.2 with sign bit?
// Q12 * Q3 is Q15 so shift down 15 after mult???  sign bit???  1 * 5 would be 4096*40
// think answer is shift down by 12 keeping answer in Q7.3, would want final shift of 3 for 8 bit DAC
//  or is it Q7.2 with a sign bit and final shift is by 2 ?

//  SIDETONE: phase update is tone *  64 / freq_samp .   600 * 64/3000 = 12.8
//  RADIO:  freq_samp = bfo tone * 64 / desired phase update.  1500 * 64 / 10 = 9600, avoids interpolation
//  to use integer phase update.   1500*64/1 = 96000  1500*64/3 = 32000   /5 = 19200
//  try 1400 lowpass/bfo and phase update of 15.  5973 sample rate, 23893 effective sample rate, 47786 interrupt rate


void rx_process(){
static int a1,b1,c1,a2,b2,c2,d2;       // delay lines for stealing A/D samples for buttons and VOX
static uint8_t flip;                   // processing I or Q ?
static uint8_t missing;                // flag for stealing A/D reads
static uint8_t adps = 4;               // self adjusting adc clock rate, should settle at slowest we can use
                                       // start value is 7, settles on 4 for current sample rate. enable again if rate changes
// static uint8_t lock = 10;              // lockout adjusting on each busy adc, slow the change rate
                                       

// https://www.dsprelated.com/showarticle/1337.php
// CIC filter terms.  Integrate, decimate by 4, Comb filter ( length 4 but only need one delay )
static int y1,y2,y3,y4;               // 16 bit calculation, input values of 10 bit should produce 14 bit result.
static int y2d, y3d;                  // delay terms
static int8_t R;                      // decimation Rate
static int z1,z2,z3,z4;
static int z2d, z3d;

static int8_t inrx, outrx;            // pipeline indexes


   flip ^= 1;
   if( flip ){       // process Q
    
       //if( ADCSRA & ( 1 << ADSC ) ){      // still busy, use a faster adc clock
       //    if( lock == 0 ) --adps, lock = 5;
       //    --lock;
       //}
       
       c1 = ADC - 512;
       ADMUX = ad_ref | 0;                // que next read for I, merge ad_ref bits
       ADCSRA = 0xc0 + adps;              // start conversion
       if( missing == 2 ) b1 = (a1 + c1) >> 1, missing = 0;          // calc missing value of b, linear interpolation
       if( missing == 1 ) sw_adc = c1 + 512, missing = 2;            // save switch buttons value read and
                                                                     // c1 is bogus at this point, fixed next time when it is b1

       // CIC filtering
       y2 = y2 + ( y1 = y1 + a1 );        // two cascaded integrators, input is a1
       a1 = b1; b1 = c1;                  // move the A/D delay line
       R = (R+1) & 3;                     // mod 4 counter
       if( R ) return;                    // decimate by 4, process on zero, skip 1 2 3
       
                                          // 1/4 rate processing
       y3 = y2 - y2d;   y2d = y2;         // two comb filters, length is decimation rate ( 4 )
       y4 = y3 - y3d;   y3d = y3;         // but only one delay term is needed as running at 1/4 rate

       y4 >>= 4;                          // shift out extra bits.  Did we gain one real bit with the oversampling?   
      
   }
   else{             // process I
    
       //if( ADCSRA & ( 1 << ADSC ) ){      // still busy, use a faster adc clock
       //    if( lock == 0 ) --adps, lock = 5;
       //    --lock;
       //}
       
       d2 = ADC - 512;
       if( sw_adc == -1 ) ADMUX = ad_ref | 3 , missing = 1;     // steal an ADC read for the button switches
       else ADMUX = ad_ref | 1;              // que next read for Q, merge ad_ref bits ( attn setting )
       ADCSRA = 0xc0 + adps;                 // start conversion
       
       a2 = ( a2 + b2 ) >> 1;                // half sample delay

       // process a2
       z2 = z2 + ( z1 = z1 + a2 );           // two cascaded integrators
       a2 = b2;  b2 = c2;  c2 = d2;          // move A/D delay terms, extra term for 1/2 sample delay

       // R = (R+1) & 3;                     // this was done in above code
       if( R ) return;                       // decimate by 4

       z3 = z2 - z2d;   z2d = z2;         // two comb filters, length is decimation rate ( 4 )
       z4 = z3 - z3d;   z3d = z3;         // but only one delay term is needed as running at 1/4 rate
       z4 >>= 4;                          // shift out extra bits

       // have y4 and z4 , I and Q at 1/4 rate
       
       // write last PWM value and queue the calculation of the next one
       if( rx_ready_flag > 0 ){
           OCR1AL = rx_val[outrx++];
           outrx &= (RXPIPE-1);
           --rx_ready_flag;
       }
       #ifdef I2STATS                     // underrun, no audio data ready to send to PWM process
         else ++rx_underruns;             // expect some starting up, then pipeline starts working
       #endif  

       if( rx_process_flag < (RXPIPE-1) ){
          Qr[inrx] = y4;  Ir[inrx++] = z4;
          inrx &= (RXPIPE-1);
          ++rx_process_flag;
       }
       #ifdef I2STATS
         else{
            ++rx_overruns;              // this would be bad, pipeline is full, cpu is too busy, audio data lost
            //--rx_underruns;           // an overrun may cause an underrun as data was not processed
         }                              // but not every time so can't adjust the counter
       #endif
         
   }
  
}




// IIR filter, design values:  LowPass Eliptic, sr 5973, band edges 1400,2000, ripple/reject 0.5db 50db
// two sections a0,a1,a2,b1,b2     Q6 constants
const int8_t k1[] = {
    (int8_t)( ( 0.287755 + 0.0078125 ) * 64.0 ),
    (int8_t)( ( 0.614265 + 0.0078125 ) * 64.0 ),
    (int8_t)( ( -0.250054 + 0.0078125 ) * 64.0 ),
    (int8_t)( ( 1.793725 + 0.0078125 ) * 64.0 ),
    (int8_t)( ( 1.0 + 0.0078125 ) * 64.0 ),
    (int8_t)( ( 0.287755 + 0.0078125 ) * 64.0 ),
    (int8_t)( ( 0.118652 + 0.0078125 ) * 64.0 ),
    (int8_t)( ( -0,748486 + 0.0078125 ) * 64.0 ),
    (int8_t)( ( 1.114201 + 0.0078125 ) * 64.0 ),
    (int8_t)( ( 1.0 + 0.0078125 ) * 64.0 )
};


// some non-interrupt rx processing,  will this work from loop(), added a pipeline as it fell behind
// calc rx_val to be sent to PWM audio on the rx interrupt 
void rx_process2(){
int val;
static uint8_t rxout, rxin;     // local indexes rather than shared, hopefully will stay in sync with interrupt processing
static int16_t wq[6], wi[6];    // IIR filter delay terms
int I,Q;
static uint8_t i;

   noInterrupts();
   I = Ir[rxout];
   Q = Qr[rxout];
   --rx_process_flag;
   interrupts();
   ++rxout;  rxout &= (RXPIPE-1);

   I = IIR2( I, k1, wi );
   Q = IIR2( Q, k1, wq );

   // simplest weaver decoder, 90 degree sine, cosine steps, 1493 bfo freq with current sample rate
   ++i;
   i &= 3;
   val = ( i & 1 ) ? I : Q;
   if( i & 2 ) val = -val;       // get Q,I,-Q,-I...
   // ?? do we need a highpass filter here to remove DC ( sin^2 + cos^2 = 1^2 = 1 )
   // https://www.dspguide.com/ch19/2.htm

   val >>= 2;       // to 8 bits, if iir filter overloads, shift out two bits before IIR2 calls
   
   // need to be at <= 10 bits for this calc
   val *= volume;
   val >>= 6;

   val = constrain( val + 128, 0, 255 );     // clip to 8 bits
   noInterrupts();
   rx_val[rxin] = (uint8_t)val;              // queue value to be sent to audio PWM during timer2 interrupt
   ++rx_ready_flag;
   interrupts();
   ++rxin;  rxin &= (RXPIPE-1);

}


//;                storage needed per section, so double for two sections
//;  5 constants,  a0   a1  a2    b1  b2         using Q6 values
//;  3 time delay  w0   w1  w2
//  mults are 8bits * 16bits,  adds are 32 bits
int16_t IIR2( int16_t inval, int8_t k[], int16_t *w ){     // 2 section IIR
long accm;

     accm = 0;
      
   // w0 = a0*value + w1*a1 + w2*a2
     accm = k[0]*inval + k[1]*w[1] + k[2]*w[2];
     accm >>= 6;  
    // if( accm > 32767 ) accm = 32767;         // saturate values in the delay line, use constrain macro?
    // if( accm < -32768 ) accm = -32768;       // see if really needed before enabling
     *w = accm;
     
   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms ( terms backwards for the dmov TMS32xxxx )
     accm = accm + k[4]*w[2] + k[3]*w[1];
     accm >>= 6;
     *(w+2) = *(w+1);  *(w+1) = *w;

   // 2nd section
     w += 3;
     
     // w0 = a0*value + w1*a1 + w2*a2
     accm = k[5]*accm + k[6]*w[1] + k[7]*w[2];
     accm >>= 6;
     //if( accm > 32767 ) accm = 32767;         // saturate values in the delay line
     //if( accm < -32768 ) accm = -32768;     
     *w = accm;
     
   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms
     accm = accm + k[9]*w[2] + k[8]*w[1];
     accm >>= 6;
     *(w+2) = *(w+1);  *(w+1) = *w;

   //  if( accm > 32767 ) accm = 32767;         // saturate return value
   //  if( accm < -32768 ) accm = -32768;       // should have values near +-512

     return (int16_t)accm;  
}

#ifdef I2STATS
 void print_i2stats(){
           uint16_t a,b,c,d,e,f;
           noInterrupts();
           a = i2ints;  b = i2polls;  c = i2stalls;
           i2ints = i2polls = i2stalls = 0;
           d = rx_overruns;  rx_overruns = 0;      // added rx_process debugging to this, commented out i2stats
           e = rx_underruns;  rx_underruns = 0;
           f = rx_ready_flag;
           interrupts();
           LCD.clrRow( 7 );
          // LCD.printNumI( a, LEFT, ROW7 );       // should be biggest number, interrupts working
          // LCD.printNumI( b, CENTER, ROW7 );     // some OLED functions may overfill our buffer, polling counts here
          // LCD.printNumI( c, RIGHT, ROW7 );      // polling may cause int flags out of sync with i2state, counts here
          LCD.printNumI( d, LEFT, ROW7 );          // rx_process overruns
          //LCD.printNumI(ADCSRA & 7 , CENTER, ROW7 );    // check auto ADC baud rate value
          LCD.printNumI( f, CENTER, ROW7 );        // pipeline delay
          LCD.printNumI( e, RIGHT, ROW7 );         // rx underruns
 }
#endif


// txtest timing loop
// !!! commment baud change in i2start, then fix when done with this test
// !!! comment timer setups in setup
// !!! check send fast optimization
//  best sample rates with small work load, best we can hope for.  I2C bus restricted
//  baud 4  5785 with waits
//  baud 1  6447 with waits  .  TWBR as 1 is 1.1111 meg.   TWBR as 2 is 1.0 meg. Not sure Si5351 is receiving these bauds correctly.
//  full work load, no waits at 9 4207,  best 2 4224 , not much improvement with faster baud after workload restricted
//  comment out hilbert, no waits at 6 4750
//  comment out calc fast, 1  6013 with waits,  4 5520, 2 5698, can calc fast be improved?
//  TWBR = 4 is 833k might be a good setting, might get a tx sample rate of 5400?
//  full load, one less xfer on I2C, no waits 12 4468 ,  best 2 4494, 4 is 4488
//     Less transfers would mean less I2C interrupt overhead, we gained 200 hz
//  small workload, one less I2C xfer, 3 6092, 9 5394, 7 5863 .  I2C bus restricted.  4 and 2 have bogus data?
//  new calc fast, all have waits 2 6494, I2C bus restricted ( have one less I2C xfer )
//  comment out send bulk optimization, 2 5603, using i2flush in loop 3 5544, added twbr as 1 5859, 2 5604
//  add some delay to loop to find where cpu load exceeds i2c load
//    100us 12 3811,  40us 4 4973, 20us 2 5505 waits, 22us 2 5475 no waits
//  enable send bulk opt again, less i2c load, 22us 3 5926, 23us 4 5890
//  random df offsets to calc fast, 20us slow, 0us 14 4115.  Is random function very very slow
//  back to nowaits norandom, 3 6100, 2 6300 waits.  
//  disable send bulk again, worse case I2c load,  test to 3, 0us 4 5300 waits, 10us 3 5321 waits 4 5278
//  enable send bulk again, less I2c load, 10 us 4 5987 3 6096 7 5665 all with waits
//  still think a sample rate of 5300 to 5400 with twbr as 4 or 3 will work.
//    Less I2c load 4 is 5987, More I2c load and 4 is 5300. 


//  Once we have no waits a faster baud will not help much.  1.111 meg may be useful if Si5351 works at that baud.
//  road to improvement, Speed up calc fast if possible.   Less I2C transfers -> small improvement.
//  Maybe the Hilbert is too long, gained 500 hz in test without.

void looptxtest(){
static int twbr = 14;
static int row = 2;
unsigned int notdone, loops;
unsigned long tm;
static uint16_t btw,bloops,bndone;

   notdone = loops = 0;
   //set_timer1( 0 );          // !!!! is this off ?, commented out in setup, actually need to stop timer2

   tm = millis();
   TWBR = twbr;
   
   si5351.freq_calc_fast(1000);   // ?? why is this here
   i2flush();
   /*
   while( i2done == 0 ){
     noInterrupts();
     i2poll();
     interrupts();
   } */
   
   while( millis() - tm < 10000 ){

       ++loops;

       tx_hilbert(64);  
       arctan3(64,23);
       fastAM(64,23);
       si5351.freq_calc_fast(2700);
       //delayMicroseconds(10); 
       if( i2done == 0 ) ++notdone;
       // i2flush();
       while( i2done == 0 ){
          noInterrupts();
          i2poll();
          interrupts();
       } 
       si5351.SendPLLBRegisterBulk();
   }

   i2flush();
   TWBR = 17;
   LCD.clrRow( row );
   LCD.printNumI( twbr, LEFT, 8*row );
   LCD.printNumI( loops, CENTER, 8*row );
   LCD.printNumI( notdone, RIGHT, 8*row );

   if( loops > bloops ){            // best on top
      bndone = notdone;
      bloops = loops;
      btw = twbr;
      LCD.clrRow( 0 );
      LCD.clrRow( 1 );
      LCD.setFont( MediumNumbers );
      LCD.printNumI( btw, LEFT, 0 );
      LCD.printNumI( bloops/10, CENTER, 0 );
      LCD.printNumI( bndone/1000, RIGHT, 0 );
      LCD.setFont( SmallFont );
   }
   i2flush();
   
   if( twbr > 10) --twbr;    // by two when over 10
   if( twbr > 20) --twbr;
   if( twbr > 3 ) --twbr;
   else{
      twbr = 14;
      bndone = 0;  bloops = 0; btw = 29;
   }
   if( ++row > 6 ) row = 2;
   
}

/***************  temp code that may be useful again for debugging  ***************/
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

  //if( sw_adc == -1 ) fake_it();      // !!! debug  Analog read of switches requested. was in loop.
    /*****
void fake_it(){        // !!! test code.   analog read of the switches, fake it for now
  
    sw_adc = analogRead( SW_ADC );
    //LCD.printNumI( sw_adc, RIGHT, ROW6 );   // readings are 772, 932, 1023 for the 3 buttons
}
******/
