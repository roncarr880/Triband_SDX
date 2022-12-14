
/*
 * uSDX Triband Radio, OLED version.
 * 
 * Main features of this software version: 
 *     Interrupt driven I2C routines. We can calculate the next Si5351 transmit information while the I2C bus is busy.
 *        In theory we may achieve a higher tx sampling rate than otherwise.
 *     Distributed receive processing with pipelined data flow.    
 *        In theory we will have more time to process more complicated receive algorithms.
 *     Able to use a library for the screen with some nice numeric fonts.
 * 
 *     Hardware mods.  The Si5351 module was NOT converted to run 3.3 volts I2C signals. 
 *     Jumpered pin 27 to 4 and 28 to 5 to get I2C signals to the OLED.
 *     On the display board I used jumpers in place of the diodes ( D1,D2 ) to run the OLED on 5 volts.
 *     
 *     I am running 5 volt I2C signals.   Standard is to run 3.3 volt I2C. I now believe it will be ok to use the jumpers
 *     if everything on your board has I2C signals at 3.3 volts.  ( an inconsistancy exists between the OLED on 3.3 volts and
 *     the standard program in that the comments in the program say they are bit banging 5 volt signals to the OLED )  So 
 *     a danger still exists in burning up your Si5351 if you add the jumpers but run the standard program instead of this
 *     one ).  To try this program the process should be, load this program -> your OLED will not work.  Then add the 
 *     jumper wires.  To revert back you should remove the wires, then load the standard program.
 *     
 * RCL filter for audio out.   470 ohm, 100uh, 100n to ground.  Put the inductor in series with the existing 470 ohm, and    
 *     add the capacitor to ground on input or output side of the 10uf series capacitor.  
 *     The two pole filter should cut the 72khz PWM by 30 db.
 *     I also put a 22 ohm resistor across the audio out jack.  A higher volume setting when using
 *     a digital volume control means more bits are being used which should result in better fidelity.
 *     
 * More modulation bits.  Put a 10k resistor from TP1 to ground on the RF board.   
 *     
 * I put in 2.2nf caps in the op-amp feedback.  I also have 47k resistors in place of 82k as I hadn't any 82k resistors on hand.   
 * 
 * My process:  compile, AVRDudess, file in user/ron/local/appdata/temp/arduino_build_xxxxxx/, pickit 2 as programmer
 *  ? power up with pickit app to test ?  
 *  does pickit app apply 12v vpp when searching for a device ID?  ( it applies 5v a few times, 9 volts, then 12 volts ).
 *          !!!!!!!!!!  Yes, so don't power the radio with the Pickit2 App. !!!!!!!!!!!
 * 
 * Button Functions,  buttons respond to TAP, Double TAP, and Long Press
 *   Select: Tap opens menu, another Tap enables edit , DTap opens menu in edit mode, Long Press qsy's -100kc
 *   Exit  : Tap backs out of menu one step, DTap backs out all the way, Long Press qsy's +100kc
 *           Also used to enable TX after a band change and to exit from RIT.
 *           Also turns on the S meter and enables the CW decoder.
 *   Encoder: Long Press opens Volume directly for edit.
 *            Tap changes tuning step 
 *            Dtap  toggles Tune Power / Full Power 
 *            
 *   RIT function is enabled on transmit.  Tap exit to cancel.        
 *   AM is tuned 1khz high, carrier is removed with a notch filter.
 *   3 power levels, Band Change - check band switches message == no power,  Tune Power, Full Power.
 *   
 *   avr-objdump -S Triband_sdx.ino.elf to see the assembly code
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
#include "my_morse.h"

#define ENC_A  6
#define ENC_B  7
#define SW_ADC A3
#define AUDIO_PIN 9             // PWM pins
#define TXMOD_PIN 10
#define RX_EN  8                // pb0
#define PTT   13
#define DIT_PIN   13            // using input_pullup on pin 13
#define DAH_PIN   12            // 12 is tx audio with cap loading and external pullup, think this is dah
#define DIT  1
#define DAH  2
#define RELAY  11               // solid state relay for keying external amp PB3

// timer settings
#define AUDIO  0b10000000       // TCCR1A values, timer1
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
#define AM  3 

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
  uint16_t tx_overruns;
  uint16_t tx_underruns;
  uint16_t tx_i2late; 
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
uint8_t mode = LSB;
uint8_t kspeed = 12;
uint8_t kmode = 1;         // 0 straight, 1 mode A, 2 mode B, 3 Ultimatic, swapped dit and dah same lineup 4,5,6,7
uint8_t side_vol = 15;
uint8_t bdelay = 30;
uint8_t cal = 128;         // final frequency calibration +-500hz
uint8_t Pmin = 0;
uint8_t Pmax = 15;         // max is 63;
uint8_t vox;
uint8_t agc_on = 1;        // normally on, turn off to test image rejection

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

#define TXPIPE  32
volatile int TVal[TXPIPE], Phase[TXPIPE];
volatile uint8_t  Mag[TXPIPE];
volatile uint8_t tx_process_flag;
volatile uint8_t tx_ready_flag;

#define FREQ_U 0               // encoder users, tuning and menu
#define MENU_U 1
uint8_t encoder_user;
int     step_ = 1000;
uint8_t tx_inhibit = 1;        // start with a band switches check message before transmit, sets power out at minimum possible
uint8_t tune_pwr;              // low power mode, double tap encoder to toggle
uint8_t s_tone;                // sidetone independant of actual transmit condition
volatile int     break_in;     // semi break in counter
volatile int8_t  waveshape;    // cw wave shaping from sine cosine table
uint8_t menu_on = 1;           // screen sharing flag
int16_t maxAM;                 // AM mode carrier level

// CIC filter compensation     FIR filter of length 3 with constants  -alpha/2,  alpha+1, -alpha/2
uint8_t a_alpha = 5;
int16_t a_plus1  = 96;
int16_t a_by2    = 16;          // negative 16 when used


#include "si5351_usdx.cpp"     // the si5351 code from the uSDX project, modified slightly for calibrate and RIT
SI5351 si5351;

const char msg1[] PROGMEM = "Check Band Switches";
const char msg2[] PROGMEM = "RIT Enabled";
const char msg3[] PROGMEM = "SDX TriBander";
const char msg4[] PROGMEM = "K1URC wb2cba pe1nnz";
const char msg5[] PROGMEM = "EEPROM ? Check vars";
const char msg6[] PROGMEM = "Tune Power Level";
const char msg7[] PROGMEM = "Full Power Enabled";

int16_t  agc;                          // made variable global for the s meter
uint8_t bits = 6;                      // agc bits == number of bits shifted out of the CIC filter process
int16_t cw_signal;                     // for cw decoder
uint8_t cw_count;


// Direct form 1 IIR filters, Q6 constants in b0,b1,b2,a1,a2 order, 2 sections
const int8_t filters[7][10] PROGMEM = {
   {  64, 0, 0, 0, 0, 64, 0, 0, 0, 0 },                  // passthrough SSB
   { 37,75,37,67,19, 47,94,47,84,40 },                   // 2400 butterworth
   { 27,54,27,36,7, 36,73,36,49,32 },                    // 2000
   { -19, 0, 19, -101, 47,  -14, 0, 14, -61, 36 },       // CW wide
   { -14,0,14,-87,43, -12,0,12,-64,39 },                 // bessel 300
   { -7,0,7,-89,52, -7,0,7,-78,50 },                     // bessel 150
  // { 46,-92,46,-89,31, 54,-108,54,-104,48 }              // AM highpass makes noise by itself
   { 59,-60,59,-67,55, 58,-57,58,-51,55 }                // AM notch at 1k
};
int8_t  kf[10] = {  64, 0, 0, 0, 0, 64, 0, 0, 0, 0 };    // default as passthrough
int16_t wi[6], wo[6];                                    // delay terms for the IIR filter
uint8_t filter;                                          // chosen filter,  menu item
volatile uint8_t from_int;                               // i2 functions called from interrupt context and non-interrupt


void setup() {

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(SW_ADC, INPUT);
  pinMode(TXMOD_PIN, OUTPUT );
  digitalWrite(TXMOD_PIN, LOW );
  pinMode( AUDIO_PIN, OUTPUT );
  digitalWrite( AUDIO_PIN, LOW );          // if thumps try the floating the pin as in timer1 startup notes
  pinMode( RX_EN, OUTPUT );
  pinMode( PTT, INPUT_PULLUP );            // pin 13, this is also DIT_PIN pin I think
  pinMode( 12, INPUT );                    // has a 10k pullup, DAH_PIN I think. 
  pinMode( RELAY, OUTPUT );                // can swap the DIT_PIN, DAH_PIN defines if desired as hardcoded the actual pin 12 here
  digitalWrite(RELAY,LOW);                 // the external amp relay

  i2init();
  si5351.powerDown();
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

  calc_maxAM();                            // AM mode carrier
  
  analogRead( SW_ADC );                    // enable the ADC the easy way?
  // PRR &= ~( 1 << PRADC );               // adc power reduction bit should be clear

  set_timer1( AUDIO );                     // enable pwm audio out
  set_timer2( RX );                        // start receiver
  digitalWrite( RX_EN, HIGH );             // antenna switch

  si5351.SendRegister(24, 0b00010000);     // this was the proposed fix for the CW waveform glitch
                                           // leaves clock2 output high when disabled

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
static uint8_t robin;          // round robin some processing
static uint8_t max_bits;
int t;

 // high priority.
  while( rx_process_flag ) rx_process2();
  while( tx_process_flag ) tx_process2();
 
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

  if( s_tone ) sidetone();

  // round robin 1ms processing. Process each on a different loop
  if( robin == 1 ) button_state(), ++robin;
  if( robin == 2 ) sel_button(), ++robin;
  if( robin == 3 ) exit_button(), ++robin;
  if( robin == 4 ) enc_button(), robin = 0;


  if( tm != millis()){          // 1 ms routines ( a quick 1ms - 20meg clock vs 16meg )
     tm = millis();
     robin = 1;
     if( mode == CW ){
        keyer();
        if( cw_count ) code_read( cw_signal ), cw_count = 0;
     }
     else ptt();

     noInterrupts();
     if( break_in ){             // semi break in counter
        if( --break_in == 0 ) rx();
     }
     interrupts();

     if( eeprom_write_pending ){
        if( --eeprom_write_pending == 0 ) strip_save();
     }

     if( bits > max_bits ) max_bits = bits;             // peak reading s meter
     if( (sec & 255) == 255 ){                          // call 4 times a second to update meter
         smeter( max_bits, agc );
         max_bits = 0;
     }
     
     if( ++sec == 15000 ){                              // printing debug counters each 15 seconds
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
    if( sw_adc == -1 ) return last;         // no adc results yet, guess it is same as last time
    val = sw_adc;                           // save value, queue another read
    sw_adc = -1;

    last = 8;                               // get the vref up to 5 volts, 8 == invalid switch
                                            // 860 983 are usdx values for breakpoints in voltage read                                        
    if( val > 680 ) last = 1;               // select sw
    if( val > 830 ) last = 2;               // was 852, set lower as any sw bounce may false detect select instead of exit
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

    switch( sw_state[2] ){
        case DTAP:  freq = 3928000; qsy(0); si5351.SendRegister(177, 0xA0);    // !!! temp qsy to 80 meters
           tune_pwr ^= 1;   pwr_message();
        break;
        case TAP:  step_ = ( step_ == 100 ) ? 1000 : 100;  break;    // toggle tuning step size
        case LONGPRESS:                                              // quick entry to edit volume
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

void pwr_message(){

  if( tune_pwr ) p_msg( msg6,6 );
  else p_msg( msg7,6 );
}

void qsy( int8_t f ){
int st;

   if( transmitting ) return;
   st = step_;                            // different step depending upon mode
   if( mode == CW ) st = st >> 1;
   if( rit_enabled ) st = 10;
   
   freq = (int32_t)freq + f * st;
   calc_divider();
   if( mode == USB ) si5351.freq( freq, 0, 90, divider );
   else si5351.freq( freq, 90, 0, divider );
   display_freq();
      // offsets for cw and AM, 600hz, 1k hz
   if( rit_enabled == 0 ){   
      if( mode == CW ){
        si5351.freq_calc_fast( -600 );
        si5351.SendPLLBRegisterBulk();   
      }
      if( mode == AM ){                    // AM is tuned 1k high off freq
        si5351.freq_calc_fast( -1000 );
        si5351.SendPLLBRegisterBulk();   
      }
   }
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
uint32_t f;
static uint8_t msg_displayed;                      // write the RIT message only once

   if( transmitting ) return;
   if( rit_enabled == 0 ) msg_displayed = 0;
   f = freq;
   if( mode == CW ) f -= 600;
   if( mode == AM ) f -= 1000;
   
   rem = f % 1000;

    // display big numbers in the blue area of the screen
    // font widths/height are: small 6 x 8, Medium 12 x 16, Big 14 x 24
    LCD.setFont(BigNumbers);
    LCD.printNumI(f/1000,12,ROW2,5,'/');
    LCD.setFont(MediumNumbers);
    LCD.printNumI(rem,5*14 + 12 + 3,ROW3,3,'0');
    LCD.setFont( SmallFont );                                                        // keep OLED in small text as the default font
    if( rit_enabled && msg_displayed == 0 ) p_msg( msg2, 6 ), msg_displayed = 1;     // RIT message
    LCD.gotoRowCol( 3,0 );
    LCD.putch( band_priv(f));
 
}

//  USA: can we transmit here and what mode
//  lower case for CW portions, upper case for phone segments
//  X - out of band, E - Extra, A - Advanced class, G - General 
char band_priv( uint32_t f ){
char r = 'X';

   f = f / 1000;
   //else f = f / 100;                         // 60 meters, test 500hz steps
          if( f >= 3500 && f <= 4000 ){
             if( f < 3525 ) r = 'e';
             else if( f < 3600 ) r = 'g';
             else if( f < 3700 ) r = 'E';
             else if( f < 3800 ) r = 'A';
             else r = 'G';
          }

       /*    60 meters by 500 hz steps
       case 1:
          if( mode == USB || mode == DIGI ){
             if( f == 53305 || f == 53465 || f == 53570 || f == 53715 || f == 54035 ) r = 'G';
          }
          if( mode == CW ){
             if( f == 53320 || f == 53480 || f == 53585 || f == 53730 || f == 54050 ) r = 'g'; 
          }
       break;
       */
          if( f >= 7000 && f <= 7300 ){
             if( f < 7025 ) r = 'e';
             else if ( f < 7125 ) r = 'g';
             else if ( f < 7175 ) r = 'A';
             else r = 'G';
          }

          if( f >= 10100 && f <= 10150 ) r = 'g';

          if( f >= 14000 && f <= 14350 ){
             if( f < 14025 ) r = 'e';
             else if( f < 14150 ) r = 'g';
             else if( f < 14175 ) r = 'E';
             else if( f < 14225 ) r = 'A';
             else r = 'G';
          }

          if( f >= 18068 && f <= 18168 ){
             if( f < 18110 ) r = 'g';
             else r = 'G';
          }

          if( f >= 21000 && f <= 21450 ){
             if( f < 21025 ) r = 'e';
             else if( f < 21200 ) r = 'g';
             else if( f < 21225 ) r = 'E';
             else if( f < 21275 ) r = 'A';
             else r = 'G';
          }

         if( f >= 24890 && f <= 24990 ){
            if( f < 24930 ) r = 'g';
            else r = 'G';
         }

         if( f >= 28000 && f <= 29700 ){
            if( f < 28300 ) r = 'g';
            else r = 'G';
         }

  return r;
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
  // And we find using 125k does free up processing time for the receiver, less interrupt overhead probably
  //TWBR = ( adr == 0x60 ) ? 4 : 72;  // oled lower priority, we shouldn't write oled while transmitting
  // TWBR = ( transmitting ) ? 4 : 72;  // oled lower priority, we shouldn't be writing the oled while transmitting
  if( transmitting && TWBR == 72 ){
      delayMicroseconds(12);            // wait one baud for stop to clear on the bus 
      TWBR = 4;
  }
  if( transmitting == 0 && TWBR != 72 ){
      delayMicroseconds( 12 );
      TWBR = 72;
  }
  
}

void my_delay( uint32_t val ){                  // delay micros with yield type of feature
uint32_t tm;

    if( transmitting ) return;                  // rx process not running, can't wait for tx, just drop the packet
    //if( transmitting == 0 ) val *= 4;             // for 125k baud delay
    tm = micros();
    while( micros() - tm < val ){
        if( rx_process_flag ) rx_process2();    // run receive processing while waiting on the I2C bus
        //if( tx_process_flag ) tx_process2();
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
   if( from_int == 0 ) interrupts(); // called from interrupt processing sendPLLRegisterBulk, interrupts enabled early in that case.
                                     // unless add this volatile variable to flag where the call to i2stop came from 
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
           //   while(TWCR & (1<<TWSTO));                                      // wait for previous stop to clear
              TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (I2INT_ENABLED);  // set start condition
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) | (I2INT_ENABLED);
              state = 3;
              //i2done = 1;
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

 
   if( state == 3 ){     // was once,  merged this processing into state 0, may not need to wait for stop
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

/*********** end I2C functions  ************/


// simple menu with parallel arrays. Any values that do not fit in 0 to 255 will need to be scaled when used.
// example map( var,0,255,-128,127)
#define NUM_MENU 15
                         // pad with spaces out to multiple of 4 fields
const char smenu[] PROGMEM = "Vol AttnBandModeKSpdKmdeSvolBdlyCal PminPmaxVox ToneAGC Filt    ";
uint8_t *svar[NUM_MENU] = {&volume,&attn,&band,&mode,&kspeed,&kmode,&side_vol,&bdelay,&cal,&Pmin,&Pmax,&vox,&a_alpha,&agc_on,&filter};
uint8_t  smax[NUM_MENU] = {  63,   3,    2,     3,     25,    7,     63,       100,   254,   10,   63,  1,   20,      1,        2 };
uint32_t stripee = 0b00000000000000000000011110100000;           // set corresponding bit for strip menu items to save in eeprom

// note: a smax value of 255 will not be restored from eeprom, it looks like erased data.
//       smax of 1 allows two values for the variable.  Max for band (2) is 3 bands 0,1,2
//       the zero value for cal is 128 allowing both pos and neg frequency calibration


// commands: 2,4 encoder, 0 reset entry, 1 select, 3 exit
void strip_menu( int8_t command ){        
static uint8_t sel;
static uint8_t ed;
static uint8_t mode;   // 0 not active, 1 select var, 2 edit var
static uint8_t hyper;

   if( transmitting ) return;
   if( command == 3 && mode == 0 ){         // exit when not needed, clear out some latched on functions
      LCD.clrRow(6);
      tx_inhibit = 0;                                 // enable transmit
      if( rit_enabled ){                              // cancel RIT
         rit_enabled = 0;                             // returns to rx freq as we don't save the tx freq, just in si5351 registers
         freq = freq - freq % step_;                  // clear out LSDigits
         qsy(0);
      }
      menu_on = 0;                                    // enable S meter display
      return;
   }

   
   // slow down encoder inputs
   hyper ^= 1;
   if( hyper && (command == 2 || command == 4) ) return;   // half speed encoder

   if( command == 0 ) hyper = mode = sel = ed = 0;    // menu reset
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
       menu_on = 1;                        // sharing screen space with the s meter
   }
   else if( menu_on ){
       strip_display( sel>>2, 0xff, 0 );   // display menu with no inverse text
       encoder_user = FREQ_U;              // encoder now changes frequency
   }

   if( (command == 2 || command == 4) && ed ) strip_post( sel );      // impliment changes 
  
}

// print 4 strings and values from somewhere in the strip menu text, offset will be a page number or group of 4
const char mode_str[] PROGMEM = "CW USBLSBAM ";
const char keyer_str[] PROGMEM = " S  A  B Ult S swAswBswU";            // modes and paddle switched modes
void strip_display( uint8_t offset, uint8_t sel, uint8_t ed ){
uint8_t i,k;
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
          for( k = 0; k < 3; ++k ) LCD.putch( pgm_read_byte( &mode_str[3*mode+k] ));
       }
       else if( offset == 1 && i == 1 ){         // text for keyer modes and swapped paddles modes
          LCD.gotoRowCol( 1, i*6*5 );
          for( k = 0; k < 3; ++k ) LCD.putch( pgm_read_byte( &keyer_str[3*kmode+k] ));
       }
       else LCD.printNumI( val, i*6*5, ROW1,3,' ' );
       LCD.invertText(0);
   }
  
}


void strip_post( uint8_t sel ){    // do any post processing needed
int i;

   // LCD.clrRow( 6 );             // help message area
   // LCD.gotoRowCol( 6,0 );       // having this here with no writes to follow causes a program hang
                                   // OLED left in command mode?  goto is one of my "enhancements" to the library.
   
   switch( sel ){
      case 1:  digitalWrite( RX_EN, ( attn & 2 ) ? LOW : HIGH );   break;
      case 2:                      // band change
        bandstack[oldband] = freq;
        freq = bandstack[band];
        oldband = band;
        p_msg( msg1, 6 );          // Check band switches message
        tx_inhibit = 1;
      //break;
      case 3:                      // mode change.  clear RIT or Si5351 will not be reset to new dividers
         rit_enabled = 0;
         set_timer2( RX );         // different sample rate for CW mode
         qsy(0);
         load_filter();
         break;
      case 8:  qsy(0);  break;                     // calibrate, implement freq change for user observation
      case 9: case 10:  calc_maxAM();   break;
      case 12:  calc_CIC();  break;                // Tone control is the CIC compensation filter parameters
      case 13:  bits = 8;  break;                  // agc off or on, set agc compression to max
      case 14: load_filter(); break;
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
                LCD.printNumI( i, RIGHT, ROW6 ); 
                delay(8000);          // message will be overwritten 
                return;               // abort restore
            }
            *svar[i] = val;
         }
      }
   }
}


void calc_maxAM(){                      // calc max AM mode carrier level
  
   maxAM = 255*Pmax;
   maxAM >>= 6;
   maxAM += Pmin;
   maxAM >>= 1;                         // peak carrier level 1/2 of peak signal
}


void calc_CIC(){                      // calculate the constants for the CIC compensation filter.  In Q6 format.
float a;

   a = (float)a_alpha / 10.0;
   a_plus1 = 64.1 * ( 1.0 + a );
   a = a / 2.0;
   a_by2 = 64.1 * a;
}

void load_filter(){                 // copy filter constants from flash to ram
uint8_t f,c;
uint8_t i;

   f = filter;                      // set up for SSB filters
   if( mode == AM ) f =  6;         // notch 1k
   if( mode == CW ) f += 3;

   noInterrupts();
   for( i = 0; i < 10; ++i ) kf[i] = pgm_read_byte( &filters[f][i] );
   interrupts();
  
}

/*******************  end menu *********************/

/*****    Radio processing   *****/

int8_t read_paddles(){                    // keyer and/or PTT function
int8_t pdl;

   pdl = 0;
   if( digitalRead( DAH_PIN ) == LOW ) pdl = 2;
   if( digitalRead( DIT_PIN ) == LOW ) pdl += 1;
   
   if( kmode & 4 ){                        // swap paddles 
      pdl <<= 1;
      if( pdl & 4 ) pdl += 1;
      pdl &= 3;                            // wire straight key to ring of plug
   }

   return pdl;
}

void side_tone_on(){    // can have practice mode, delayed breakin, key waveform shaping

   // use tx_inhibit for practice mode.  Change bands, don't clear the message.
   
   s_tone = 1;
   if( transmitting == 0 ) tx();
   noInterrupts();
   waveshape = 1;                           // waveshape counter to ramp up
   interrupts();
   si5351.SendRegister(3, 0b11111011);      // key the drive, no backwave
}

void side_tone_off(){

   s_tone = 0;
   noInterrupts();
   waveshape = -16;
   interrupts();
}


void ptt(){                        // ssb PTT or straight key via keyer() function
static uint16_t dbounce;
static int8_t txing;                  // local dupe of variable transmitting.  uses: delayed breakin, wave shaping, practice mode
int8_t pdl;

   pdl = digitalRead( PTT ) ^ 1; 
   dbounce >>= 1;                  // shift bits right, any bit as 1 counts as on, no bits is off, stretches on time slightly
   if( pdl ) dbounce |= 0x400;     // 1ms delay per bit debounce, 0x400 is 15ms, 0x80 is 8ms

   if( mode == CW ){               // straight key mode
      if( txing && dbounce == 0 ) txing = 0, side_tone_off();
      else if( txing == 0 && dbounce ) txing = 1, side_tone_on();
   }
   else{                           // SSB
      if( txing && dbounce == 0 ) txing = 0 , rx();
      else if( txing == 0 && dbounce ) txing = 1, tx(); 
   }
}


// http://cq-cq.eu/DJ5IL_rt007.pdf      all about the history of keyers

#define WEIGHT 200        // extra weight for keyed element

void keyer( ){            // this function is called once every millisecond
static int8_t state;
static int count;
static int8_t cel;           // current element
static int8_t nel;           // next element - memory
static int8_t arm;           // edge triggered memory mask
static int8_t iam;           // level triggered iambic mask
int8_t pdl;


   if( (kmode & 3) == 0 ){    // straight key mode
      ptt();
      return;
   }
   pdl = read_paddles();
   if( count ) --count;

   switch( state ){
     case 0:                               // idle
        cel = ( nel ) ? nel : pdl;         // get memory or read the paddles
        nel = 0;                           // clear memory
        if( cel == DIT + DAH ) cel = DIT;
        if( cel == 0 ) break;
        iam = (DIT+DAH) ^ cel;
        arm = ( iam ^ pdl ) & iam;         // memory only armed if alternate paddle is not pressed at this time, edge trigger
                                                    // have set up for mode A
        if( (kmode & 3) == 2 ) arm = iam;           // mode B - the descent into madness begins.  Level triggered memory.
        if( (kmode & 3) == 3 ) iam = cel;           // ultimatic mode
        
        count = (1500+WEIGHT)/kspeed;               // normally 1200 but running at 20mhz with short timing
        if( cel == DAH ) count *= 3;
        state = 1;
        side_tone_on();
     break; 
     case 1:                                  // timing the current element. look for edge of the other paddle
        if( count ) nel = ( nel ) ? nel : pdl & arm;
        else{
           count = 1500/kspeed;
           state = 2;
           side_tone_off();
        }
     break;   
     case 2:                                  // timing the inter-element space
        if( count ) nel = ( nel ) ? nel : pdl & arm;
        else{
           nel = ( nel ) ? nel : pdl & iam;   // sample alternate at end of element and element space
           state = 0;
        }
     break;   
   }
  
}



void tx(){                // change to transmit

    set_timer2( 0 );
    
    digitalWrite( RX_EN, LOW );                 // mute
    digitalWrite(RELAY,HIGH );                  // solid state relay for external amp
    if( mode == CW ) set_timer1( TXMOD+AUDIO );
    else set_timer1( TXMOD );
    // set_timer2( TX );
    transmitting = 1;
    si5351.SendRegister(3, 0b11111011 );
    rit_enabled = 1;                            // auto RIT feature, cancel with exit button

    set_timer2( TX );
}

void rx(){                // change to receive

    set_timer2( 0 );
    set_timer1( AUDIO );
    // set_timer2( RX ); 
    si5351.SendRegister(3, 0b11111100);
    transmitting = 0;
    digitalWrite(RELAY,LOW);
    digitalWrite( RX_EN, ( attn & 2 ) ? LOW : HIGH );
    set_timer2( RX );
}

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
   OCR1AL = 128;  OCR1BL = 0;  TCNT1L = 0;
   TCCR1A = clk | 1;
   TCCR1B = 0b00001001;
  // pinMode( AUDIO_PIN, OUTPUT );
   }
   interrupts();
}

void set_timer2( int8_t timer_mode){

  noInterrupts();
  TCCR2B = 0;                        // stop
  TCNT2 = 0;                         // clear counter
  PRR &= ~(1 << PRTIM2 );            // powered up
  TCCR2A = 2;                        // CTC mode
  TIMSK2 = timer_mode;               // enable interrupts A match or B match, RX TX defines
                                     // OCR2A = (F_CPU / pre-scaler / fs) - 1;
                                     // actual fs = f_fcu / ( N * (1 + OCR2A ))
  
  if( timer_mode == RX ){
     if( mode == CW ) OCR2A = 62;          // 40000, actual is 39682, slower for cw as out of cpu
     else OCR2A = 51;                      // count to value, resets, interrupts, fs = 48077:51  54347:45
  }
  //if( timer_mode == TX ) OCR2A = 222;      // fs = 11210 int rate,  5605 tx update rate, if change here then change _UA
  if( timer_mode == TX ) OCR2A = 230;      // fs = 10822            5411 tx update rate

  if( timer_mode != 0 ){
    OCR2B = OCR2A - 1;                       // use the 2nd interrupt for transmit
    TCCR2B = 2;                              // start clocks, divide by 8 clock prescale
  }
  interrupts();
}


ISR(TIMER2_COMPA_vect){               // Timer2 COMPA interrupt

   rx_process();  
}

ISR(TIMER2_COMPB_vect){               // Timer2 COMPB interrupt

   if( mode == CW ) cw_process();       // goto cw functions, wave shaping
   else tx_process();                  
}

/*******   tx functions   *******/

void cw_process(){                    // cw wave shaping from transmit interrupt
static int8_t c;
int power;
int inv;

   if( ++c < 3 ) return;                     // shape, I think mod 3 will be about 5ms
   c = 0;
   if( waveshape > 0 && waveshape < 17 ){    // ramp up cw power
      power = pgm_read_byte( &sin_cos[waveshape] ) << 2;       // 0 - 256
      power *= Pmax;
      power >>= 6;
      if( tune_pwr ) power >>= 1;
      if( tx_inhibit == 0 ) OCR1BL = constrain(power + Pmin,0,255);
      ++waveshape;
      break_in = 0;
   }
   if( waveshape < 0 ){                      // ramp down
      inv = -waveshape;
      power = pgm_read_byte( &sin_cos[inv] ) << 2;
      power *= Pmax;
      power >>= 6;
      if( tune_pwr ) power >>= 1;
      if( ++waveshape == 0 ){
        OCR1BL = 0;
        si5351.SendRegister(3, 0b11111111);    // tx clock off
        break_in = 10 * (int)bdelay + 1;      
      }
      else if( tx_inhibit == 0 ) OCR1BL = constrain(power + Pmin,0,255);
   }
  
}


int16_t valq, vali;                                 // results of hilbert filter are global

/*
// a 31 tap classic hilbert every other constant is zero, kaiser window
void tx_hilbert( int16_t val ){
static int16_t wi[31];                              // delay terms
int16_t t;

const int16_t k0 = (int16_t)( 1024.5 * 0.002972769320862211 );     // scale up two bit   195
const int16_t k1 = (int16_t)( 256.5 * 0.008171666650726522 );      // max worst case     522
const int16_t k2 = (int16_t)( 256.5 * 0.017465643081957562 );      //                   1142 
const int16_t k3 = (int16_t)( 256.5 * 0.032878923709314147 );      //                   2154 
const int16_t k4 = (int16_t)( 256.5 * 0.058021930268698417 );      //                   3786
const int16_t k5 = (int16_t)( 256.5 * 0.101629404192315698 );      //                   6528
const int16_t k6 = (int16_t)( 256.5 * 0.195583262432201366 );      //                  13056   27383 tot
const int16_t k7 = (int16_t)( 256.5 * 0.629544595185021816 );      //

    val = constrain(val,-128,127);                     // avoid overflow, tx clipper
                                                       // !!! but when used for RX it clips I but not Q
                                                       // and scaling by clipping not valid as I and Q are out of phase
   // think it will handle 7 bits in with the adjustment on k7 multiply
   for( int i = 0; i < 30; ++i )  wi[i] = wi[i+1];
   wi[30] = val;

   valq = wi[15];
   vali =  k0 * ( wi[0] - wi[30] ) >> 2;
   vali += k1 * ( wi[2] - wi[28] ) + k2 * ( wi[4] - wi[26] ) + k3 * ( wi[6] - wi[24] ) +
           k4 * ( wi[8] - wi[22] ) + k5 * ( wi[10] - wi[20]) + k6 * ( wi[12] - wi[18]);
   vali >>= 8;
   t = k7 * ((wi[14] - wi[16]));
   vali += (t >> 8);
}
*/


// 19 tap classic hilbert, no window, also used for RX
void tx_hilbert( int16_t val ){
const int16_t k0 = (int16_t)( 256.0 * 0.064724396357330738 );
const int16_t k1 = (int16_t)( 256.0 * 0.083217089189936755 );
const int16_t k2 = (int16_t)( 256.0 * 0.116503933432949208 );
const int16_t k3 = (int16_t)( 256.0 * 0.194173231907191046 );
const int16_t k4 = (int16_t)( 256.0 * 0.582519710000210189 );
static int16_t w[19];
uint8_t i;
int16_t t;

    for( i = 0; i < 18; ++i )  w[i] = w[i+1];
    w[18] = val;
    valq = w[9];

    vali = k0 * ( w[0] - w[18] ) + k1 * ( w[2] - w[16] ) + k2 * ( w[4] - w[14] ) + k3 * ( w[6] - w[12] );  //+ k4 * ( w[8] - w[10] );
    vali >>= 8;
    t = k4 * ( w[8] - w[10] );
    vali += (t >> 8);
}

#define TX_REF (0x40 | 0x80)                 // 1.1 volt reference
#define _UA   5411                           // make same as sample rate

void tx_process(){
int cin;
static uint8_t R;
static int16_t y1;
static int16_t y2, y1d;
static int txin, txout;

       cin = ADC - 512;
       ADMUX = TX_REF | 2;                   // I think we can ignore all button inputs during tx, no stealing ADC readings
       ADCSRA = 0xc0 + 5;                    // next start conversion

       // one stage of CIC, final rate is half of interrupt rate
       y1 = y1 + cin;                        // integrator
       R ^= 1;
       if( R ) return;
       
       y2 = y1 - y1d;   y1d = y1;            // comb length 2
       y2 >>= 4;                             // drop the 2 extra CIC bits and go from 10 bits to 8 bits

       if( tx_process_flag < TXPIPE-1 ){
          TVal[txin++] = y2;
          ++tx_process_flag;
          txin &= TXPIPE-1;
       }
       #ifdef I2STATS
         else ++tx_overruns;                           // no room in buffer
       #endif  

       if( tx_ready_flag ){    
          OCR1BL = Mag[txout];
          if( mode != AM ){
             si5351.freq_calc_fast(Phase[txout]);                     // could this be done in process2 ?
                                                                      // NO, it needs to be in the pipeline for correct sync
             from_int = 1;
             if( i2done )  si5351.SendPLLBRegisterBulk(); 
             else{                                                          // skip and count error, clear buffer
                //uint32_t tm = micros();
                //while( i2done == 0 ) i2poll();                             // clear i2 buffer, in interrupt no noInterrupts needed
                polling = 1;                                                 // hang above ststement on tx?
                //if( micros() - tm  < 10 ) si5351.SendPLLBRegisterBulk();   // just a little late, maybe the next one will catch up
                #ifdef I2STATS
                  //else ++tx_i2late;                      // did not send phase update to si5351, same freq as last time
                  ++tx_i2late;
                #endif  
             }
             from_int = 0;
          }
          ++txout;
          txout &= TXPIPE-1;
          --tx_ready_flag;
       }
       #ifdef I2STATS
         else ++tx_underruns;                    // nothing ready to send
       #endif  

}

  // TVal[] Phase[] Mag[]  tx_process_flag  tx_ready_flag

void tx_process2(){          // do as much as possible outside of interrupt context, allows i2c interrupts to run
static int16_t mag;
static int prev_phase;
int dp, phase;
int y2;
static uint8_t tx_in, tx_out;
static int16_t comp;
static uint8_t mod;
static int16_t a,b,c;        // delay line mag to phase adjustment


       noInterrupts();
       y2 = TVal[tx_in++];
       tx_in &= TXPIPE-1;
       --tx_process_flag;
       interrupts();

       // microphone compression or gain.  with 250/32 have almost 18db of mic gain possible
       y2 *= comp;                                       // mult by compression/gain factor
       y2 >>= 5;                                         // divide by 32
       if( abs( y2 ) > 96 ) --comp;                      // fast attack, aim for max signal into following processes
       if( ++mod == 0 ){                                 // slow decay                                 
          if( comp < 250 ) ++comp;                       // !!! could make 250 a variable in the menu, mike gain
       }

       y2 = CIC_comp( y2 );                              // CIC compensation filter, adds more high tones

       if( mode == AM ) mag = ccAM(y2), dp = -1000;      // controlled carrier AM
       
       else{                                             // SSB
          tx_hilbert( y2 );                              // get valq and vali
          mag = fastAM( vali, valq );
          mag *= Pmax;                                   // slope, Pmax is 0 to 63 to stay in 16 bits
          mag >>= 6;
       // mag += Pmin;                                   // y intercept
          mag = constrain( mag + Pmin, 0, 255 );         // clip to 8 bits 
          phase  = arctan3( vali, valq );                // !!! swapped I and Q from T3.2 version
          dp = phase - prev_phase;                       // delta phase
          prev_phase = phase;
          if( dp < -_UA/2 ) dp += _UA;
          if( mode == LSB ) dp = -dp;
          dp = constrain(dp,-3000,3000);
       }   

       if( tx_inhibit ) mag = 0;
       if( tune_pwr ) mag >>= 1;

       // delay line for mag (or phase ) for best sync of phase and magnitude
       c = mag;
       mag = b;                     // 1 sample delay
       //mag = a;                   // 2 sample delay
       //a = b;
       b = c;
       
       if( tx_ready_flag < TXPIPE -1 ){
          noInterrupts();
          Phase[tx_out] = dp;
          Mag[tx_out++] = mag;
          tx_out &= TXPIPE-1;
          ++tx_ready_flag;
          interrupts();
       }
  
}

int16_t ccAM( int16_t mag ){             // controlled carrier AM
static int16_t carrier;
static uint8_t mod;
int16_t mx;
   
   mag *= Pmax;
   mag >>= 6;
   if( (mag + carrier) < Pmin ) carrier = abs( mag ) + Pmin + 10;
   ++mod;
   mod &= 63;
   if( mod == 0 ) --carrier;
   carrier = constrain( carrier, Pmin, maxAM);         // max carrier should be 1/2 of max signal
   mag = constrain((mag+carrier),0, 255 );             // clip to 8 bits
   
   return mag;
}

inline int16_t fastAM( int16_t i, int16_t q ){           // estimation of sqrt( i^2 + q^2 )

  
    // quicker version   max + min/4
       i = abs(i);   q = abs(q);  
       if( i > q ) q >>= 2;
       else i >>= 2;
       return ( i + q );

  /*
     // better accuracy version   max or 7/8 max + 1/2 min
int16_t mb4; 

    i = abs(i), q = abs(q);
    if( i > q ){
       mb4 = i;   i = q;  q = mb4;         // i ^= q; q ^= i; i ^= q; interesting trick to swap in place
    }
    mb4 = q >> 2;
    if( i < mb4 ) return q;
    q -= ( mb4 >> 1 );                     // 7/8 * q == q - 1/8q
    i = i >> 1;
    
    return ( i + q );
    */
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
// CIC filter terms.  Integrate, decimate by 4, Comb filter N=3 R=4 M=1
static int y1,y2,y3,y4,y5,y6;              // 16 bit calculation, input values of 10 bit should produce 16 bit result.
static int y3d, y4d, y5d;                  // delay terms
static int8_t R;                           // decimation Rate
static int z1,z2,z3,z4,z5,z6;
static int z3d, z4d, z5d;

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
       y3 = y3 + (y2 = y2 + ( y1 = y1 + a1 ));        // three cascaded integrators, input is a1
       a1 = b1; b1 = c1;                  // move the A/D delay line
       R = (R+1) & 3;                     // mod 4 counter
       if( R ) return;                    // decimate by 4, process on zero, skip 1 2 3
       
                                          // 1/4 rate processing
       y4 = y3 - y3d;   y3d = y3;         // three comb filters, length is decimation rate ( 4 )
       y5 = y4 - y4d;   y4d = y4;         // but only one delay term is needed as running at 1/4 rate
       y6 = y5 - y5d;   y5d = y5;

       y6 >>= bits;                       // shift out extra bits.  
      
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
       z3 = z3 + (z2 = z2 + ( z1 = z1 + a2 ));           // three cascaded integrators
       a2 = b2;  b2 = c2;  c2 = d2;          // move A/D delay terms, extra term for 1/2 sample delay

       // R = (R+1) & 3;                     // this was done in above code
       if( R ) return;                       // decimate by 4

       z4 = z3 - z3d;   z3d = z3;         // three comb filters, length is decimation rate ( 4 )
       z5 = z4 - z4d;   z4d = z4;         // but only one delay term is needed as running at 1/4 rate
       z6 = z5 - z5d;   z5d = z5;
       z6 >>= bits;                       // shift out extra bits

       // have y6 and z6 , I and Q at 1/4 rate
       
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
          Qr[inrx] = y6;  Ir[inrx++] = z6;
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


//int dbval;   //!!! debug

// some non-interrupt rx processing,  will this work from loop(), added a pipeline as it fell behind.
// calc rx_val to be sent to PWM audio on the rx interrupt.  Hilbert phasing version.
void rx_process2(){
int val;
static uint8_t rxout, rxin;
int I,Q;
static int16_t sig_level; 
//static int16_t agc = 1;         // changed variable to a global
static uint8_t agc_counter;       // fast agc with the uint8
static uint8_t agc_recover;
static int16_t q_delay[16];
static int8_t quin;


   noInterrupts();
   I = Ir[rxout];
   Q = Qr[rxout];
   --rx_process_flag;
   interrupts();
   ++rxout;  rxout &= (RXPIPE-1);

   // at 16 bits, need to go to 8 bits while keeping as many information bits as possible
   // impliment a 4 bit compressor, always drop 4 bits, bits are dropped in the CIC filter process
   // need to be at 8 bits for hilbert processing

   if( I > sig_level ) sig_level = I;                    // peak signal detect
   
   if( ++agc_counter == 0 && agc_on ){
      noInterrupts();        
      if( sig_level > 96 && bits < 8  ) ++bits;             // 6 db agc steps 
      else if( sig_level < 40 && bits > 4 ){                // always drop 4 bits of the 16, just noise bits I think
         if( ++agc_recover == 4 ) --bits, agc_recover = 0;  // slower recover than attack
      }
      sig_level = 0;
      interrupts();
      if( bits <= 4 && agc < ( volume+volume ) ) ++agc;     // small signal boost
      else if( agc < volume ) ++agc;
      // smeter( bits, agc );                               // crash, recursive, need to call from loop
   }


   // Async complex AM detector
   // https://www.dsprelated.com/showarticle/938.php
   // Figure 4.  Hilbert and delay not needed as we already have I and Q signals.
   //val = ( abs(I)/2 + abs(Q)/2 );    the fastAM sounds better

   if( mode == AM ){                         // decoding AM at baseband does not work well at all
      val = fastAM( I/2, Q/2 );              // tune off freq and notch out the carrier
      val = 2*qdhigh( val );                 // remove DC offset
   }
   else{
      tx_hilbert( I );                       // tx hilbert also used for RX
      //I = vali;
      q_delay[quin] = Q;                     // circular delay buffer
      //quin = (quin+1) & 15;                // for length 31 hilbert
      if( ++quin > 9 ) quin = 0;             // delay for length 19 hilbert
      //Q = q_delay[quin];
      val = vali - q_delay[quin];            // val = I - Q;
   }
  

   val = IIRdf1( val, kf, wi, wo );                  // direct form 1 filter, constants kf updated for desired filter
   if( mode != CW )  val = CIC_comp( val );          // peak higher freqs
   else goertzel( val );

   if( agc_on ) val *= agc;
   else val *= volume;
   val >>= 6;
   if( (abs(val)) > 2*(int)volume && agc > 0 ) --agc;     // volume ranges 0 to 63, signal +- 128

   //dbval = val;

   val = constrain( val + 128, 0, 255 );             // clip to 8 bits, zero offset for PWM

   noInterrupts();
   rx_val[rxin] = (uint8_t)val;              // queue value to be sent to audio PWM during timer2 interrupt
   ++rx_ready_flag;
   interrupts();
   ++rxin;  rxin &= (RXPIPE-1);

}


// sample rate 6000, 10ms N = 60, k = 6? for 600hz tone, q = 2*cos(2pi*K/N), a = inval + q*b - c
// sample rage 5000, 10ms N = 50, k = 7 for 600 hz tone, 
void goertzel( int16_t val ){
const int8_t q = (int8_t)( 1.2748 * 64.0 );
const int8_t cs= ( int8_t)(0.637 * 64.0 );
const int8_t sn= ( int8_t)(0.770 * 64.0 );
static int16_t a,b,c;                           // delay terms
static int8_t i;                                // iterations

   if( i == 0 ){                                // done, save result
      a = (c * cs) >> 6;
      a = b - a;
      c = (c * sn ) >> 6;
      cw_signal = fastAM(a,c);
      cw_count = 1;
      b = c = 0;                                // reset
   }
  
   a = q*b >> 6;
   a += val - c;
   c = b;  b = a;
   
   if( ++i == 50 ) i = 0;
  
}

// CIC compensation: -Alpha/2, 1 + Alpha, -Alpha/2  3 tap FIR.
int16_t CIC_comp( int16_t val ){
static int16_t a,b,c;

   a = b;  b = c; c = val;
   val = a_plus1*b - a_by2*( a + c );
   return ( val >> 6 ); 
}



// 8 bit one pole highpass filter  k's are 0.863272, 0.726542, 0, -1, 0   in q6 are 55 46 0 -64 0. Direct form 2
int16_t qdhigh( int16_t inval ){
int16_t val;
static int16_t w0,w1;

   // w0 = a0*value + w1*a1 + w2*a2     a2 is zero
   val = 55 * inval + 46 * w1;
   w0 = val >> 6;

   // val = w0 + (w2)*b2 + (w1)*b1      b2 is zero
   val -= 64 * w1;
   val >>= 6;
   w1 = w0;
   return val;
}


void smeter( uint8_t db, int16_t agc ){
static int8_t bars;
int8_t b,i,k;

  // LCD.printNumI( dbval, CENTER, ROW6, 5, ' ' );   // debug

   if( transmitting ) return;
   
      if( menu_on ){
        bars = 0;
        return;                   // screen area used by the menu
      }
      
      // how many bars to show, 6db steps
      b = 1;
      if( attn & 1 ) b += 2;
      b += ( db - 4 );
      //if( agc >= 2*volume ) --b;
      if( agc <= ( volume >> 1 )) ++b;
      if( agc <= ( volume >> 2 )) ++b;
      if( agc <= ( volume >> 4 )) ++b;

      if( b == bars ) return;                // no change

      // just display or erase what is needed

      if( bars == 0 ){
         LCD.clrRow(0,0,60);
         LCD.clrRow(1,0,60);
         LCD.gotoRowCol( 1, 0 );
         LCD.putch('S');
         LCD.write(0);
      }

      if( b > bars ){                        // write new bars
         LCD.gotoRowCol( 1, 8 + 4*bars );
         k = 0xc0;
         for( i = 1; i < bars; ++i ){
            k >>= 1;
            k |= 0x80;
            if( k == 0xff ) break;
         }
         for( i = bars; i < b; ++i ){
            LCD.write(k,3);
            LCD.write(0);
            if( k != 0xff ){                 // grow size of the s meter bars
                k >>= 1;
                k |= 0x80;
            }
         }
      }
      else{                                  // erase
         b = bars - 1;                       // erase just one bar
         LCD.gotoRowCol( 1, 8 + 4*b );     
         LCD.write(0,4);
      }
      
      bars = b;
      LCD.printNumI( bars, RIGHT, ROW6 );    // !!! just for debug

}




// general 2 section direct form 1 filter. Q6 constants as produced by Iowa Hills program , a0 is 1.0, a1 a2 uses subs instead
// of adds.  Constants will be in order b0,b1,b2,a1,a2,b0,b1,b2,a1,a2 or 10 total, 6 input delay, 6 output delay
int16_t  IIRdf1( int16_t val, int8_t *k, int16_t *wi, int16_t *wo ){
uint8_t i;

   for( i = 5; i > 0; --i ){                        // move delays
      wi[i] = wi[i-1];
      wo[i] = wo[i-1];
   }

   // two sections
   for( i = 0; i < 2; ++i ){
      wi[0] = val;
      val = *k++ * wi[0] + *k++ * wi[1] + *k++ * wi[2] - *k++ * wo[1] - *k++ * wo[2];
      val = constrain( (val >> 6),-128,127 );
      wo[0] = val;
      wi += 3;  wo += 3;                           // next section
   }   
   
   return val;
}



// ***************   group of functions for a read behind morse decoder    ******************
//   attempts to correct for incorrect code spacing, the most common fault.

int8_t cread_buf[16];
int8_t cread_indx;
int8_t dah_table[8] = { 20,20,20,20,20,20,20,20 };      // morse speed determined by length of the dah's
int8_t dah_in;                                          // designed to work up to 25 wpm


int8_t cw_detect(int16_t sig ){
uint8_t det;                       // cw mark space detect
static int8_t count;               // mark,space counts
int8_t stored;
static uint8_t mod;                // morse scope speed
static uint8_t wav;                 
static uint8_t col;


   det = ( sig > 50 ) ? 1 : 0;
   det = cw_denoise( det );

   if( det ) wav >>= 1;                    // morse signal graph       
   if( ++mod == 3 ){
      LCD.gotoRowCol( 5,col );
      if( wav != 0x80 ) LCD.write(wav);
      else LCD.write(0);
      if( ++col > 127 ) col = 0;
      wav = 0x80;
      mod = 0;
   }

   stored = 0;               // when find a change in the signal, save the counts for later use
   if( det ){                // marking, negative counts
      if( count > 0 ){
         if( count < 99 ) storecount(count), stored = 1;
         count = 0;
      }
      --count;
   }
   else{                     // spacing
      if( count < 0 ){
        storecount(count), stored = 1;
        count = 0; 
      }
      if( count < 120 ) ++count;                        // keep from overflowing to -127
      if( count == 99 ) storecount(count), stored = 1;  // one second no signal
   }
   
   return stored;
}

// slide some bits around to remove 1 reversal of mark space
int8_t cw_denoise( int8_t m ){
static int8_t val;

   if( m ){
      val <<= 1;
      val |= 1;
   }
   else val >>= 1;

   val &= 7;
   if( m ) return val & 4;      // need 3 marks in a row to return true
   else return val & 2;         // 1 extra mark returned when spacing
                                // so min mark count we see is -2 if this works correctly
                                // this shortens mark count by 1 which may be ok
}

// store mark space counts for cw decode
void storecount( int8_t count ){

     cread_buf[cread_indx++] = count;
     cread_indx &= 15;

     if( count < 0 ){      // save dah counts
        count = -count;
        if( count >= 12 ){      // 12 for 10ms per sample, works up to 25 wpm
          dah_table[dah_in++] = count;
          dah_in &= 7;
        }
     }
}

void shuffle_down( int8_t count ){    /* consume the stored code read counts */
int8_t i;

  for( i= count; i < cread_indx; ++i ){
    cread_buf[i-count] = cread_buf[i];
  }
  
  cread_indx -= count;
  cread_indx &= 15;     // just in case out of sync  
}


int8_t code_read_scan(int8_t slice){  /* find a letter space */
int8_t ls, i;

/* scan for a letter space */
   ls = -1;
   for( i= 0; i < cread_indx; ++i ){
      if( cread_buf[i] > slice ){
        ls = i;
        break;
      }
   }
   return ls;   
}


unsigned char morse_lookup( int8_t ls, int8_t slicer){
unsigned char m_ch, ch;
int8_t i,elcount;

   /* form morse in morse table format */
   m_ch= 0;  elcount= 1;  ch= 0;
   for( i = 0; i <= ls; ++i ){
     if( cread_buf[i] > 0 ) continue;   /* skip the spaces */
     if( cread_buf[i] < -slicer ) m_ch |= 1;
     m_ch <<= 1;
     ++elcount;
   }
   m_ch |= 1;
   /* left align */
   while( elcount++ < 8 ) m_ch <<= 1;

   /* look up in table */
   for( i = 0; i < 47; ++i ){
      if( m_ch == pgm_read_byte(&morse[i]) ){
        ch = i+',';
        break;
      }
   }

   return ch;  
}


void code_read( int16_t val ){  /* convert the stored mark space counts to a letter on the screen */
int16_t slicer;
int8_t i;
unsigned char m_ch;
int8_t ls,force;
static int8_t wt;    /* heavy weighting will mess up the algorithm, so this compensation factor */
static int8_t singles;
static int8_t farns,ch_count;
static int8_t eees;

   if( transmitting || menu_on ) return;   

   if( cw_detect( val ) == 0 && cread_indx < 15 ) return;
   if( cread_indx < 2 ) return;    // need at least one mark and one space in order to decode something

   /* find slicer from dah table */
   slicer= 0;   force= 0;
   for( i = 0; i < 8; ++i ){
     slicer += dah_table[i];
   }
   slicer >>= 4;   /* divide by 8 and take half the value */

   ls = code_read_scan(slicer + wt);
   
   if( ls == -1 && cread_indx == 15 ){   // need to force a decode
      for(i= 1; i < 30; ++i ){
        ls= code_read_scan(slicer + wt - i);
        if( ls >= 0 ) break;
      } 
      --wt;    /* compensate for short letter spaces */
      force= 1;
   }
   
   if( ls == -1 ) return;
   
   m_ch = morse_lookup( ls, slicer );
   
   /* are we getting just E and T */
   if( m_ch == 'E' || m_ch == 'T' ){   /* less weight compensation needed */
      if( ++singles == 4 ){
         ++wt;
         singles = 0;
      }
   }
   else if( m_ch ) singles = 0;   
  
   /* if no char match, see if can get a different decode */
   if( m_ch == 0 && force == 0 ){
     ls = code_read_scan( slicer + wt - ( slicer >> 2 ) );
     m_ch = morse_lookup( ls, slicer );
     if( m_ch > 64 ) m_ch += 32;       // lower case for this algorithm
     //if( m_ch ) --wt;     this doesn't seem to be a good idea
   }
 
   if( m_ch ){   /* found something so print it */
      ++ch_count;
      if( m_ch == 'E' || m_ch == 'I' ) ++eees;  // suppress noise prints
      else eees = 0;
      if( eees < 5 ){
         decode_print(m_ch);
      }
      if( cread_buf[ls] > 3*slicer + farns ){   // check for word space
        if( ch_count == 1 ) ++farns;            // single characters, no words printed
        ch_count= 0;
        decode_print(' ');
      }
   }
     
   if( ls < 0 ) ls = 0;   // check if something wrong just in case  
   shuffle_down( ls+1 );  

   /* bounds for weight */
   if( wt > slicer ) wt = slicer;
   if( wt < -(slicer >> 1)) wt= -(slicer >> 1);
   
   if( ch_count > 10 ) --farns;
   
}


// print on rows 6 and 7
void decode_print( char c ){
static uint8_t row = 6, col = 0;

   
   LCD.gotoRowCol( row, col );
   LCD.putch(c);
   col += 6;
   if( col > 126 - 6 ){
      col = 0, ++row;
      if( row > 7 ) row = 6;
   }
   else LCD.putch(' ');                // erase one char ahead

}

//  ***************   end of morse decode functions







#ifdef I2STATS
 void print_i2stats(){
  
           if( transmitting ) return;
           // can we write debug info to the OLED in cw mode transmit without any issues? 
           // if( mode != CW && transmitting ) return;
           
           uint16_t a,b,c,d,e,f,g,h,i,j;
           noInterrupts();
           a = i2ints;  b = i2polls;  c = i2stalls;
           i2ints = i2polls = i2stalls = 0;
           d = rx_overruns;  rx_overruns = 0;      // added rx_process debugging to this, commented out i2stats
           e = rx_underruns;  rx_underruns = 0;
           f = rx_ready_flag;
           g = tx_ready_flag;
           h = tx_underruns;  tx_underruns = 0;
           i = tx_overruns;   tx_overruns = 0;
           j = tx_i2late;      tx_i2late = 0;
           interrupts();
           LCD.clrRow( 7 );
           //LCD.clrRow( 6 );
          // LCD.printNumI( a, LEFT, ROW7 );       // should be biggest number, interrupts working
          // LCD.printNumI( b, CENTER, ROW7 );     // some OLED functions may overfill our buffer, polling counts here
          // LCD.printNumI( c, RIGHT, ROW7 );      // polling may cause int flags out of sync with i2state, counts here
          
          //LCD.printNumI( d, LEFT, ROW7 );          // rx_process overruns
          //LCD.printNumI(ADCSRA & 7 , CENTER, ROW7 );    // check auto ADC baud rate value
          //LCD.printNumI( f, CENTER, ROW7 );        // pipeline delay
          //LCD.printNumI( e, RIGHT, ROW7 );         // rx underruns

          
          LCD.printNumI( j, RIGHT, ROW6 );         // tx i2c late
          LCD.printNumI( g, CENTER, ROW7 );        // tx pipeline
          LCD.printNumI( h, RIGHT, ROW7 );         // tx underruns
          LCD.printNumI( i, LEFT, ROW7 );          // tx overruns  
 }
#endif


// txtest timing loop
// !!! commment baud change in i2start, then fix when done with this test
// !!! comment timer setups in setup
// !!! check send fast optimization
//  best sample rates with small work load, best we can hope for.  I2C bus restricted
//  baud 4  5785 with waits
//  baud 1  6447 with waits  .
//  TWBR as 1 is 1.1111 meg.  TWBR as 2 is 1.0 meg. Not sure Si5351 is receiving these bauds correctly.
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
//  Thought a faster baud than needed will add to interrupt overhead, but not really seeing it in this test.
//  7 I2C xfers say 70 bits will take 84us+overhead at 833 baud
//  road to improvement, Speed up calc fast if possible.   Less I2C transfers -> small improvement.
//  Maybe the Hilbert is too long, gained 500 hz in test without.
/*
void looptxtest(){
static int twbr = 14;
static int row = 2;
unsigned int notdone, loops;
unsigned long tm;
static uint16_t btw,bloops,bndone;

   notdone = loops = 0;

   tm = millis();
   TWBR = twbr;
   
   si5351.freq_calc_fast(1000);   // ?? why is this here
   //i2flush();
   
   while( i2done == 0 ){
     noInterrupts();
     i2poll();
     interrupts();
   } 
   
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
*************/

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

/*    Weaver mode, bfo leaks through on weak signals, Q6 constants too small ?
// some non-interrupt rx processing,  will this work from loop(), added a pipeline as it fell behind
// calc rx_val to be sent to PWM audio on the rx interrupt 
void rx_process2(){
int val;
static uint8_t rxout, rxin;     // local indexes rather than shared
static int16_t wq[9], wi[9];    // IIR filter delay terms
static int16_t wh[2];           // 1 pole highpass delay terms
int I,Q;
static uint8_t i;
static int16_t agc = 1;
static uint8_t agc_counter;

   noInterrupts();
   I = Ir[rxout];
   Q = Qr[rxout];
   --rx_process_flag;
   interrupts();
   ++rxout;  rxout &= (RXPIPE-1);

   if( I > sig_level ) sig_level = I;                // peak signal detect
   if( agc_counter == 0 ){
      noInterrupts();                                // keeping some of the extra CIC bits
      if( sig_level > 256 && bits < 4 ) ++bits;
      else if( sig_level < 16 and bits > 1 ) --bits;
      sig_level = 0;
      interrupts();
   }

   //I = IIR2( I, k1, wi );        // 2 section eliptic filter, longer processing time
   //Q = IIR2( Q, k1, wq );

   //I = IIR2BW( I, k1ch, wi );       // 2 section butterworth or cheby
   //Q = IIR2BW( Q, k1ch, wq );

   I = IIR3sp( I, wi );              // 3 section special butterworth 1/4 sr cutoff
   Q = IIR3sp( Q, wq );

   // simplest weaver decoder, 90 degree sine, cosine steps, 1502 bfo freq with current sample rate
   ++i;
   i &= 3;
   val = ( i & 1 ) ? I : Q;
   if( i & 2 ) val = -val;       // get Q,I,-Q,-I...
   
   // ?? do we need a highpass filter here to remove DC ( sin^2 + cos^2 = 1^2 = 1 )
   // https://www.dspguide.com/ch19/2.htm
   
   val >>= 2;       // to 8 bits, if iir2 filter overloads, shift out two bits before IIR2 calls
   val = qdhigh(val, wh );
   
  // val = Q >> 2;      // !!! DC receiver
   // need to be at <= 10 bits for this calc
   val *= agc;   // volume or agc
   val >>= 6;

   if( ++agc_counter == 0 && agc < 4*volume ) ++agc;
   if( abs(val) > 2*volume ) --agc;

   val = constrain( val + 128, 0, 255 );     // clip to 8 bits, zero offset for PWM
   noInterrupts();
   rx_val[rxin] = (uint8_t)val;              // queue value to be sent to audio PWM during timer2 interrupt
   ++rx_ready_flag;
   interrupts();
   ++rxin;  rxin &= (RXPIPE-1);

}
********/

/*
 * 

/*
// IIR filter, design values:  LowPass Eliptic, sr 5973, band edges 1400,2000, ripple/reject 0.5db 50db
// actual sample rate 6009                               has a ringing/distortionl,I think at the weaver bfo
// two sections a0,a1,a2,b1,b2     Q6 constants
const int8_t k1[] = {
    (int8_t)( ( 0.287755 ) * 64.0 ),
    (int8_t)( ( 0.614265 ) * 64.0 ),
    (int8_t)( ( -0.250054 ) * 64.0 ),
    (int8_t)( ( 1.793725  ) * 64.0 ),
    (int8_t)( ( 1.0 ) * 64.0 ),
    (int8_t)( ( 0.287755  ) * 64.0 ),
    (int8_t)( ( 0.118652  ) * 64.0 ),
    (int8_t)( ( -0.748486 ) * 64.0 ),
    (int8_t)( ( 1.114201  ) * 64.0 ),
    (int8_t)( ( 1.0 ) * 64.0 )
};

// Lowpass Eliptic, sr 6000, 1500 2200,  0.2 50    Also rings.
const int8_t k1[] = {
    (int8_t)( ( 0.335063 ) * 64.0 ),
    (int8_t)( ( 0.333035 ) * 64.0 ),
    (int8_t)( ( -0.15957 ) * 64.0 ),
    (int8_t)( ( 1.876546  ) * 64.0 ),
    (int8_t)( ( 1.0 ) * 64.0 ),
    (int8_t)( ( 0.335063  ) * 64.0 ),
    (int8_t)( ( -0.143943  ) * 64.0 ),
    (int8_t)( ( -0.695227 ) * 64.0 ),
    (int8_t)( ( 1.413391  ) * 64.0 ),
    (int8_t)( ( 1.0 ) * 64.0 )
};

// 2 section butterworth, 1300hz  cutoff, 6000 hz sample rate 
const int8_t k1bw[] = {               // a0 a1 a2, for butterworth b1 is always 2.0,  b2 is 1.0
  (int8_t)( ( 0.244851  ) * 64.0 ),
  (int8_t)( ( 0.218430  ) * 64.0 ),
  (int8_t)( ( -0.050591  ) * 64.0 ),

  (int8_t)( ( 0.244851 ) * 64.0 ),
  (int8_t)( ( 0.302566  ) * 64.0 ),
  (int8_t)( ( -0.455264  ) * 64.0 )
  
};

/*
// 2 section Cheby 1500hz cutoff 0.3db 45db.  Can run with butterworth function as b1 b2 is 2.0 and 1.0
const int8_t k1ch[] = {                       // sounds good with highs present, may be the station I am listening to
  (int8_t)( ( 0.290464  ) * 64.0 ),           // audio image rejection not as good as the eliptic
  (int8_t)( ( 0.394597  ) * 64.0 ),           // has funny ringing at 3k, aliasing? 
  (int8_t)( ( -0.157573  ) * 64.0 ),

  (int8_t)( ( 0.290464 ) * 64.0 ),
  (int8_t)( ( -0.15003  ) * 64.0 ),
  (int8_t)( ( -0.660443  ) * 64.0 )
  
};


// 2 section Cheby 1400hz cutoff 0.3db 45db.  Can run with butterworth function as b1 b2 is 2.0 and 1.0, rings AGC issue?
const int8_t k1ch[] = { 
  (int8_t)( ( 0.245772  ) * 64.0 ),
  (int8_t)( ( 0.587437  ) * 64.0 ), 
  (int8_t)( ( -0.208894  ) * 64.0 ),

  (int8_t)( ( 0.245772) * 64.0 ),
  (int8_t)( ( 0.070725 ) * 64.0 ),
  (int8_t)( ( -0.680535  ) * 64.0 )
  
};

// special 3 section butterworth, 1/4 sample rate cutoff.  Constants hardcoded, many constants are zero.
// needs 9 delay terms, k's are .309, 0, .017332, 2, 1.  .309, 0, 0.171573, 2, 1.  .309, 0, -0.588791, 2, 1
int16_t  IIR3sp( int16_t val, int16_t *w ){
long accm;
      
   // w0 = a0*value + w1*a1 + w2*a2    a1 is zero
     accm = 20*val + w[2];
     accm >>= 6;
     *w = accm;
     
   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms ( terms backwards for the dmov TMS32xxxx )
     accm = accm + w[2] + w[1] + w[1];     // b1 = 2, b2 = 1
     *(w+2) = *(w+1);  *(w+1) = *w;

     w += 3;      // second section

     accm = 20*accm + 11 * w[2];
     accm >>= 6;
     *w = accm;
     accm = accm + w[2] + w[1] + w[1];     // b1 = 2, b2 = 1
     *(w+2) = *(w+1);  *(w+1) = *w;

     w += 3;      // third section

     accm = 20*accm - 38 * w[2];
     accm >>= 6;
     *w = accm;
     accm = accm + w[2] + w[1] + w[1];
     *(w+2) = *(w+1);  *(w+1) = *w;
     
}

*/



/*
int16_t qdnotch( int16_t inval ){    // wide band noise, fixed, but oscillates on sharp noise
int32_t val;
static int16_t w0,w1,w2;
static int16_t x0,x1,x2;

  // w0 = a0*value + w1*a1 + w2*a2
  val = 64*inval + 79*w1 - 50*w2;
  w0 = val >> 6;
   
  // val = b0*w0 + (w2)*b2 + (w1)*b1
  val = 57*w0 + 57*w2 - 67*w1;            // !!! optimize 57( w0+w2)
  //val >>= 6;
  w2 = w1;  w1 = w0;

  // 2nd section
  // w0 = a0*value + w1*a1 + w2*a2
  val = val + 52*x1 - 48*x2;
  x0 = val >> 6;
   
  // val = b0*w0 + (w2)*b2 + (w1)*b1
  val = 54*x0 + 54*x2 - 63*x1;            // !!! optimize
  val >>= 6;
  x2 = x1;  x1 = x0;

  return val;  
}
*/
/*
//  6 constants.  a0 a1 a2    a0 a1 a2.  b1 is 2.0  b2 is 1.0
//  6 time delay, 3 for each section
int16_t IIR2BW( int16_t inval, int8_t k[], int16_t *w ){   // 2 section IIR butterworth
long accm;
      
   // w0 = a0*value + w1*a1 + w2*a2
     accm = k[0]*inval + k[1]*w[1] + k[2]*w[2];
     accm >>= 6;
     *w = accm;
     
   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms ( terms backwards for the dmov TMS32xxxx )
     accm = accm + w[2] + w[1] + w[1];     // b1 = 2, b2 = 1
     *(w+2) = *(w+1);  *(w+1) = *w;

     w += 3;      // second section

        // w0 = a0*value + w1*a1 + w2*a2
     accm = k[3]*inval + k[4]*w[1] + k[5]*w[2];
     accm >>= 6;
     *w = accm;

   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms ( terms backwards for the dmov TMS32xxxx )
     accm = accm + w[2] + w[1] + w[1];
     *(w+2) = *(w+1);  *(w+1) = *w;
    
     return (int16_t)accm;
}

//;                storage needed per section, so double for two sections
//;  5 constants,  a0   a1  a2    b1  b2         using Q6 values
//;  3 time delay  w0   w1  w2
//  mults are 8bits * 16bits,  adds are 32 bits

int16_t IIR2( int16_t inval, int8_t k[], int16_t *w ){     // 2 section IIR
long accm;

     //accm = 0;
      
   // w0 = a0*value + w1*a1 + w2*a2
     accm = k[0]*inval + k[1]*w[1] + k[2]*w[2];
     *w = accm >> 6;
     
   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms ( terms backwards for the dmov TMS32xxxx )
     accm = accm + k[4]*w[2] + k[3]*w[1];
     accm >>= 6;
     *(w+2) = *(w+1);  *(w+1) = *w;

   // 2nd section
     w += 3;
     
     // w0 = a0*value + w1*a1 + w2*a2
     accm = k[5]*accm + k[6]*w[1] + k[7]*w[2];    
     *w = accm >> 6;
     
   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms
     accm = accm + k[9]*w[2] + k[8]*w[1];
     accm >>= 6;
     *(w+2) = *(w+1);  *(w+1) = *w;

     return (int16_t)accm;
}

*/

/*
// 2 section butterworth, 2800hz  cutoff, 6793 hz sample rate 
const int8_t k1bw[] = {               // a0 a1 a2, for butterworth b1 is always 2.0,  b2 is 1.0
  (int8_t)( ( 0.693535  ) * 64.0 ),
  (int8_t)( ( -1.147529  ) * 64.0 ),
  (int8_t)( ( -0.347469  ) * 64.0 ),

  (int8_t)( ( 0.693535 ) * 64.0 ),
  (int8_t)( ( -1.418667  ) * 64.0 ),
  (int8_t)( ( -0.665849  ) * 64.0 )
  
};
*/

/*
// a less general version with the idea that the filter will be called only once, delay sections can be static local
// the general version seems to compile to shorter mult section and inlines the loops
int16_t  IIRdf1_2( int16_t val, int8_t *k ){
uint8_t i;
static int16_t wi[6], wo[6];

   for( i = 5; i > 0; --i ){                        // move delays
      wi[i] = wi[i-1];
      wo[i] = wo[i-1];
   }

   // 1st section
      wi[0] = val;
      val = *k++ * wi[0] + *k++ * wi[1] + *k++ * wi[2] - *k++ * wo[1] - *k++ * wo[2];
      val = constrain( (val >> 6),-128,127 );
      wi[3] = wo[0] = val;
   // 2nd section  
      val = *k++ * wi[3] + *k++ * wi[4] + *k++ * wi[5] - *k++ * wo[4] - *k * wo[5];
      val = constrain( (val >> 6),-128,127 );
      wo[3] = val;
   
   return val;
}
*/

/*
//  Cheby bandpass 300 - 900 at 6000 sr, .3 ripple 30 db
//  This filter oscillates sometimes when receive sudden increase in signal
int16_t IIRcw8( int16_t inval ){
static int16_t w[6];
int32_t accm;

   accm = 21*inval + 48*w[1] - 30*w[2];
   accm >>= 6;
   w[0] = constrain(accm,-32000,32000);

   accm = accm + w[2] + w[1] + w[1];
   w[2] = w[1];  w[1] = w[0];

   accm = 10*accm + 110*w[4] - 51*w[5];
   accm >>= 6;
   w[3] = constrain(accm,-32000,32000);

   accm = accm + w[5] - w[4] - w[4];
   w[5] = w[4];   w[4] = w[3];
   return accm;
}
*/

/*
int16_t IIRcw8( int16_t inval ){
static int16_t w[3];
int16_t accm;

   accm = 10*inval + 40*w[1] -15*w[2];
   accm >>= 6;
   w[0] = accm;

   accm = accm + w[2] + w[1] + w[1];
   w[2] = w[1];  w[1] = w[0];
   
   return accm;
}
*/

/*
//  6 constants.  a0 a1 a2    a0 a1 a2.  b1 is 2.0  b2 is 1.0
//  6 time delay, 3 for each section, 8 bit inval
int16_t IIR2BW8( int16_t inval, int8_t k[], int16_t *w ){   // 2 section IIR butterworth
int16_t accm;
      
   // w0 = a0*value + w1*a1 + w2*a2
     accm = k[0]*inval + k[1]*w[1] + k[2]*w[2];
     accm >>= 6;
     *w = accm;
     
   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms ( terms backwards for the dmov TMS32xxxx )
     accm = accm + w[2] + w[1] + w[1];     // b1 = 2, b2 = 1
     *(w+2) = *(w+1);  *(w+1) = *w;

     w += 3;      // second section

        // w0 = a0*value + w1*a1 + w2*a2
     accm = k[3]*inval + k[4]*w[1] + k[5]*w[2];
     accm >>= 6;
     *w = accm;

   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms ( terms backwards for the dmov TMS32xxxx )
     accm = accm + w[2] + w[1] + w[1];
     *(w+2) = *(w+1);  *(w+1) = *w;
    
     return accm;
}
*/
/*
int16_t IIR2_8( int16_t inval, int8_t k[], int16_t *w ){     // 2 section IIR, 8 bit version, max values 128 * 64
int16_t accm;

     accm = 0;
      
   // w0 = a0*value + w1*a1 + w2*a2
     accm = k[0]*inval + k[1]*w[1] + k[2]*w[2];
     *w = accm >> 6;
     
   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms ( terms backwards for the dmov TMS32xxxx )
     accm = accm + k[4]*w[2] + k[3]*w[1];
     accm >>= 6;
     *(w+2) = *(w+1);  *(w+1) = *w;

   // 2nd section
     w += 3;
     
     // w0 = a0*value + w1*a1 + w2*a2
     accm = k[5]*accm + k[6]*w[1] + k[7]*w[2];    
     *w = accm >> 6;
     
   // value = w0 + (w2)*b2 + (w1)*b1    and dmov the w terms
     accm = accm + k[9]*w[2] + k[8]*w[1];
     accm >>= 6;
     *(w+2) = *(w+1);  *(w+1) = *w;

     return accm;
}
*/

// try a direct form I filter instead of the direct form II and see if stable
// swapping signs on the a1 a2 terms for all adds formula
/*
Sect 0
a0   1.000000000000000000
a1   1.585280958391858120   101
a2   -0.741224118467670001   47
b0   -0.298188434384186241   19
b1   0.000000000000000000     0
b2   0.298188434384186241    19

Sect 1
a0   1.000000000000000000
a1   0.959928298017437731    61
a2   -0.558752139873679221   36
b0   -0.224781441464945275   14
b1   0.000000000000000000     0
b2   0.224781441464945275    14
*/

/*
int16_t IIRcw8( int16_t inval ){
static int wi[6], wo[6];
uint8_t i;
int16_t val;


   for( i = 5; i > 0; --i ){
      wi[i] = wi[i-1];
      wo[i] = wo[i-1];
   }
   // 1st section
   wi[0] = inval;
   val = -19*wi[0] + 19*wi[2]  + 101*wo[1] - 47*wo[2];    // b1 is zero  19*(wi[2] - wi[0])
   wo[0] = val >> 6;
   wo[0] = constrain(wo[0],-128,127);                     // seems to be stable with the constrain values

   // 2nd section
   wi[3] = wo[0];
   val = -14*wi[3] + 14*wi[5]  +  61*wo[4] - 36*wo[5];    //   14*(wi[5] - wi[3])
   wo[3] = val >> 6;
   wo[3] = constrain(wo[3],-128,127);
   
   return wo[3];
}
*/
/*
// length 15 padded to 16
// sr  5000 or 6000
const int8_t k_cw[16] =  {
   (int8_t)(-0.026331 * 64),
   (int8_t)(-0.020283 * 64),
   (int8_t)(-0.047333 * 64),
   (int8_t)(-0.097973 * 64),
   (int8_t)(-0.084384 * 64),
   (int8_t)(0.057563 * 64),
   (int8_t)(0.257407 * 64),
   (int8_t)(0.352667 * 64),
   (int8_t)(0.257407 * 64),
   (int8_t)(0.057563 * 64),
   (int8_t)(-0.084384 * 64),
   (int8_t)(-0.097973 * 64),
   (int8_t)(-0.047333 * 64),
   (int8_t)(-0.020283 * 64),
   (int8_t)(-0.026331 * 64),
      0
      }; 


// FIR bandpass center 600 hz
// out of cpu when using the menu, changed to lower sample rate for CW mode
int16_t FIRcw( int16_t inval ){
static int16_t w[16];
int i,k;
static int8_t in;
int16_t accm;

   w[in] = inval;
   in = (in +1) & 15;
   
   accm = 0;
   k = in;

   for( i = 0; i < 16; ++i ){
      accm += k_cw[i] * w[k];
      k = (k+1) & 15;
   }
   return (accm >> 6); 
}
*/
