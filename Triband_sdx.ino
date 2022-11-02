
/*
 * uSDX Triband Radio, OLED version.
 * 
 * Main feature of this software version is interrupt driven I2C routines. We can calculate the next Si5351 transmit information while
 * the I2C bus is busy.
 * 
 * !!! Hardware mod.  The Si5351 module was NOT converted to run 3.3 volts I2C signals. 
 *     Jumpered pin 27 to 4 and 28 to 5 to get I2C signals to the OLED.  Arduino D2, D3 should not be enabled
 *     I am running 5 volt I2C signals.
 *     
 *     !!! Do not use jumpers if your Si5351 is running with 3.3 volt I2C, Si5351 soldered to the board or Modified module.  
 *     Add level converters instead for SDA, SCL, 3.3 side is ATMega328, 5 volt side is OLED. 
 *     Pin 27 to level converter to pin 4.   Pin 28 to level converter to pin 5.
 *     The Mega328 will now run 3.3 volt I2C, the OLED will be at 5 volt, the Si5351 stays at 3.3 volt
 *     This is an untested idea.  The OLED may need a slower baud clock with the extra level converters.
 * 
 * My process:  compile, AVRDudess, file in user/ron/local/appdata/temp/arduino_build_xxxxxx/, pickit 2 as programmer
 * power up with pickit app to test?  does pickit app apply vpp when searching for a device ID?  
 * !!!!!!!!!!  Yes so don't do this.
 * 
 * Button Functions,  buttons respond to TAP, Double TAP, and Long Press
 *   Select: Tap opens menu, another Tap enables edit , DTap opens menu in edit mode, Long Press qsy's -100kc
 *   Exit  : Tap backs out of menu one step, DTap backs out all the way, Long Press qsy's +100kc
 *   Encoder: Long Press opens Volume directly for edit.
 *            Tap changes tuning step 
 *            Dtap
 *            Any press to enable TX after band change, CHECK the switches first!
 *            Any press to turn off RIT
 *   
 */

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
volatile uint8_t  gi2state;
uint8_t i2done = 1;
uint8_t polling;

uint8_t rit_enabled;
uint32_t freq;
uint16_t divider;  
uint8_t transmitting;
int sw_adc;                    // request flag and result of ADC of the switches

uint32_t bandstack[3] = { 21100000UL, 14100000UL, 7100000UL };

#define I2STATS                // see if the i2c interrupts are functioning and being useful in freeing up cpu time
#ifdef I2STATS
  uint16_t i2polls;
  uint16_t i2ints;
  uint16_t i2stalls;
  uint16_t rx_overruns;       //  Added another debug counter non-i2c related
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
uint8_t cal = 128;       // final frequency calibration +-500hz
uint8_t Pmin = 0;
uint8_t Pmax = 50;
uint8_t dv;              // dummy var menu placeholder

uint8_t ad_ref;          // some bits to merge with the A/D mux bits to select the reference voltage

uint8_t oldband;         // for saving freq in bandstack on band change

uint16_t eeprom_write_pending;     // delay eeprom writes while the user fiddles the values

// receiver vars
#define PIPE  64
volatile int rx_val[PIPE], Qr[PIPE], Ir[PIPE];     // post processing pipeline needed
volatile uint8_t rx_process_flag;
volatile uint8_t rx_ready_flag;

#define FREQ_U 0
#define MENU_U 1
uint8_t encoder_user;
int    step_ = 100;
int8_t tx_inhibit = 1;         // start with a band switches check before transmit


#include "si5351_usdx.cpp"     // the si5351 code from the uSDX project, modified slightly for calibrate and RIT
SI5351 si5351;

const char msg1[] PROGMEM = "Check Band Switches";
const char msg2[] PROGMEM = "RIT Enabled";
const char msg3[] PROGMEM = "SDX TriBander";
const char msg4[] PROGMEM = "K1URC wb2cba pe1nnz";


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
  p_msg( msg1,6 );                         // display band switch check message, manual switches
  
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

  //if( sw_adc == -1 ) fake_it();      // !!! debug  Analog read of switches requested.  Remove this, done elsewhere when enabled.

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

     if( ++sec == 60000 ){
        sec = 0;
        #ifdef I2STATS
          print_i2stats();
        #endif
     }
  }           /** end one ms routines **/

}

/*****
void fake_it(){        // !!! test code.   analog read of the switches, fake it for now
  
    sw_adc = analogRead( SW_ADC );
    //LCD.printNumI( sw_adc, RIGHT, ROW6 );   // readings are 772, 932, 1023 for the 3 buttons
}
******/

char read_buttons(){                        // 3 switches on an analog pin
int val;
static char last;

    if( digitalRead( SW_ADC ) == LOW ){     // no switch pressed
        last = 0;
        return 0;
    }
    last = 8;                               // get the vref up to 5 volts, invalid switch
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
      if( sw_true ) ad_ref = 0x40;                         // need 5 volt reference
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

   f = freq/1000000;                  // lazy typing way
   divider = 126;
   if( f > 6 ) divider = 100;
   if( f > 9 ) divider = 66;
   if( f > 12 ) divider = 50;
   if( f > 18 ) divider = 34;
   if( f > 26 ) divider = 24;
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


/*****  Non-blocking  I2C  functions   ******/

// TWI interrupt version
// if twi interrupts enabled, then need a handler 
ISR(TWI_vect){
  i2poll();
  if( gi2state == 0 ) i2poll();   // needed to get out of state zero.
  #ifdef I2STATS
     ++i2ints;
  #endif
}


void i2init(){
  TWSR = 0;
  TWBR = 17;    //8  500k, 12 400k, 72 100k   for 16 meg clock. ((F_CPU/freq)-16)/2
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
     t = i2out;                     // !!! what about reads on i2c, needs work here i2rin changing if reading
     interrupts();
     if( rx_process_flag ) rx_process2();   // keep rx going rather than waste time, special case for this radio
     else delayMicroseconds( 40 );          // i2out should be changing if have interrupts happening 400k 
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
  // !!! change baud rate here for si5351 vs OLED ?  17 or 6
  // !!! may be better to just use the transmitting variable or both
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



/*********** end I2C functions  ************/


// simple menu with parallel arrays. Any values that do not fit in 0 to 255 will need to be scaled when used.
// example map( var,0,255,-128,127)
#define NUM_MENU 11
const char smenu[] PROGMEM = "Vol AttnBandModeKSpdKmdeSvolVox Cal PminPmax    ";      // pad spaces out to multiple of 4 fields
uint8_t *svar[NUM_MENU] = {&volume,&attn,&band,&mode,&kspeed,&kmode,&side_vol,&vox,&cal,&Pmin,&Pmax};
uint8_t  smax[NUM_MENU] = {  63,   3,    2,     2,     25,    3,     63,       1,   255, 50,   255 };
uint32_t stripee = 0b00000000000000000000011100100000;                         // bit set for strip menu items to save in eeprom

// 2,4 encoder, 0 reset entry, 1 select, 3 exit
void strip_menu( int8_t command ){        
static uint8_t sel;
static uint8_t ed;
static uint8_t mode;   // 0 not active, 1 select var, 2 edit var

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

 if( ed == 0 ){                                  // skip extra OLED writes
   LCD.gotoRowCol( 0, 0 );                             // print header row
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
      case 8:  qsy(0);  break;                     // cal implement freq change for user observation
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

void strip_restore(){          // restore items from eeprom.  If data is 0xff we ignore.
int i;
uint8_t val;

   for( i = 0; i < NUM_MENU; ++i ){
      if( stripee & ( 1 << i )){
         EEPROM.get( i, val );
         if( val != 0xff ) *svar[i] = val;
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

      // no prescale, arg clk controls what clock is on, WGM13-10 = 0101 fast 8 bit mode 5  ( usdx uses mode 14 )
      // OCR1AL controls audio,  OCR1BL controls transmit pwm.
   PRR &= ~(1 << PRTIM1 );
   OCR1AH = 0;                       // temp high byte register is saved for B low write also
   OCR1AL = 128;  OCR1BL = Pmin;  TCNT1L = 0;
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
  OCR2A = 51;                        // count to value, resets, interrupts, fs = 47786, !!! different value for TX
  OCR2B = OCR2A - 1;                 // use the 2nd interrupt for transmit
  TCCR2B = 2;                        // start clocks, divide by 8 clock
  interrupts();
}


ISR(TIMER2_COMPA_vect){               // Timer2 COMPA interrupt

   rx_process();  
}

ISR(TIMER2_COMPB_vect){               // Timer2 COMPB interrupt, change TIMSK2 to pick what process is running

   //tx_process();
}


void audio_out( int16_t val ){        // clip/saturate values to 8 bits
 
   OCR1AL = constrain( val + 128, 0, 255 );
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
   s >>= 6;           // divide by 64, max volume
   audio_out( s );
}


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
       if( missing == 1 ) sw_adc = c1 + 512, missing = 2;            // c1 is bogus at this point, fixed next time.

       // CIC filtering
       y2 = y2 + ( y1 = y1 + a1 );        // two cascaded integrators, input is a1
       a1 = b1; b1 = c1;                  // move the A/D delay line
       R = (R+1) & 3;                     // mod 4 counter
       if( R ) return;                    // decimate by 4, process on zero, skip 1 2 3
       
                                          // 1/4 rate processing
       y3 = y2 - y2d;   y2d = y2;         // two comb filters, length is decimation rate ( 4 )
       y4 = y3 - y3d;   y3d = y3;         // but only one delay term is needed as running at 1/4 rate

       y4 >>= 4;                          // shift out extra bits.  Did we gain one bit with the oversampling?   
      
   }
   else{             // process I
    
       //if( ADCSRA & ( 1 << ADSC ) ){      // still busy, use a faster adc clock
       //    if( lock == 0 ) --adps, lock = 5;
       //    --lock;
       //}
       
       d2 = ADC - 512;
       if( sw_adc == -1 ) ADMUX = ad_ref | 3 , missing = 1;     // steal an ADC read
       else ADMUX = ad_ref | 1;              // que next read for Q, merge ad_ref bits
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
           audio_out( rx_val[outrx++] );
           outrx &= (PIPE-1);
           --rx_ready_flag;
       }
       #ifdef I2STATS
         else ++rx_underruns;             // expect a couple starting up, then pipeline starts working
       #endif  

       if( rx_process_flag < (PIPE-1) ){
          Qr[inrx] = y4;  Ir[inrx++] = z4;
          inrx &= (PIPE-1);
          ++rx_process_flag;
       }
       #ifdef I2STATS
         else{
            ++rx_overruns;              // this would be bad, pipeline is full
            //--rx_underruns;             // an overrun may cause an underrun as data was not processed
         }
       #endif
         
   }
  
}

// some non-interrupt processing,  will this work from loop(), added a pipeline as it fell behind
// calc rx_val to be sent to PWM audio on the rx interrupt 
void rx_process2(){
int val;
static uint8_t rxout, rxin;     // local indexes rather than shared, hopefully will stay in sync with interrupt processing

   noInterrupts();
   val = Ir[rxout] >> 2;   // Output is 8 bits.  Qr not used, simple DC receiver.
                    // ?? or leave this at 10 bits and possible clip 2 ( 12db ) at the output PWM
   --rx_process_flag;
   interrupts();
   ++rxout;  rxout &= (PIPE-1);

   // need to be at <= 10 bits for this calc or use long type for val
   val *= volume;
   val >>= 6;

   noInterrupts();
   rx_val[rxin++] = val;
   ++rx_ready_flag;
   rxin &= (PIPE-1);
   interrupts();
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
          LCD.printNumI( f, CENTER, ROW7 );
          LCD.printNumI( e, RIGHT, ROW7 );
 }
#endif


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
