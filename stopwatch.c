/* 
 Clockit Stop Watch
 Clockit KIT-10930(https://www.sparkfun.com/products/10930)

 revision history:
 <stopwatch.c> Mark D. Shattuck 12/28/2013
 12/28/2013 Mark D. Shattuck <mds> timer.c
            Basic stopwatch using Sparkfun Clockit 4-digit 7-segment display
            000 mds Complete re-write of the Spark Fun 
                    03/04/09 [Nathan Seidle] 
                    02/24/11 [Jim Lindblom]
                    original alarm clock software
    
 
 Description:
 Convert Clockit into a stop watch. 
 NEW firmware code for the Sparkfun Clockit Alarm Clock Kit 
 Clockit KIT-10930(https://www.sparkfun.com/products/10930).
 
 Functions: 
 Display => Elapsed time in seconds with floating decimal
            Adjustable brightness. 
 Timer   => Reset, Start, Stop, Re-Start, [Lap]
 
 Operation:
 SNOOZE => RESET
 UP     => START
 DOWN   => STOP

 Hold UP + DOWN => RESTART
 Repeated SNOOZE increase/decrease [Alarm ON/OFF] brightness
 
 [Lap] NYI (Not Yet Implemented]
 
 Hardware:
 4-digit 7-segment display and ATmega328P micro-controller 
 
 Detailed Description:
 Basic stop watch using the Atmel 8-bit ATmega328P micro-controller and a common 
 anode 7-segment 4-digit LED panel.  Hardware is from SparkFun Clockit KIT-10930 
 (https://www.sparkfun.com/products/10930). The timebase is set by an external
 16MHz crystal oscillator.  Three push buttons (UP/DOWN/SNOOZE) and one switch 
 (ALARM ON/OFF) control operations.  Elaped time is indicated in seconds using
 a floating decimal point which moves to the right as the time increases. For 
 0.000 - 9.999 seconds 1/1000 of seconds are shown.  For 10.00 to 99.99 seconds
 hundredths are shown etc.  A piezo-electric buzzer is not used. The stopwatch
 operates in three modes RESET, RUN, STOP.  In RESET mode the elapsed time is 
 0.000s and the counter is not running. RESET is entered by pressing  SNOOZE 
 during STOP or RUN mode.  RUN is entered by pressing UP during RESET.  In RUN 
 mode the counter is counting and the elapsed time is displayed. In STOP mode
 the counter is not running and the elapsed time is displayed.  STOP is entered 
 by pressing DOWN during RUN mode.  In STOP mode holding UP and pressing DOWN 
 will restart the timer and change from STOP to RUN mode.  To change the 
 display brightness press SNOOZE during RESET mode.  The display will get
 brighter if the Alarm switch is ON and dimmer if the Alarm switch is OFF.

 Theory of Operation:
 1) All functions are implemented using interrupts. main() is just a loop.
 2) The 4 digit 7-segment display has 6 common anodes -- 4 digits, the colon,
    and the apostrophe (upper dot between the 3rd and 4th digit).  The colon and 
    apostrophe anodes should be connected together to save an IO pin on 
    the micro-controller.  However, this was not done in hardware so it is 
    implemented in software.  This leaves 5 anodes to update.  The display
    is controlled using the overflow (OVF) and compare A (COMPA) interrupts from
    the 8-bit COUNTER/TIMER2.  To get a nice looking display all 5 elements need 
    to be updated at greater than 60Hz.  One element will be displayed on each OVF
    interrupt.  Therefore we need a minimum of 5*60=300 interrupts per second.  
    The clock is running at 16MHz and a pre-scale of 128 for 125KHz.  The 8-bit 
    counter overflows every 256 clock cycles or at ~488Hz, which gives ~98 updates 
    per second per element.  This is a little high, but half that value is a little 
    low and at 16MHz the CPU can execute 32K cycles per interrupt. A value closer to
    300Hz could be achieved using CTC mode (described below) but seemed unnecessary.
    There are no limiting resistors on the LED display so a crude pulse-width-
    modulation (PWM) scheme is used to limit the total power delivered to the LEDs.  
    This also allows easy control over the brightness.  The PWM is achieved by using 
    the COMPA interrupt in conjunction with the OVF interrupt.  The element is 
    turned on when the count reached OVF and turned off when the count reaches
    the value stored in OCR2A (the trigger for COMPA).  If OCR2A is 100 (the default)
    then the LED will be on for a 100/256 of the interrupt cycle. By changed OCR2A
    the brightness can be adjusted.  In principle the a maximum should be set to 
    protect the LED from getting too much power, however for the blue Young Sun
    YSD-439AB4B-35 used in the kit purchase in Dec 2013 a MAX_BRIGHT of 250 could 
    be used without problem.  However above OCR2A of ~200 the display brightness 
    seemed to saturate and here MAX_BRIGHT is set to 200.  For other LEDs a lover
    value may be needed.  Using interrupts the display is updated in the "background"
    so a volatile global array digit_val[5] of 5 element is used to change the 
    display.  The TIMER2_OVF_vect ISR (interrupt service routine) cycles through 
    one element of digit_val on each interrupt.  digit_val[0-3] can be 0-9. All 
    other values give blank.  digit_val[4] is a 2-bit bitmap + 0xF0, with bit 
    0->colon and 1->apostrophe.  The decimal points are controlled separately with
    global bitmap dp=0bxxxx3210. The for low bits controlling digit 3-2-1-0. The
    display control is set up to be portable to other projects.  Key portions 
    include: ISR (TIMER2_OVF_vect), ISR (TIMER2_COMPA_vect), function clear_display(),
    global variables digit, digit_val, and dp, commented section of ioinit() and 
    the associated defines.
 3) The core timing for the elapsed time is handled by the 16-bit Timer1. In CTC 
    mode (WGM mode 12) the timer counts up to ICR1=250 and generates a capture 
    (TIMER1_CAPT_vect) interrupt.  The clock frequency is 16MHz.  The pre-scaler 
    is set to 64, so each count is 64/16=4us.  Therefore, 1ms/4us = 250 counts 
    each ms (mili-second). If the system is in state==RUN, then Timer1's ISR 
    updates a global volatile MSDIG=13 element array mils[MSDIG] otherwise it returns.  
    mils contains up to MSDIG digits of the elapsed time. mils[0] contains the 
    1/1000th place, mils[1] contains the 1/100th place up to the 10^(MSDIG-4) place. 
    mils[0] is incremented on each interrupt when state==RUN, then each element 
    is tested for overflow to 10.  If a digit is 10, then it is set to 0 and the 
    next digit is incremented and tested for overflow.
 4) The brightness and state of the system RESET, RUN, STOP is determined by interrupts 
    driven by the Alarm switch and the three push buttons SNOOZE, UP, DOWN.  Alarm, UP,
    and DOWN are all part of PCINT0 and the SNOOZE is on PCINT2.  The ISRs process the
    state changes:
    SNOOZE => RESET: (if state!=RESET, Set digit_val[0-4]=0, mils[0-MSDIG-1]=0, dp=0x01) 
    UP     => START: (if state==RESET, state=RUN)
    DOWN   => STOP:  (if state==RUN, state=STOP)
    Hold UP + DOWN => RESTART: (if state==STOP, state=RUN)
    Brightness: SNOOZE => (if state==RESET, increase/decrease [Alarm ON/OFF] brightness)
    Also while UP is pressed display colon and while DOWN is pressed display apostrophe.
 5) Function initio() sets up the IO pin and counter-states.
 
 Detailed Hardware:
 AVRmega328P with 7-segment 4-digit display [YSD-439AB4B-35]

               ---------------------------------------
               |            AVR ATmega328P           |   
               |-------------------------------------|
         RESET-| 1  RESET                     PC5 28 |-B
          DIG1-| 2  PD0                       PC4 27 |-G   
          DIG2-| 3  PD1                       PC3 26 |-A      
             D-| 4  PD2                       PC2 25 |-C+COL-C
         COL-A-| 5  PD3                       PC1 24 |-F+APOS-C
          DIG3-| 6  PD4                       PC0 23 |-E  
           VCC-| 7  VCC                      AGND 22 |-GND   
           GND-| 8  GND                      AREF 21 |-VCC   
   16.000MHz-1-| 9  XTAL1                    AVCC 20 |-VCC    
   16.000MHz-2-| 10 XTAL2        {PCINT5}[SCK]PB5 19 |-BTN1(UP)        
            DP-| 11 PD5         {PCINT4}[MISO]PB4 18 |-BTN2(DOWN)         
          DIG4-| 12 PD6                 [MOSI]PB3 17 |-APOS-A      
  (SNOOZE)BTN3-| 13 PD7{PCINT23}              PB2 16 |-BUZZ-2   
     (ALARM)S1-| 14 PB0{PCINT0}               PB1 15 |-BUZZ-1
               ---------------------------------------
         

               ----------------------
               |  4-digit 7-segment |   
               |    Common Anode    |   
               |--------------------|
           PD0-| 1  DIG1       B 16 |-PC5
           PD1-| 2  DIG2       G 15 |-PC4
           PD2-| 3  D          A 14 |-PC3   
           PD3-| 4  COL-A      C 13 |-PC2
           PC0-| 5  E      COL-C 12 |-PC2       
           PD4-| 6  DIG3       F 11 |-PC1   
           PD5-| 7  DP    APOS-A 10 |-PB3      
           PD6-| 8  DIG4  APOS-C  9 |-PC1
               ----------------------
         

               ----------------------
               | 6-PIN SPI Program  |   
               |--------------------|
        [MISO]-| 1  MISO      VCC 2 |-VCC
         [SCK]-| 3  SCK      MOSI 4 |-[MOSI]
         RESET-| 5  RESET     GND 6 |-GND   
               ----------------------
*/
               
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define FOSC 16000000 //16MHz crystal

// Common anodes
#define DIG_1   PORTD0
#define DIG_2   PORTD1
#define DIG_3   PORTD4
#define DIG_4   PORTD6
#define COL     PORTD3
#define APOS    PORTB3

// Cathodes
#define SEG_A   PORTC3
#define SEG_B   PORTC5
#define SEG_C   PORTC2
#define SEG_D   PORTD2
#define SEG_E   PORTC0
#define SEG_F   PORTC1

#define DP      PORTD5
#define COL_C   PORTC2
#define APOS_C  PORTC1

// Buttons
#define BUT_START   PORTB5
#define BUT_STOP    PORTB4
#define BUT_RESET   PORTD7
#define SW_ALARM    PORTB0

#define BUZZ1   PORTB1
#define BUZZ2   PORTB2

#define RESET 0
#define RUN 1
#define STOP 2

#define MAX_BRIGHT 200


// # of 1 ms clock digits
#define MSDIG 13

//Functions Definitions
void ioinit (void);
void clear_display(void);

//Global Variables
volatile uint8_t digit=0;  // currently displayed digit 0-3 or 4 for : and '
volatile uint8_t digit_val[5]={0,0,0,0,0xF0};  // currently displayed digit 0-3 or 4 for : and '
volatile uint8_t dp=0x01;  // currently displayed decimal point bitmap 0bxxxx3210

volatile uint8_t mils[MSDIG]; // 1/1000 of sec 7 digits mils[0]=1/1000,mils[1]=1/100 etc
volatile uint8_t decpos=0;    // position of decimal point

volatile uint8_t state=RESET;      // state of timer: RESET, RUN, STOP, SETUP
 
//Main
int main (void){
    ioinit(); //Initialize
    
    while(1){
    }
    
    return(0);
}

//Interrupt Service Routines ISR

// turn on 7-segment
ISR (TIMER2_OVF_vect){

    clear_display();
    //Set DIGIT/COLON+APOS (common anode)

    digit++;// Rotate digit each cycle
    if(digit>4){
     digit=0;
    }
    switch(digit) {
        case 0:
            PORTD |= _BV(DIG_1); // DIGIT1 anode on=1
            break;
        case 1:
            PORTD |= _BV(DIG_2); // DIGIT2 anode on=1
            break;
        case 2:
            PORTD |= _BV(DIG_3); // DIGIT3 anode on=1
            break;
        case 3:
            PORTD |= _BV(DIG_4); // DIGIT4 anode on=1
            break;
        case 4:
            PORTD |= _BV(COL); // COLON anode on=1 
            PORTB |= _BV(APOS);  // APOS anode on=1
        default: 
            break;
    }

    //Set number (cathode)
    
        switch(digit_val[digit]) {
        case 0:
            PORTC &= 0b11010000; //Segments ABCEF
            PORTD &= 0b11111011; //Segments D
            break;
        case 1:
            PORTC &= 0b11011011; //Segments BC
            break;
        case 2:
            PORTC &= 0b11000110; //Segments ABEG
            PORTD &= 0b11111011; //Segments D
            break;
        case 3:
            PORTC &= 0b11000011; //Segments ABCG
            PORTD &= 0b11111011; //Segments D
            break;
        case 4:
            PORTC &= 0b11001001; //Segments BCGF
            break;
        case 5:
            PORTC &= 0b11100001; //Segments ACFG
            PORTD &= 0b11111011; //Segments D
            break;
        case 6:
            PORTC &= 0b11100000; //Segments AFGCE
            PORTD &= 0b11111011; //Segments D
            break;
        case 7:
            PORTC &= 0b11010011; //Segments ABC
            break;
        case 8:
            PORTC &= 0b11000000; //Segments ABCEFG
            PORTD &= 0b11111011; //Segments D
            break;
        case 9:
            PORTC &= 0b11000001; //Segments ABCFG
            PORTD &= 0b11111011; //Segments D
            break;
        case 0b11110001:
            PORTC &= 0b11111011; //Segments [C]: COLON 
            break;
        case 0b11110010:
            PORTC &= 0b11111101; //Segments [F]: APOS
            break;
        case 0b11110011:
            PORTC &= 0b11111001; //Segments [CF]: COLON+APOS 
            break;
        default: 
            break;
    }
    // DP
    if(bit_is_set(dp,digit)) {
        PORTD &= 0b11011111; //Segments DP: DECIMAL POINT
    }
}

// turn off 7-segment 
ISR (TIMER2_COMPA_vect){

    clear_display();
}

// 1ms clock
ISR (TIMER1_CAPT_vect) {
    uint8_t n,C;
    
    if(state==RUN){
        C=1;
        mils[0]++;
        for(n=0;n<MSDIG-1 && C==1;n++){
            C=0;
            if(mils[n]==10){
                C=1;
                mils[n]=0;
                mils[n+1]++;
                if(n+1>decpos+3){ //floating decimal effect only tested to 9999s=2.7hr
                    decpos++;
                    if(decpos<4){
                        dp=_BV(decpos);
                    }else{
                        dp=0x00;
                    }
                }
            }
        }
        for(n=0;n<4;n++){
            digit_val[3-n]=mils[n+decpos];
        }
    }
        
}

ISR(PCINT0_vect){ // START, STOP, ALARM change state
    
    if(bit_is_set(PINB,BUT_START)){
        digit_val[4] &=~(_BV(0));
    }else{
        digit_val[4] |=_BV(0);
        if(state==RESET){
            state=RUN;
        }
    }

    if(bit_is_set(PINB,BUT_STOP)){
        digit_val[4] &=~(_BV(1));
    }else{
        digit_val[4] |=_BV(1);
        if(state==RUN){
            state=STOP;
        }
    }

    if(bit_is_clear(PINB,BUT_START) && bit_is_clear(PINB,BUT_STOP)){
        if(state==STOP){
            state=RUN;
        }
    }

}

ISR(PCINT2_vect){ // RESET button changed state
    uint8_t n;
    
    if(bit_is_clear(PIND,BUT_RESET)){
        if(state==RESET){
            if(bit_is_set(PINB,SW_ALARM)){
                if(OCR2A<MAX_BRIGHT-4){
                    OCR2A+=5;
                }
            }else{
                if(OCR2A>4){
                    OCR2A-=5;
                }
            }
        }           
        state=RESET;
        dp=0x01;
        decpos=0;
        for(n=0;n<MSDIG;n++){
            mils[n]=0;
        }
        for(n=0;n<4;n++){
            digit_val[n]=0;
        }
    }
}

//Functions
void clear_display(void) {
    PORTB &=~(_BV(APOS)); // APOS anode off=0
    PORTC = 0b00111111;  // Set BGACFE cathode off=1
    PORTD &= 0b10100100; // Set DIG4,DIG3,COL,DIG2,DIG1 anodes off=0 PD752=NC=1
    PORTD |= 0b00100100; // Set DP,D cathode off=1 PD764310=NC=0
}

void ioinit(void){

    // Setup IO; 1 = output, 0 = input 
    DDRB = 0b11111111 & ~(_BV(BUT_START)|_BV(BUT_STOP)|_BV(SW_ALARM));   
    DDRC = 0b11111111; // LED output
    DDRD = 0b11111111 & ~(_BV(BUT_RESET)); 
    

    PORTB = _BV(BUT_START)|_BV(BUT_STOP)|_BV(SW_ALARM); //Enable pull-ups
    PORTD = 0b10100100; //Enable pull-up on reset button
    PORTC = 0b00111111;

    //*** Display Code start
    sei(); //Enable interrupts
    //4-digit 7-segment Display timer0
    //Set pre-scalar to clk/128 @ 16MHz/128 = 125kHz => 8us per tick
    //4-digit + 1 for COLON/APOS = 5; refresh every 5*256*8us = 10.24ms ~=> 100Hz 
    TCCR2B = _BV(CS22)|_BV(CS20); 
    //TCNT2 generate interrupt every 2.048 ms (256 * 8us)
    //OCIE2A generate interrupt during each cycle 0-255. Use for brightness 0-255
    TIMSK2 = _BV(OCIE2A)|_BV(TOIE2);
    OCR2A = 100;  // set brightness 0-255
    //*** Display Code end
    
    //Init Timer1 for ms counting CTC mode
    //Set pre-scalar to clk/64 @ 16MHz/64 = 250kHz => 4us per tick
    TCCR1B |= _BV(CS10)|_BV(CS11); 
    TCCR1B |= _BV(WGM12)|_BV(WGM13);  // Mode 12 CTC mode
    TIMSK1 |= _BV(ICIE1);             //Enable interrupts
    ICR1 = 250;                       // SET TOP to 250*4us=1 ms

    PCICR |=_BV(PCIE0);                             //Set PCINT0-7
    PCMSK0 |=_BV(PCINT5)|_BV(PCINT4)|_BV(PCINT0);   //Activate 0,4,5 ALARM,STOP,START

    PCICR |=_BV(PCIE2);                             //Set PCINT16-23
    PCMSK2 |=_BV(PCINT23);   //Activate 23 RESET
    
}
