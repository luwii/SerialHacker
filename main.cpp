#include <Arduino.h>                            // platformIO needs this
#include <avr/pgmspace.h>                       // to put help store strings into flash
#include <utils.c>                              // various little helper functions

/* 
 SerialHacker (v0.1 2019-10-17) 
 - Based on Github repo for RS232enum  (v0.3.0 20110924) 
 
 This software will help identify serial communication activity on 
 an unknown circuit board.  Connect your 'Arduino' pins to the unknown board 
 and this sketch will try every possible pin combination
 and baud rate to determine RX/TX. It can also try to send wakeup
 characters to stimulate the line, such as a carriage return or other
 txpatterns that you add. This code sends data using a custom
 SoftwareSerial library. 
 
 For the Parallel scans the return transition counts would not be perfectly
 accurate, but entirely useable. Instead the software just gives a good indication 
 of active lines (parallel scan) and specifically serial lines that 
 respond to wakeup patterns / stimulation (active parallel and active
 per pin scans). Where reports indicate activity one could run the baud rate detection
 on active pins. 
 Use this info, plug up a serial cable and confirm with terminal software on
 your workstation using the indicated baud rate and wakeup patterns.

 Changes to RS232enum code
 1. When scanning, instead of storing returned data per pin, a simple transistion count is kept. 
    This free's up memory and allows additional features to be added. (And I don't think the actual data is needed)
 2. Baud rate estimation/detection routine was added.
 3. Menu's updated, and ability to choose pins and Tx baud rates added.
 4. You can also do a baud rate detection on the pins that show activity.
 5. Removed the define statements that save memory.
 6. Minor updates to compile on PlatformIO, and work with Arduino Due.
 7. Removed the Information screen as it was incompatible with the Due and did not needed anymore.

 SETUP:
 1. set hardware type to ensure tx is at the right speed for the Softserial routines.
 2. set selectable_pins[] to the arduino pins likely to be used for the scan
 3. set selectable_pinnames[] to the names for each pin (that better relate to
    your target. Could be the colour of the wire or a board pin number)
 4. set baudrates[] to a list of baudrates available for use when
    sending txpattern wakeup signals on the TX line.
 5. (optional) set txpatterns[] to desired stimulation patterns
 

 USAGE:
 -  load sketch and attach to arduino at 115200 baud to the PC.
 -  On PC, run a terminal emulator or open a window to interact with the interface

 RS232enum ORIGINAL AUTHOR & CODE BRANCHES: (Which formed the basis of this)
 http://github.com/cyphunk/RS232enum/ 
 Further documentation for the source project: http://deadhacker.com/tools/

 DEVELOPER NOTES:
 -  pfmt() = local printf()
 -  ppgm() = print() which uses the FLASH memory of the board. Some 
    boards have limited SRAM so we use the FLASH ram where possible.

    This code is public domain, abuse as you wish and at your own risk
 */

/*******************************************************************************************************
   BEGIN USER DEFINITIONS
 *******************************************************************************************************/
// DEFINE THE TARGET BOARD
// for the sendSerial to send at the correct speed. Options are: Teensy, Arduino, or Due
#define Due

// PINS AVAILABLE TO CHOOSE FROM AT RUNTIME
// Arduino must use microcontroller pin numbers for pins[]:
byte       selectable_pins[]     = {    23,    25,    27,    29,    31,    32,    33,    34,    35,    36,    37 };  // max of 64, to choose from at runtime
char      *selectable_pinnames[] = {  "p23", "p25", "p27", "p29", "p31", "p32", "p33", "p34", "p35", "p36", "p37" }; // max of 64 to choose from at runtime

// BAUDRATES AVAILABLE TO CHOOSE FROM AT RUNTIME
// uint32_t  selectable_baudrates[] = { 300, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600, 115200 };
uint32_t  selectable_baudrates[] = { 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200 };


// DEFINE TX WAKEUP PATTERNS (LSB first):
//uint16_t txpatterns[] = { 0x0000, 0xFFFF, 0x0a00, ~0x0a00 };         // 10=0xA=CR
//byte     txpatterns[] = { B11111111,B00000000,B01010101,B10101010 }; // 10=0xA=CR, 0x41="A"
//byte     txpatterns[] = { 0x0a, 0x0d, 0x41 };                      
byte       txpatterns[] = { 0x0a };                        

// DEFAULT LISTENING TIME (in us):
uint32_t  usperrx       = 4000000;

/*******************************************************************************************************
   END OF USER DEFINITIONS
 *******************************************************************************************************/


const byte   numberof_selectable_pins = sizeof(selectable_pins); // the number of entries in selectable_pins[] array. Supports up to 64, menu allows
                                                                 // first 10 to be toggled / selectable 
const byte   numberof_selectable_bauds = sizeof(selectable_baudrates)/sizeof(uint32_t);                                                              
byte         pins[numberof_selectable_pins];                    // selected pins numbers
char        *pinnames[numberof_selectable_pins];                // selected pin names


boolean      printnames          = true;                        // false reduces output to console 
byte         pinslen             = numberof_selectable_pins;    // # of selected pins. (populated in setup)
uint32_t     baudrates[numberof_selectable_bauds];
byte         baudrateslen  = numberof_selectable_bauds;
byte         txpatternslen = sizeof(txpatterns);

byte         pinstatelen   = ((pinslen-1) / 8) + 1; // bitfield len. calc backwards
byte        *pinstatebuf;                           // bitfield. alloc later, bit array

boolean selected_pins[numberof_selectable_pins];    // stores if pin is selected or not
boolean selected_bauds[numberof_selectable_bauds];  // stores if selected or not
volatile boolean interrupted=false;                 // used for interrupt on baud rate detection


// DECLARE (FROM UTILS)
void         pfmt(char *fmt, ... );
void         ppgm(const char *str);
boolean      noprint    = 0; 
uint32_t     timefunction(void (*function)(uint32_t), uint32_t arg);
uint32_t     timefunction(void (*function)(byte), byte arg);
uint32_t     timefunction(int (*function)());

void         determine_baudrate(byte pinindex, boolean quiet); // from main
void         usage();

/* initialize pins to read or write depending on test */
void setup_pins (byte outpin_i=0) 
{
     for (int i=0; i<pinslen; i++) 
     {
          pinMode(pins[i], INPUT);
          digitalWrite(pins[i], HIGH); // trigger intern pull-up resistor
          pinstatebuf[i/8] = 0xFF;     // init the buf
     }
}

// *********************************** Softserial (for tx) ****************************
uint32_t bitDelay;
uint32_t halfBitDelay;
uint32_t _baudRate;
uint32_t _bitPeriod; 
void softserial_setup (uint32_t baud) 
{
        _bitPeriod   = 1000000 / baud;
        _baudRate    = baud;
        // to be honest the following calculation may have serious issues. I have 
        // tested this with the Teensy++ at both 16MHZ and 8MHZ at all of the 
        // supported baudrates.
        #ifdef Teensy
          bitDelay     = _bitPeriod - clockCyclesToMicroseconds(12); // aprox time of digitalWrite?
        #endif
        // 16Mhz (Atmel) Ardunio / Arduino Pro mini seem to work on this:
        #ifdef Arduino
          bitDelay     = _bitPeriod - clockCyclesToMicroseconds(50);
        #endif 
        // Arduino Due (ARM processor)
        #ifdef Due
          bitDelay     = _bitPeriod - 2;  //don't ask, it works for me
        #endif

        halfBitDelay = bitDelay / 2;
}
void softserial_write(uint8_t _transmitPin, uint8_t b)      // write serial data using a pin
{
        if (_baudRate == 0)
                return;
 
        byte mask;
        digitalWrite(_transmitPin, LOW);
        delayMicroseconds(bitDelay);

        for (mask = 0x01; mask; mask <<= 1) 
        {
                if (b & mask)
                { // choose bit
                  digitalWrite(_transmitPin,HIGH); // send 1
                }
                else
                {
                  digitalWrite(_transmitPin,LOW); // send 1
                }
                delayMicroseconds(bitDelay);
        }

        digitalWrite(_transmitPin, HIGH);
        delayMicroseconds(bitDelay);
}

// *********************************** end Softserial *****************************

// *********************** Interrupt for Baud rate detection ****************************
#ifdef Due
void TC3_Handler()
{
        TC_GetStatus(TC1, 0);
        pfmt("\r\nNo reading ..\r\n");
        interrupted=true;
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)irq);
        TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
        uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
        rc = rc *5;  // slow this down by factor of 5 (frequency =1 => .2 Hz)
        TC_SetRA(tc, channel, rc/2); //50% high, 50% low
        TC_SetRC(tc, channel, rc);
        TC_Start(tc, channel);
        tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
        tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
        NVIC_EnableIRQ(irq);
}
#endif
// *********************************** end Interrupt *******************************

// *************************************  Scans  ***********************************
/* passive_parallel_scan
 * Will count transistions on selected pins to provide an idea of activity levels
 */

void passive_parallel_scan (uint32_t usperrx)
{   
    byte state;
    byte               *pinstatechangedbuf;                   // which pins changed
    unsigned int       *pinchangecountbuf;                    // count of changes to each pin
    pinstatebuf        = (byte *) malloc( pinstatelen );      // #bytes to store one set of pins read, in bits
    pinchangecountbuf  = (unsigned int  *) malloc( pinslen ); // stores count of changes to each pin
        

    for (byte i = 0; i < pinslen; i++)              // clear arrays, pins to input
    {  
        pinMode(pins[i],INPUT);
        bitWrite(pinstatebuf[i/8], i%8, 1);
        pinchangecountbuf[i] = 0;
    }

    pfmt( "\r\n\r\n" );
    ppgm(PSTR("Passive parallel scan\r\n"));
    ppgm(PSTR("-----------------------\r\n"));
    pfmt(     "Listens to all pins in parallel for %u milliseconds\r\n",ustoms(usperrx));
    ppgm(PSTR("Displays number of detected transitions per pin.\r\n"));

    // Read the initial pin state for baseline
    for (byte rx = 0; rx < pinslen; rx++) 
    {
       state = digitalRead(pins[rx]);
       bitWrite(pinstatebuf[rx/8], rx%8, state);
    }
   
    // Read the pins & count the changes
    microsbegin();
    uint32_t usstop = micros()  + usperrx;      //Time to sample
    while (micros() <= usstop) 
    {
       for (byte rx = 0; rx < pinslen; rx++) 
       {                     
           state = digitalRead(pins[rx]);    
           if (state != bitRead(pinstatebuf[rx/8], rx%8) ) 
           {                       
              bitWrite(pinstatebuf[rx/8], rx%8, state);  
              pinchangecountbuf[rx] = pinchangecountbuf[rx] +1;
           }
        }
    }

    // Show the results
    pfmt( "\r\nPin,Transition count\r\n" );
    delay(1);
    for (byte i = 0; i < pinslen; i++)                   
    { 
        pfmt( " %s ",pinnames[i] );
        pfmt( "%o\r\n",pinchangecountbuf[i] );
    }

    // Menu loop
    pfmt( "\r\n<d> Determine baud on active pins\r\n<x> to Exit" );
    pfmt("\r\n");
    char command[2];
    while( 1 )
    {
      if (Serial.available())
      {
         // Read menu option
         delay(5); // hoping read buffer is idle after 5 ms
         int i = 0;
         while (Serial.available() && i < 2) 
               command[i++] = Serial.read();
         
         Serial.flush();
         command[i] = '\0';          // terminate string
         //pfmt(command); // echo back

         // Execute command
         if(strcmp(command, "d") == 0)  
         { 
            for (byte i = 0; i < pinslen; i++)                   
            { 
            if (pinchangecountbuf[i] > 0 ) 
               {  pfmt( "\r\n" );
                  determine_baudrate(i, true);
               } 
            }
            pfmt("\r\n\r\n<return>");
            break; 
         }
         else { usage(); break;}
      }  // if serial available
    } //while 1
    free(pinchangecountbuf);
}


/* Active_parallel_scan
 * Will send a txpattern wakeup pattern hoping this will stimulate the target to 
 * start printing to the console (it is common that some terminals require you 
 * send a carriage return before they present a login screen)
 * all non-tx pins are then scanned in parallel
 */
void active_parallel_scan (uint32_t usperrx) 
{   byte state;
    byte               *pinstatechangedbuf;                  // which pins changed
    unsigned int       *pinchangecountbuf;                   // count of changes to each pin
    pinstatebuf        = (byte *) malloc( pinstatelen );     // #bytes to store one set of pins read, in bits
    pinchangecountbuf  = (unsigned int  *) malloc( pinslen); // stores count of changes to each pin
                               
    for (byte i = 0; i < pinslen; i++)                       // clear arrays, pins to input
    {  
         pinMode(pins[i],INPUT);                             // to read initial pin states
         bitWrite(pinstatebuf[i/8], i%8, 1);
         pinchangecountbuf[i] = 0;
    }
    pfmt("\r\n\r\n");
    ppgm(PSTR("Active Parallel Scan\r\n"));
    ppgm(PSTR("-----------------------\r\n"));
    ppgm(PSTR("Sends wakeup stimulant to tx pin at listed baudrates\r\n"));
    pfmt(     "Listens to all non tx pins in parallel for %u milliseconds\r\n",ustoms(usperrx));
    ppgm(PSTR("Sending at: "));
    for (int i=0; i<baudrateslen; i++) 
         pfmt("%lu ",baudrates[i]);
    pfmt("\r\n\r\n");
    
    // Read the initial pin state for baseline
    for (byte rx = 0; rx < pinslen; rx++) 
    {
       state = digitalRead(pins[rx]);
       bitWrite(pinstatebuf[rx/8], rx%8, state);
    }

        microsbegin();
        uint32_t usstop;
        uint16_t baud_i;
        byte pat, tx, rx;

        for (baud_i = 0; baud_i < baudrateslen; baud_i++) 
        {
                pfmt("baud: %lu\r\n",baudrates[baud_i]);
                softserial_setup(baudrates[baud_i]);
                for (pat = 0; pat < txpatternslen; pat++) 
                {
                        for (tx = 0; tx < pinslen; tx++) 
                        {
                                pfmt(" 0x%02X -> %s\r\n",txpatterns[pat],pinnames[tx]);
                                
                                pinMode(pins[tx], OUTPUT);
                                softserial_write(pins[tx], txpatterns[pat]);
                                usstop = micros() + usperrx;
                                while (micros() <= usstop)
                                {
                                        for (rx = 0; rx < pinslen; rx++) 
                                        {
                                                if(rx == tx) continue;
                                                state = digitalRead(pins[rx]);
                                                if (state != bitRead(pinstatebuf[rx/8], rx%8) ) 
                                                {
                                                    bitWrite(pinstatebuf[rx/8], rx%8, state);  
                                                    pinchangecountbuf[rx] = pinchangecountbuf[rx] +1;
                                                }
                                        }
                                }
                                pinMode(pins[tx], INPUT); // reset    
                                digitalWrite(pins[tx], HIGH); // pull-up resistor
                        }
                }
        }
    pfmt("\r\nElapsed time: ");
    ustotime(microselapsed());

    // Show the results
    ppgm(PSTR("\r\nPrints pins whose bits change during test"));
    pfmt( "\r\nPin,Transition count\r\n" );
    for (byte i = 0; i < pinslen; i++)
    {   if( pinchangecountbuf[i] > 8 )   // filter out the low values, need at least 8 transitions
        {
          pfmt( "%s ",pinnames[i] );
          pfmt( "%o\r\n",pinchangecountbuf[i] );
        }
    }

     // Menu loop
    pfmt( "\r\n<d> Determine baud on active pins\r\n<x> to Exit" );
    pfmt("\r\n");
    char command[2];
    while( 1 )
    {
      if (Serial.available())
      {
         // Read menu option
         delay(5); // hoping read buffer is idle after 5 ms
         int i = 0;
         while (Serial.available() && i < 2) 
               command[i++] = Serial.read();
         
         Serial.flush();
         command[i] = '\0';          // terminate string

         // Execute command
         if(strcmp(command, "d") == 0)  
         { 
            for (byte i = 0; i < pinslen; i++)                   
            { 
            if (pinchangecountbuf[i] > 0 ) 
               {  pfmt( "\r\n" );
                  determine_baudrate(i, true);
               } 
            }
            pfmt("\r\n\r\n<return>");
            break; 
         }
         else { usage(); break;}
      }  // if serial available
    } //while 1

    free(pinchangecountbuf);
}



/* Active_per_pin_scan
 * The same as active_parallel_scan except that each rx pin is scanned individually
 */
void active_per_pin_scan (uint32_t usperrx) 
{
    byte               *pinstatechangedbuf;                  // which pins changed
    unsigned int       *pinchangecountbuf;                   // count of changes to each pin
    pinstatebuf        = (byte *) malloc( pinstatelen );     // # bytes to store one set of pins read, in bits
    pinchangecountbuf  = (unsigned int  *) malloc( pinslen); // stores count of changes to each pin
   
    ppgm(PSTR("Active_per_pin_scan\r\n"));
    ppgm(PSTR("-------------------\r\n"));
    pfmt(     "Listens to 1 rx pin at a time for %u milliseconds\r\n",ustoms(usperrx));
    ppgm(PSTR("Sends wakeup stimulant to tx pin at defined baudrates\r\n"));
    ppgm(PSTR("If activity was detected on a pin, they will be listed\r\n"));
    ppgm(PSTR("Sending at: "));
    for (int i=0; i<baudrateslen; i++) 
         pfmt("%lu ",baudrates[i]);
    pfmt("\r\n\r\n");

    uint32_t usstop;        
    uint16_t baud_i;
    byte pat,tx,rx,state;

    // Skipping the read the initial pin state for baseline, don't think it's needed.
    // This scan can take a while, the values can just change, and results are estimates

    for (byte i = 0; i < pinslen; i++)                       // clear arrays, pins to input
         pinchangecountbuf[i] = 0;
         
    microsbegin();
    for (pat=0; pat < txpatternslen; pat++) 
    {
         for (tx=0; tx < pinslen; tx++) 
         {
              pinMode(pins[tx], OUTPUT);
              // no need to delay before read. just need to be sure we can 
              // poll all pins long enough to detect low serial baud.
              for (rx=0; rx < pinslen; rx++) 
              {
                   if(rx==tx) continue; // no need to digitalRead(rxpin)
                      for (baud_i=0; baud_i < baudrateslen; baud_i++) 
                      {
                           pfmt(" tx:%s baud:%lu;  rx:%s \r\n",pinnames[tx],baudrates[baud_i],pinnames[rx]);
                           softserial_setup(baudrates[baud_i]);
                           softserial_write(pins[tx], txpatterns[pat]);
                           usstop = usperrx+micros();
                           while (micros() <= usstop )
                           {
                               state = digitalRead(pins[rx]); 
                               if (state != bitRead(pinstatebuf[rx/8], rx%8) ) 
                               {
                                   bitWrite(pinstatebuf[rx/8], rx%8, state);  
                                   pinchangecountbuf[rx] = pinchangecountbuf[rx] +1;
                               }
                            }
                       }
                       // Show the results

                       boolean found = 1;
                       for (byte i = 0; i < pinslen; i++)
                       {    if (pinchangecountbuf[i] > 8)  // Only print if more than 8 transitions
                            {   if(found) 
                                { 
                                   pfmt( "\r\nPin,Transition count\r\n" );
                                   found = 1;
                                }
                                pfmt( " %s ",pinnames[i] );
                                pfmt( " %o \r\n",pinchangecountbuf[i] );
                                pinchangecountbuf[i]=0;
                            }
                        }
                        pfmt( "\r\n");
                }
                pinMode(pins[tx], INPUT); // reset to INPUT   
                digitalWrite(pins[tx], HIGH); //pull-up
          }
    }

     // Menu loop
    pfmt( "\r\n<d> Determine baud on active pins\r\n<x> to Exit" );
    pfmt("\r\n");
    char command[2];
    while( 1 )
    {
      if (Serial.available())
      {
         // Read menu option
         delay(5); // hoping read buffer is idle after 5 ms
         int i = 0;
         while (Serial.available() && i < 2) 
               command[i++] = Serial.read();
         
         Serial.flush();
         command[i] = '\0';          // terminate string
         //pfmt(command); // echo back

         // Execute command
         if(strcmp(command, "d") == 0)  
         { 
            for (byte i = 0; i < pinslen; i++)                   
            { 
            if (pinchangecountbuf[i] > 0 ) 
               {  pfmt( "\r\n" );
                  determine_baudrate(i, true);
               } 
            }
            pfmt("\r\n\r\n<return>");
            break; 
         }
         else { usage(); break;}
      }  // if serial available
    } //while 1

  free(pinchangecountbuf);
}

/* pins_state
 * prints the current high/low state of all the selected pins
 * serial should be high when doing nothing
 */
void pins_state () 
{
    // align names and pin state columns by finding the max string length
    pfmt("\r\nPins state\r\n");
    pfmt("----------\r\n\r\n");
    ppgm(PSTR("pin:      "));
    for (byte i = 0; i < pinslen; i++) 
    { 
        pfmt("%s  ", pinnames[i]);
    }
    ppgm(PSTR("\r\nstatebuf: "));
    for (byte i = 0; i < pinslen; i++) 
    {
       // align columns using space:
       for (byte j = 0; j < strlen(pinnames[i])-1; j++)
            pfmt(" ");
       pfmt("%d  ", bitRead(pinstatebuf[i/8], i%8));
    }
    ppgm(PSTR("\r\ncurrent:  "));
    for (int i = 0; i < pinslen; i++) 
    {
        for (byte j = 0; j < strlen(pinnames[i])-1; j++)
             pfmt(" ");
        pfmt("%d  ", digitalRead(pins[i]));
    }
    pfmt( "\r\n\r\n<return> " );
    pfmt("\r\n");
}


// *********************************** end _scans *****************************



void set_rx_wait()
{
     String inputValue;
     char value[32];
     pfmt("\r\n\r\nRX wait\r\n");
     pfmt("-------\r\n\r\n");
     pfmt("Enter RX poll time in milliseconds [enter to keep %dms]: ", ustoms(usperrx));
     // WAIT FOR VALUE
     while (!Serial.available()) { }

     inputValue = Serial.readString(); 
     inputValue.toCharArray(value, 31);
     if (atoi(value))
         usperrx = mstous(atoi(value));
     pfmt("\r\nSet to %dms\r\n\r\n<return>", ustoms(usperrx));      
}


void determine_baudrate(byte pinindex, boolean quiet)                           
      // the results of this is not exact, it assumes resonably random input data,
      // and gives a good idea what the baud rate might be.
      // it needs n bit transitions to work, otherwise it can hang if target board #define Due not set.(user variables at the top)
      // (Due uses an ARM timer interupt and I just did not add an Atmel compatile one) 
      // 230400 is untested, dont have a good data source 
      // A 16Mhz Atmel Arduino works OK up to 57600. 
      // Arduino Due works fine up to 115200, maybe more

{     
      const byte n=20;                                  // number of samples to take
      byte pin=pins[pinindex];                          // pin to use for reading to determine baudrate
      byte j=n;
      byte newn;                                        // temp variable 
      byte state,previousState = 0;                     // used in baudrate detection,state change triggers for timing
                                                    
      unsigned long  timeStart,timeEnd,timeDiff;        // measurements 
      unsigned long  measuredResults[n]    = { };
      unsigned long  temp;
      unsigned long  baudrates[]          = { 1200,2400,4800,9600,14400,19200,28800,38400,57600,115200,230400 };
      byte           baudrateconfidence[] = { 0,0,0,0,0,0,0,0,0,0,0 };
     
      // On Screen message
      if( !quiet)
      { 
         ppgm(PSTR("\r\nDetermining baud rate\r\n"));
         ppgm(PSTR("---------------------\r\n"));

         #ifdef Arduino           
           ppgm(PSTR("This only works up to 57600 on 16MHz Atmel 328\r\n"));
         #endif
         ppgm(PSTR( "Highest score is most likely.\r\n"));
         #ifndef Due 
           ppgm(PSTR( "If stuck here, there were not enough state transitions on the pin.\r\n" )); 
         #endif
              
      }
      pfmt("Reading pin: %s\r\n\r\n", pinnames[pinindex]);
      pinMode(pin,INPUT);

      // Clear the buffer  
      for( int i=0; i<n; i++) 
      { 
          measuredResults[i]=0; 
      }

      // RS232 waits high. Wait for first zero or a random down transition
      state = digitalRead(pin);
      while( state==1) state = digitalRead(pin);
        
      // Measure n transition times
      previousState = 0;
      timeStart = micros();   // 4us accuracy on 16HZ Atmega328
      #ifdef Due
        startTimer(TC1, 0, TC3_IRQn,1); //TC1 channel 0, the IRQ for that channel the desired frequency (# times per 5 seconds)
      #endif

      for( int i=0; i<n; i++)
         {        
           while( state==previousState && interrupted==false) state = digitalRead(pin);      //Can be improved with direct pin read, but hardware specific
           timeEnd = micros(); 
           previousState = state;
           timeDiff = timeEnd - timeStart;
           measuredResults[i] = timeDiff;
           timeStart = timeEnd;
          }
      #ifdef Due
         TC_Stop(TC1,0);
      #endif
      // Print measurements for debugging
      // for( int i=0; i <n; i++) { Serial.print( i ); Serial.print( " " ); Serial.println(measuredResults[i] ); }

      // Sort the measurements to find shortest (which should be the bit duration)
      // Lowest # not accurate, need to take the lowest repeating #
      
      while( j > 1 )
           {
             newn=0;
             for (int i = 1; i<=(j-1); i++)
                 {    
                   if (measuredResults[i-1] > measuredResults[i])
                      {  
                        temp = measuredResults[i-1];
                        measuredResults[i-1] = measuredResults[i];
                        measuredResults[i] = temp;                 
                        newn = i;      
                      }  
                 } // for
             j = newn;
           } // while

      //pfmt("\r\nSorted measured values...\r\n\r\n");
      //for( int i=0; i<n; i++) { Serial.println(measuredResults[i]); }
      
      j = round(n/2);        // only looking at the fastest 50% of transitions

      // Count each baud rate bucket for the fastest transistions
      for( int i=0; i<j; i++) 
         {  
            if ( measuredResults[i] > 750  && ( measuredResults[i] <916 )) baudrateconfidence[0]++; //1200
            if ( measuredResults[i] > 375  && ( measuredResults[i] <457 )) baudrateconfidence[1]++; //2400
            if ( measuredResults[i] > 187  && ( measuredResults[i] <229 )) baudrateconfidence[2]++; //4800
            if ( measuredResults[i] > 93   && ( measuredResults[i] <114 )) baudrateconfidence[3]++; //9600
            if ( measuredResults[i] > 62   && ( measuredResults[i] <75  )) baudrateconfidence[4]++; //14400
            if ( measuredResults[i] > 46   && ( measuredResults[i] <58  )) baudrateconfidence[5]++; //19200
            if ( measuredResults[i] > 30   && ( measuredResults[i] <37  )) baudrateconfidence[6]++; //28800
            if ( measuredResults[i] > 23   && ( measuredResults[i] <29  )) baudrateconfidence[7]++; //38400
            if ( measuredResults[i] >=15   && ( measuredResults[i] <=21 )) baudrateconfidence[8]++; //57600
            if ( measuredResults[i] >= 7   && ( measuredResults[i] <=9  )) baudrateconfidence[9]++; //115200
            if ( measuredResults[i] >= 3   && ( measuredResults[i] <=5  )) baudrateconfidence[10]++; //234000
         }
         
      pfmt("Score - Baudrate\n\r");
      for( int i=0; i<11; i++)
         { if(baudrateconfidence[i]>0) 
             {
               Serial.print(baudrateconfidence[i]); 
               Serial.print("   -  "); 
               Serial.println(baudrates[i]); 
             }
         }
      if( !quiet ) pfmt("\r\n<return>");      
}

void help() 
{
        ppgm(PSTR("\r\n\r\n\r\n\r\n\r\n\r\nHelp\r\n"
                "-----\r\n"
                "\r\n"
                
                "[1] Passive Parallel:  (Issues at high speed)\r\n    -Poll all RX pins in parallel\r\n    -No active TX\r\n\r\n"
                "[2] Active Parallel:   (Issues at high speed)\r\n    -Poll all RX pins in parallel\r\n    -One TX pins used to print patterns\r\n\r\n"
                "[3] Active Per Pin:\r\n    -Poll RX pins incrementally (less error prone)\r\n    -One TX pins used to print patterns\r\n\r\n"
                //"[4] All:\r\n    -Runs tests [1-3] in sequence\r\n"
                "[5] Pin state        Display current value on pins\r\n"
                "[6] Rx wait          Set test wait time when polling rx lines\r\n"
                "[7] Set Pins         Set the pins to use for scanning\r\n"
                "[8] Determine baud \r\n    -Read a small number of samples, assuming random data\r\n    -Uses shortest bit intervals to estimate a baud rate\r\n"
                "[9] Select Tx baudrates    Set the pins used for TX\r\n"
                 "\r\n\r\n<return>"));
}
void usage() 
{
        ppgm(PSTR("\r\n\r\n\r\n\r\n\r\n\r\nSerialHacker\r\n--------\r\n"));
        pfmt("Rx wait : %d ms \r\n", ustoms(usperrx));
        ppgm(PSTR("TX baud rates: "));
        for (int i=0; i<baudrateslen; i++) 
             pfmt("%lu ",baudrates[i]);

        pfmt("\r\n");

        pfmt("Pins:");
        for (int i=0; i<pinslen; i++) 
             pfmt(" %s ",pinnames[i]);

        ppgm(PSTR("\r\n\r\n"
                "\r\n"
                " [1] Passive parallel \r\n"
                " [2] Active parallel  \r\n"
                " [3] Active per pin   \r\n"
                //" [4] All (1-3)        \r\n\r\n"
                " [5] Pin state        \r\n"
                " [6] Rx wait          \r\n"
                " [7] Select pins      \r\n"
                " [8] Determine baud   \r\n"
                " [9] Select Tx baudrates\r\n"
              //" [t] Tx speed test \r\n"          //commented out as its an endless loop
                " [?] Help   \r\n\r\n"
                "> "));

}


void test_tx( void )   // use to test Softserial bitperiod calculations by comparing with known good
{                      // on oscciloscope. Sends 10101010 forever, hence not on the menu.
  pfmt("\r\nSending 57.6kbps 1010 signal forever on first selected pin: %s...",pinnames[0]);   
  pinMode(pins[0],OUTPUT);
  softserial_setup(57600);                            
  while (1)            // making this a finite loop could affect the speed 
        { softserial_write(pins[0], B10101010); }
}



void select_pins( void )   // allows the first 10 pins for the selectable list to be toggled for use
                           // if there are more pins on the list, they just stay there
{ 
  const   byte asciioffset = 97; // set to 48 for numbered menu, 97 for letter menu
  int     counter=pinslen;       // count of selected pins after each button press
  char    command[2];            // keyboard input
  int     i;                     // loops
  int     rp =0;                 // repeat, only read a valid number 
  int     temp;
   
  pfmt("\r\n\r\nSelect pins to use:\r\n");
  pfmt("--------------------\r\n");

  // Heading
  pfmt("Current selection: ");
  for(i=0; i<pinslen; i++)  
      pfmt( "%s ",pinnames[i]);    

  pfmt("\r\n");
    
  // Option list
  pfmt("\r\nToggle on/off:\r\n");
  i=0;
  while( i<numberof_selectable_pins && i<22)  //25 letters in the alphabet + skip x onwards to exit menu
  {  temp = i + asciioffset;
     pfmt( "%c) %s\r\n", char(temp),selectable_pinnames[i]); 
     i++;
  }
  pfmt("\r\nx to Exit\r\n> ");
  
  // Loop menu till user is done
  while ( strcmp(command, "x")!=0 )
  { 
     command[0]='.';  //any non-trigger value

     // Read command
     delay(5); // hoping read buffer is idle after 5 ms
     i = 0;
     while (Serial.available() && i < 1 )
            command[i++] = Serial.read(); 
      
      Serial.flush();
      command[1] = 0; 
   
     // Compare keystroke with menu option 
     char temp;
     for( i=0; i<numberof_selectable_pins; i++)
     {    temp = i + asciioffset;   
          char mystring[] = {(char)temp, '\0'};
          if(strcmp(command, mystring) == 0 && numberof_selectable_pins>=2) 
          { 
             if( counter > 1 || selected_pins[i]==0 ) 
                selected_pins[i] = !selected_pins[i]; 
             rp=1; 
             pfmt("%c",mystring[0]); 
          } 
     }
     if(strcmp(command, "x") == 0 ) { usage(); break; } 
     
    
     if( rp==1)   // if something logical was pressed
     { 
        pfmt("\r\n");
        //Build new pin selection strings
        counter=0;
        for( i=0; i<numberof_selectable_pins; i++ )
        {   
           if( selected_pins[i] )
           {
              pins[counter] = selectable_pins[i];
              pinnames[counter] = selectable_pinnames[i];
              counter++;
           }
        }  //for

       pinslen = counter; 
  
       //Heading
       pfmt("\r\nSelect pins to use:\r\n");
       pfmt("--------------------\r\n");
       pfmt("Current selection: ");
       for(i = 0; i<pinslen; i++)  { pfmt( "%s ",pinnames[i]);     }   
           pfmt("\r\n");
  
       // Option List
       pfmt("\r\nToggle on/off:\r\n");
       i=0;
       while( i<numberof_selectable_pins && i<22)  //25 letter in the alphabet + x to exit menu
       {  temp = i + asciioffset;
          pfmt( "%c) %s\r\n", char(temp),selectable_pinnames[i]); 
          i++;
       }
      pfmt("\r\nx to Exit\r\n> ");

       rp=0;

    } // changed the pin selection
   } // while - loop menu till done
}



void select_bauds( void )   // makes the first 10 baud rates selectable 
                            // if there are more pins on the list, they just stay there
{ 
  const   byte asciioffset = 97; // set to 48 for numbered menu, 97 for letter menu
  int     counter=baudrateslen;  // count # of selected bauds after each button press
  char    command[2];            // keyboard input
  int     i;                     // loops
  int     rp =0;                 // repeat, only read a valid number 


  // Heading
  pfmt("\r\n\r\nSelect baud rates to use:\r\n");
  pfmt("-----------------------\r\n");

  pfmt("Current selection: ");
  for(i=0; i<baudrateslen; i++)  
      pfmt( "%d ",baudrates[i]);    

  pfmt("\r\n");


  // Option list - New
  pfmt("\r\nToggle on/off:\r\n");
  i=0;
  byte temp;
  while( i<numberof_selectable_bauds && i<22)  //25 letter in the alphabet, skip x onwards to exit menu
  {  temp = i + asciioffset;
     pfmt( "%c) %d\r\n", char(temp),selectable_baudrates[i]); 
     i++;
  }
  pfmt("\r\nx to Exit\r\n> ");

  
  // Loop menu till user is done
  while ( strcmp(command, "x")!=0 )
  { 
     command[0]='.';  //any non-trigger value  

     // Read command
     delay(5); // hoping read buffer is idle after 5 ms
     i = 0;
     while (Serial.available() && i < 1 )
            command[i++] = Serial.read(); 
      
      Serial.flush();
      command[1] = '\0'; 

     // Compare keystroke with menu option 
     char temp;
     for( i=0; i<numberof_selectable_bauds; i++)
     {    temp = i + asciioffset;   
          char mystring[] = {(char)temp, '\0'};
          //pfmt("\r\n command=%s, compare=%s",command,mystring);
          
          if(strcmp(command, mystring) == 0 && numberof_selectable_bauds>=2) 
          { pfmt("inside toggle %d %d", counter, selected_bauds[i]);   
             if( counter > 1 || selected_bauds[i]==0 ) 
             selected_bauds[i] = !selected_bauds[i]; 
             rp=1; 
             pfmt("%c",mystring[0]); 
          } 
     }
     if(strcmp(command, "x") == 0 ) { usage(); break; } 
  
     // If a valid menu option was selected
     if( rp==1)  
     { 
        pfmt("\r\n");
        //Build new baudrate selection strings
        counter=0;
        for( i=0; i<numberof_selectable_bauds; i++ )
        {   
           if( selected_bauds[i] )
           {
              baudrates[counter] = selectable_baudrates[i];
              counter++;
           }
        }  //for

       baudrateslen = counter; 

       // Heading
       pfmt("\r\n\r\nSelect baud rates to use:\r\n");
       pfmt("-----------------------\r\n");
       pfmt("Current selection: ");
       for(i=0; i<baudrateslen; i++)  
           pfmt( "%d ",baudrates[i]);    
       pfmt("\r\n");
    
       // Option list
       pfmt("\r\nToggle on/off:\r\n");
       i=0;
       while( i<numberof_selectable_bauds && i<22)  //22 letter in the alphabet + x to exit menu
       {  temp = i + asciioffset;
          pfmt( "%c) %d\r\n", char(temp),selectable_baudrates[i]); 
          i++;
       }
       pfmt("\r\nx to Exit\r\n> ");
       rp=0;
    } // changed the pin selection

   } // while - loop menu till done
}
void setup() 
{
   pinstatebuf  = (byte *) malloc( 2 );       // 2 bytes worth of bits for max of 20 pins
   
   Serial.begin(115200);
   setup_pins();
   Serial.print("> ");
  
   // Set pins equal to selectable pins to start
   for( int i=0; i<pinslen; i++)             
   {
     pins[i]           = selectable_pins[i];
     pinnames[i]       = selectable_pinnames[i];
     selected_pins[i]  = selectable_pins[i];    // initially no options in the list is selected  
     selected_bauds[i] = selectable_pinnames[i];    // initially no options in the list is selected     
   }

   // Set baudrates to selectable baudrates to start
   for( int i=0; i<numberof_selectable_bauds; i++)
        baudrates[i] = selectable_baudrates[i];
   
   usage();

   pinMode(39,OUTPUT);        // this is custom to my circuit, can be removed if found
   digitalWrite(39,HIGH); 
}


void loop()                                  // main menu
{  
   char command[2];
   if (Serial.available())
   {
      // Read menu option
      delay(5);                    // hoping read buffer is idle after 5 ms
      int i = 0;
      while (Serial.available() && i < 2) 
             command[i++] = Serial.read();
      
      Serial.flush();
      command[i] = '\0';          // terminate string
      Serial.println(command);    // echo back
        
      // Execute command
           if(strcmp(command, "?") == 0)  help();
      else if(strcmp(command, "1") == 0)  passive_parallel_scan(usperrx);
      else if(strcmp(command, "2") == 0)  active_parallel_scan(usperrx);
      else if(strcmp(command, "3") == 0)  active_per_pin_scan(usperrx);
      /*
      else if(strcmp(command, "4") == 0 ) // Do something
      */
      else if(strcmp(command, "5") == 0)  pins_state();
      else if(strcmp(command, "6") == 0)  set_rx_wait();
      else if(strcmp(command, "7") == 0)  select_pins();
      else if(strcmp(command, "8") == 0)  determine_baudrate(0, false);
      else if(strcmp(command, "9") == 0)  select_bauds();
      else if(strcmp(command, "t") == 0)  test_tx();
     
      else usage(); 
 
   }
}
