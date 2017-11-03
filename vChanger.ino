


/**************************************************************************/
/*! 
Varför är volymen så låg kan man boosta den med kod?

Kan skriva till I2C direkt från interupt istället för med biblotektet?

*/
/**************************************************************************/
//http://playground.arduino.cc/Main/SoftwareI2CLibrary
//https://www.arduino.cc/en/Reference/PortManipulation

uint16_t mic = 0;
int NormMic = 0;
int micFilter = 0;
int oldmic = 0;

int volMax = 0, volMin = 9999;

bool Onstate = false;
int Checkbtn = 0;

int diff = 0;

//mdif screeen
int OldMdiff = 0;

//Clear screen
bool ScreenCleared = true;


uint16_t amax,amin,bmax,bmin;

#include <WaveHC.h>
#include <WaveUtil.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

#define ADC_CHANNEL 0 // Microphone on Analog pin 0
uint16_t in = 0, out = 0, xf = 0, nSamples; // Audio sample counters
uint8_t  adc_save;                          // Default ADC mode
int micMax = 0, micMin = 9999;
// Wave shield DAC: digital pins 2, 3, 4, 5
#define DAC_CS_PORT    PORTD
#define DAC_CS         PORTD2
#define DAC_CLK_PORT   PORTD
#define DAC_CLK        PORTD3
#define DAC_DI_PORT    PORTD
#define DAC_DI         PORTD4
#define DAC_LATCH_PORT PORTD
#define DAC_LATCH      PORTD5

uint8_t nhi, nlo;
bool flag = 0;

// WaveHC didn't declare it's working buffers private or static,
// so we can be sneaky and borrow the same RAM for audio sampling!
extern uint8_t
  buffer1[PLAYBUFFLEN],                   // Audio sample LSB
  buffer2[PLAYBUFFLEN];                   // Audio sample MSB
#define XFADE     16                      // Number of samples for cross-fade
#define MAX_SAMPLES (PLAYBUFFLEN - XFADE) // Remaining available audio samples

// Used for averaging all the audio samples currently in the buffer
uint8_t       oldsum = 0;
unsigned long newsum = 0L;

#define SDA_PIN 4
#define SDA_PORT PORTC
#define SCL_PIN 5 
#define SCL_PORT PORTC


//Display pins
int Max7219_pinCLK = 10;
int Max7219_pinCS = 9;
int Max7219_pinDIN = 8;

unsigned char ix;
unsigned char jx; 

int voltSamples=0;

unsigned char disp1[8][8]={
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01011000, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01011000, 0b01011100, 0b00000000, 
  0b00000000, 0b00000000, 0b00000000, 0b00010000, 0b01010000, 0b01111100, 0b11111110, 0b00000000,
  0b00000000, 0b00010000, 0b00010000, 0b01010100, 0b01111101, 0b11111101, 0b11111111, 0b00000000,
  0b00000000, 0b00000000, 0b00001000, 0b01001000, 0b01111111, 0b11111111, 0b11111111, 0b00000000,
  0b00000000, 0b00000001, 0b00100001, 0b10110101, 0b11111101, 0b11111111, 0b11111111, 0b00000000,
  0b00000000, 0b00100100, 0b01110101, 0b01110111, 0b11111111, 0b11111111, 0b11111111, 0b00000000
//
 
};

void Write_Max7219_byte(unsigned char DATA) 
{   
  unsigned char i;
  digitalWrite(Max7219_pinCS,LOW);  
  for(i=8;i>=1;i--)
  {    
    digitalWrite(Max7219_pinCLK,LOW);
    digitalWrite(Max7219_pinDIN,DATA&0x80);
    DATA = DATA<<1;
    digitalWrite(Max7219_pinCLK,HIGH);
  }                                 
}
 
void Write_Max7219(unsigned char address,unsigned char dat)
{
  digitalWrite(Max7219_pinCS,LOW);
  Write_Max7219_byte(address);          
  Write_Max7219_byte(dat);               
  digitalWrite(Max7219_pinCS,HIGH);
}
 
void Init_MAX7219(void)
{
  Write_Max7219(0x09, 0x00);      
  Write_Max7219(0x0a, 0x03a);      
  Write_Max7219(0x0b, 0x07);       
  Write_Max7219(0x0c, 0x01);      
  Write_Max7219(0x0f, 0x00);      

  Write_Max7219(0x01, 0b0);   
  Write_Max7219(0x02, 0b0); 
  Write_Max7219(0x03, 0b0); 
  Write_Max7219(0x04, 0b0); 
  Write_Max7219(0x05, 0b0); 
  Write_Max7219(0x06, 0b0); 
  Write_Max7219(0x07, 0b0); 
  Write_Max7219(0x08, 0b0); 
  
}

void setup(void) {
  Serial.begin(9600);

  amax = 0;
  amin = 4000;
  bmax = 0;
  bmin = 4000;

  pinMode(12, INPUT); //start btn

  pinMode(Max7219_pinCLK,OUTPUT);
  pinMode(Max7219_pinCS,OUTPUT);
  pinMode(Max7219_pinDIN,OUTPUT);
  Init_MAX7219(); //Init display

  //Serial.println("Hello!");
  randomSeed(analogRead(A2));
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  TWBR = 12; // 400 khz
  //dac.begin(0x60);
  //Serial.println("Generating a triangle wave");

  pinMode(2, OUTPUT);    // Chip select
  pinMode(3, OUTPUT);    // Serial clock
  pinMode(4, OUTPUT);    // Serial data
  pinMode(5, OUTPUT);    // Latch
  digitalWrite(2, HIGH); // Set chip select high

  // Optional, but may make sampling and playback a little smoother:
  // Disable Timer0 interrupt.  This means delay(), millis() etc. won't
  // work.  Comment this out if you really, really need those functions.
  TIMSK0 = 0;
  analogReference(EXTERNAL); // 3.3V to AREF
  adc_save = ADCSRA;         // Save ADC setting for restore later
  startPitchShift();
}

void loop(void) {
    if (Checkbtn > 4500) {
      Onstate = digitalRead(12);
      
      Checkbtn = 0;
    }
    Checkbtn++;
    
    if (flag == 1) {  
      mic =(nhi << 8 | nlo); //Set register to mic 

      if (Onstate) {
        if (mic > volMax) {
          volMax = mic;
        }
        if (mic < volMin) {
          volMin = mic;
        }
      }
        
      NormMic = map(mic, volMin, volMax, 400, 1000 ); //400 , 700
      //Find right volume

      int micDiff = oldmic - NormMic;
      if (micDiff < 43 && micDiff > 0) { //43
        NormMic -= micDiff;
      } else if (micDiff > -43 && micDiff < 0) {
        NormMic += micDiff;
      }

      oldmic = NormMic; //Save old mic

      //Serial.println(mic);
      //Serial.print(" ");
      //Serial.print(micMax);
      //Serial.print(" ");
      //Serial.print(micMin);
      //Serial.print(" ");
      //Serial.println(0);
      
      if (Onstate) { // fixa så att den inte frågar så ofta. kolla bara ibland och använd en toggle
        Wire.beginTransmission(0x60);
        Wire.write(64);                     // cmd to update the DAC
        Wire.write(NormMic >> 4);        // the 8 most significant bits...
        Wire.write((NormMic & 15) << 4); // the 4 least significant bits...
        Wire.endTransmission();
        flag = 0;
      }

      
    }

    if (Onstate) {
      if (ScreenCleared) {ScreenCleared = false;}
      voltSamples++;
      if (voltSamples < 1200) {
         //Find Min max mic noice
        if (NormMic > micMax) {
          micMax = NormMic;
        }
        if (NormMic < micMin) {
          micMin = NormMic;
        }
      } else {  
        //Write to display every 1024c

        //diff = (micMax - micMin);
        //skip min
       
        int mdiff = map(micMax,810,990,0,7); //600 is 0zero
         //Serial.println(mdiff);
        if (mdiff > 7) {
          mdiff = 7;
        } else if (mdiff < 0) {
          mdiff = 0;
        }
        
        if (OldMdiff !=mdiff) { //if diff not change skip
          for(ix=2;ix<8;ix++) { // Dont write first and last line always zero.
            //we dont need to write zeros
            Write_Max7219(ix,disp1[mdiff][ix-1]); //mdiff
          }
        }
        OldMdiff = mdiff;
        
        voltSamples = 0;
        micMax = 0;
        micMin = 1025;
        //maxmin reset was here
      }
    } else if (!ScreenCleared) {
      for(ix=2;ix<8;ix++) {
        Write_Max7219(ix,disp1[0][ix-1]); //mdiff
      }
      //volMax = 0;
      //volMin = 9999;
      ScreenCleared = true;
    }

    
}

void startPitchShift() {
  int pitch = analogRead(1);
  //Serial.print("Pitch: ");
  //Serial.println(pitch);


  // Right now the sketch just uses a fixed sound buffer length of
  // 128 samples.  It may be the case that the buffer length should
  // vary with pitch for better results...further experimentation
  // is required here.
  nSamples = 128 ;
  //nSamples = F_CPU / 3200 / OCR2A; // ???
  //if(nSamples > MAX_SAMPLES)      nSamples = MAX_SAMPLES;
  //else if(nSamples < (XFADE * 2)) nSamples = XFADE * 2;

  memset(buffer1, 0, nSamples + XFADE); // Clear sample buffers
  memset(buffer2, 2, nSamples + XFADE); // (set all samples to 512)

  TCCR2A = _BV(WGM21) | _BV(WGM20); // Mode 7 (fast PWM), OC2 disconnected
  TCCR2B = _BV(WGM22) | _BV(CS21) | _BV(CS20);  // 32:1 prescale
  OCR2A  = map(pitch, 0, 1023,
    F_CPU / 32 / (9615 / 2),  // Lowest pitch  = -1 octave
    F_CPU / 32 / (9615 * 2)); // Highest pitch = +1 octave
  
  DIDR0 |= _BV(ADC0D);  // Disable digital input buffer on ADC0
  ADMUX  = ADC_CHANNEL; // Channel sel, right-adj, AREF to 3.3V regulator
  ADCSRB = 0;           // Free-run mode
  ADCSRA = _BV(ADEN) |  // Enable ADC
    _BV(ADSC)  |        // Start conversions
    _BV(ADATE) |        // Auto-trigger enable
    _BV(ADIE)  |        // Interrupt enable
    _BV(ADPS2) |        // 128:1 prescale...
    _BV(ADPS1) |        //  ...yields 125 KHz ADC clock...
    _BV(ADPS0);         //  ...13 cycles/conversion = ~9615 Hz

  TIMSK2 |= _BV(TOIE2); // Enable Timer2 overflow interrupt
  sei(); // disable global interrupts
}

ISR(ADC_vect, ISR_BLOCK) { // ADC conversion complete
  // Save old sample from 'in' position to xfade buffer:
  buffer1[nSamples + xf] = buffer1[in];
  buffer2[nSamples + xf] = buffer2[in];
  if(++xf >= XFADE) xf = 0;

  // Store new value in sample buffers:
  buffer1[in] = ADCL; // MUST read ADCL first!
  buffer2[in] = ADCH;
  if(++in >= nSamples) in = 0;
}

ISR(TIMER2_OVF_vect) { // Playback interrupt
  uint16_t s;
  uint8_t  w, inv, hi, lo, bit;
  int      o2, i2, pos;

  // Cross fade around circular buffer 'seam'.
  if((o2 = (int)out) == (i2 = (int)in)) {
    // Sample positions coincide.  Use cross-fade buffer data directly.
    pos = nSamples + xf;
    hi = (buffer2[pos] << 2) | (buffer1[pos] >> 6); // Expand 10-bit data
    lo = (buffer1[pos] << 2) |  buffer2[pos];       // to 12 bits
  } if((o2 < i2) && (o2 > (i2 - XFADE))) {
    // Output sample is close to end of input samples.  Cross-fade to
    // avoid click.  The shift operations here assume that XFADE is 16;
    // will need adjustment if that changes.
    w   = in - out;  // Weight of sample (1-n)
    inv = XFADE - w; // Weight of xfade
    pos = nSamples + ((inv + xf) % XFADE);
    s   = ((buffer2[out] << 8) | buffer1[out]) * w +
          ((buffer2[pos] << 8) | buffer1[pos]) * inv;
    hi = s >> 10; // Shift 14 bit result
    lo = s >> 2;  // down to 12 bits
  } else if (o2 > (i2 + nSamples - XFADE)) {
    // More cross-fade condition
    w   = in + nSamples - out;
    inv = XFADE - w;
    pos = nSamples + ((inv + xf) % XFADE);
    s   = ((buffer2[out] << 8) | buffer1[out]) * w +
          ((buffer2[pos] << 8) | buffer1[pos]) * inv;
    hi = s >> 10; // Shift 14 bit result
    lo = s >> 2;  // down to 12 bits
  } else {
    // Input and output counters don't coincide -- just use sample directly.
    hi = (buffer2[out] << 2) | (buffer1[out] >> 6); // Expand 10-bit data
    lo = (buffer1[out] << 2) |  buffer2[out];       // to 12 bits
  }
  

  // Might be possible to tweak 'hi' and 'lo' at this point to achieve
  // different voice modulations -- robot effect, etc.?

  flag = 1;
  //nhi = buffer2[in];
  //nlo = buffer1[in];
  nhi = hi;
  nlo = lo;

  if(++out >= nSamples) out = 0;
}
