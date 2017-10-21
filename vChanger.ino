


/**************************************************************************/
/*! 
Varför är volymen så låg kan man boosta den med kod?

Kan skriva till I2C direkt från interupt istället för med biblotektet?

*/
/**************************************************************************/
//http://playground.arduino.cc/Main/SoftwareI2CLibrary
//https://www.arduino.cc/en/Reference/PortManipulation



#include <WaveHC.h>
#include <WaveUtil.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

//Adafruit_MCP4725 dac;

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

uint16_t nhi, nlo;
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

//Adafruit_MCP4725 dac;
Adafruit_MCP4725 dac; // constructor

#define SDA_PIN 4
#define SDA_PORT PORTC
#define SCL_PIN 5 
#define SCL_PORT PORTC
#include <SoftI2CMaster.h>

//#define I2C_TIMEOUT 500
//#define I2C_NOINTERRUPT 1 //0 or 1 vem fan vet?

//#define I2C_FASTMODE 1
//#define I2C_CPUFREQ (F_CPU/8) //??

int Max7219_pinCLK = 10;
int Max7219_pinCS = 9;
int Max7219_pinDIN = 8;

unsigned char ix;
unsigned char jx; 

int voltSamples=0;

unsigned char disp1[19][8]={
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0b00000000, 0b01000000, 0b00000000, 0b01000000, 0b01000000, 0b00000000, 0b00000000, 0b00000000, 
  0b00000000, 0b01100000, 0b00000000, 0b01100000, 0b01100000, 0b01000000, 0b00000000, 0b00000000, 
  0b01000000, 0b01110000, 0b01100000, 0b01111000, 0b01100000, 0b01100000, 0b01000000, 0b00000000,
  0b01100000, 0b01111000, 0b01110000, 0b01111110, 0b01110000, 0b01111000, 0b01000000, 0b01110000,
  0b01100000, 0b01111000, 0b01110000, 0b01110000, 0b01111100, 0b01110000, 0b01110000, 0b01110000,
  0b01111000, 0b01110000, 0b01111100, 0b01111000, 0b01110000, 0b01111000, 0b01100000, 0b01111110,
  0b01111100, 0b01111000, 0b01111110, 0b01110000, 0b01111100, 0b01111110, 0b01111100, 0b01110000,
// Heart Pattern
 
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

  pinMode(Max7219_pinCLK,OUTPUT);
  pinMode(Max7219_pinCS,OUTPUT);
  pinMode(Max7219_pinDIN,OUTPUT);
  Init_MAX7219();
  if(i2c_init()) { //init i2c system
    Serial.println("ok!");
  }else {
    Serial.println("fuck!");
  }
  
  //Serial.println("Hello!");
  randomSeed(analogRead(A2));
  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  TWBR = 12; // 400 khz
  dac.begin(0x60);
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
    //uint16_t mic =(nhi << 8 | nlo); //Set register to mic

    uint16_t mic =(nhi | nlo); //Set register to mic //already shifted
    
    //for(jx=0;jx<19;jx++)
    //{
    //  delay(1200);
    //  for(ix=1;ix<9;ix++)
    //    Write_Max7219(ix,disp1[jx][ix-1]);
    //    delay(1200);
    //}

    //int mic2 = nhi | nlo;
    if (flag == 1) {
      //int volt = map((int)((buffer2[in] << 8) | buffer1[in]),0,744,0,4094);
      //int volt = map(mic,0,6000,0,4094);
      dac.setVoltage(mic, false);
      //Serial.println(volt);
      //Serial.println(mic);
      
      //i2c_start(0xC0 | I2C_WRITE);
      //i2c_write(0x40);
      //i2c_write(mic / 16);
      //i2c_write((mic % 16) << 4);
      //i2c_stop();
      
      flag = 0;
    }
    voltSamples++;
    if (voltSamples < 1024) {
      //Find Min max mic noice
      if (mic > micMax) {
        micMax = mic;
      }
      if (mic < micMin) {
        micMin = mic;
      }
    } else {
      //Write to display every 1024c
      int diff = (micMax - micMin);
      int mdiff = map(diff,1000,3000,0,7);
      for(ix=1;ix<9;ix++) {
        Write_Max7219(ix,disp1[mdiff][ix-1]);
      }
      voltSamples = 0;
      micMax = 0;
      micMin = 1025;
    }
    //Serial.println((buffer2[in] << 8) | buffer1[in]);
    //
}

void startPitchShift() {
  int pitch = analogRead(1);
  //Serial.print("Pitch: ");
  //Serial.println(pitch);
  
  nSamples = 256 ;
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

    //Debug
    //nhi = buffer1[in];
    //nlo = buffer2[in];
  
    //newsum += abs((((int)buffer2[in] << 8) | buffer1[in]) - 512);
    //Serial.println((buffer2[in] << 8) | buffer1[in]);
    
    //Serial.println(volt);
    
    if(++in >= nSamples) {
      in     = 0;
      //Temp disable this dosent do anything , changed
      //oldsum = (uint8_t)((newsum / nSamples) >> 1); // 0-255
      //newsum = 0L;
    }
  
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
    

    //Bypass vchange debug
    nhi = hi;
    nlo = lo;
    //nhi = buffer1[out] << 8;
    //nlo = buffer2[out];
    flag = 1;
  
    //dac.setVoltage((hi | lo), false);
    //sei();
    //nhi = hi;
    //nlo = lo;
    // Might be possible to tweak 'hi' and 'lo' at this point to achieve
    // different voice modulations -- robot effect, etc.?
    
    //Serial.println(hi);
    //Serial.println(lo);
    if(++out >= nSamples) out = 0;
}

