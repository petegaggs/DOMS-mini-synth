/*
 * Dual Oscillator Mini-Synth (DOMS), by Peter Gaggs
 * MIDI input
 * Arduino Controlled
 * AS3394E IC main voice
 * AS3340 2nd oscillator
 * LFO and envelope implemented with PWM
 * 
 * MIT License
 * Copyright (c) 2019 petegaggs
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h> 
MIDI_CREATE_DEFAULT_INSTANCE();
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <SPI.h>
//pins
#define SPI_CS_PIN 7
#define LFO_PWM_PIN 9
#define ENV_PWM_PIN 10
#define NOISE_PIN 4
#define LFO_WAVE_PIN A0
#define LFO_FREQ_PIN A1
#define ENV_ATTACK_PIN A2
#define ENV_RELEASE_PIN A3
#define TEST_PIN 8 //PB0 test interrupt timing
#define DAC_SCALE_PER_SEMITONE 42
#define MIDI_BASE_NOTE 12 //C0

//timer stuff
#define LFO_PWM OCR1A
#define ENV_PWM OCR1B

// exponential table for envelope
const uint8_t envTable[] PROGMEM = {
0,3,6,9,12,15,18,21,24,27,30,33,35,38,41,43,46,49,51,54,56,59,61,64,66,68,71,73,75,78,80,82,84,86,88,91,93,95,97,99,101,
103,105,106,108,110,112,114,116,118,119,121,123,124,126,128,129,131,133,134,136,137,139,140,142,143,145,146,148,
149,151,152,153,155,156,157,159,160,161,162,164,165,166,167,168,170,171,172,173,174,175,176,177,179,180,181,182,
183,184,185,186,187,188,189,189,190,191,192,193,194,195,196,197,197,198,199,200,201,202,202,203,204,205,205,206,207,
208,208,209,210,210,211,212,212,213,214,214,215,216,216,217,217,218,219,219,220,220,221,222,222,223,223,224,224,225,
225,226,226,227,227,228,228,229,229,230,230,231,231,231,232,232,233,233,234,234,234,235,235,236,236,236,237,237,237,
238,238,239,239,239,240,240,240,241,241,241,242,242,242,243,243,243,243,244,244,244,245,245,245,245,246,246,246,247,
247,247,247,248,248,248,248,249,249,249,249,250,250,250,250,250,251,251,251,251,251,252,252,252,252,252,253,253,253,
253,253,254,254,254,254,254,254,255,255,255,255,255,255};


uint8_t currentMidiNote; //the note currently being played
uint8_t keysPressedArray[128] = {0}; //to keep track of which keys are pressed

uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to start
uint8_t ditherByte; // random dither
uint8_t lfoPwmFrac; // fractional part of pwm
uint8_t lfoPwmSet; // whole part of pwm
uint8_t envPwmSet;
uint8_t envCurrentLevel;
uint8_t envStoredLevel;
uint8_t envMultFactor;
bool triPhase = true; // rising or falling phase for triangle wave

// LFO stuff
bool lfoReset = false;
uint8_t lastLfoCnt = 0;
uint32_t lfoPhaccu;   // phase accumulator
uint32_t lfoTword_m;  // dds tuning word m
uint8_t lfoCnt;       // top 8 bits of accum is index into table
enum lfoWaveTypes {
  RAMP,
  SAW,
  TRI,
  SQR,
  ENV_UP, // sweep up following note on, then stop
  ENV_DOWN, // sweep down following note on, then stop
  SAMPLE_AND_HOLD
};
lfoWaveTypes lfoWaveform;

enum filterEnvStates {
  SWEEP,
  HOLD
};
filterEnvStates filterEnvState;

// envelope stuff
uint32_t envPhaccu;   // phase accumulator
uint32_t envAttackTword;  // dds tuning word attack stage
uint32_t envReleaseTword;  // dds tuning word release stage

uint8_t envCnt;
uint8_t lastEnvCnt;
enum envStates {
  WAIT,
  START_ATTACK,
  ATTACK,
  SUSTAIN,
  START_RELEASE,
  RELEASE
};
envStates envState;

void setup() {
  //MIDI stuff
  MIDI.begin(MIDI_CHANNEL_OMNI);      
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  //SPI stuff
  pinMode (SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.begin(); 
  setNotePitch(60); //middle C for test
  //set env high and LFO low for now
  pinMode(LFO_PWM_PIN, OUTPUT);
  digitalWrite(LFO_PWM_PIN, HIGH);
  pinMode(ENV_PWM_PIN, OUTPUT);
  digitalWrite(ENV_PWM_PIN, HIGH);
  // timer 1 phase accurate PWM 8 bit, no prescaling, non inverting mode channels A & B used
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)| _BV(WGM10);
  TCCR1B = _BV(CS10);
  // timer 1 interrupt
  TIMSK1 = _BV(TOIE1);
  pinMode(NOISE_PIN, OUTPUT);
  envState = WAIT;
  pinMode(TEST_PIN, OUTPUT);
  //synthNoteOn(60); // for test
}

void loop() {
  MIDI.read();
  getLfoParams();
  getEnvParams();
}

SIGNAL(TIMER1_OVF_vect) {
  // timer ISR
  // set PB0 high to measure timing
  PORTB |= 0x01;
  // handle noise signal. Set or clear noise pin PD4 (digital pin 4)
  unsigned lsb = lfsr & 1;
  if (lsb) {
    PORTD |= 0x10;
  } else {
    PORTD &= ~0x10;
  }
  // advance LFSR
  lfsr >>= 1;
  if (lsb) {
    lfsr ^= 0xA3000000u;
  }
  ditherByte = lfsr & 0xFF; // random dither for env and lfo
  // handle LFO DDS
  if (lfoReset) {
    lfoPhaccu = 0; // reset the lfo
    lastLfoCnt = 0;
    lfoReset = false;
    filterEnvState = SWEEP; // start sweep
  } else {
    lfoPhaccu += lfoTword_m; // increment phase accumulator  
  }
  lfoPwmFrac = (lfoPhaccu >> 16) & 0xFF; // fractional part of 16 bit value
  lfoCnt = lfoPhaccu >> 24;  // use upper 8 bits for phase accu as frequency information
  switch (lfoWaveform) {
    case RAMP:
      lfoPwmSet = lfoCnt; // whole part
      if ((lfoPwmFrac > ditherByte) && (lfoPwmSet < 255)) {
        lfoPwmSet += 1;
      }
      break;
    case ENV_UP:
      if (lfoCnt < lastLfoCnt) {
        filterEnvState = HOLD; // end sweep
      }
      if (filterEnvState == SWEEP) {
        lfoPwmSet = lfoCnt; // whole part
        if ((lfoPwmFrac > ditherByte) && (lfoPwmSet < 255)) {
          lfoPwmSet += 1;
        }
      } else {
        lfoPwmSet = 255; // HOLD
      }
      break;
    case SAW:
      lfoPwmSet = 255 - lfoCnt; // whole part
      // note dither is done in reverse for this waveform
      if ((lfoPwmFrac > ditherByte) && (lfoPwmSet > 0)) {
        lfoPwmSet -= 1;
      }
      break;
    case ENV_DOWN:
      if (lfoCnt < lastLfoCnt) {
        filterEnvState = HOLD; // end sweep
      }
      if (filterEnvState == SWEEP) {
        lfoPwmSet = 255 - lfoCnt; // whole part
        // note dither is done in reverse for this waveform
        if ((lfoPwmFrac > ditherByte) && (lfoPwmSet > 0)) {
          lfoPwmSet -= 1;
        }
      } else {
          lfoPwmSet = 0; // HOLD
        }
      break;
    case TRI:
      if (lfoCnt < lastLfoCnt) {
        triPhase = not(triPhase); // reverse direction
      }
      if (triPhase) {
        // rising (same as ramp)
        lfoPwmSet = lfoCnt; // whole part
        if ((lfoPwmFrac > ditherByte) && (lfoPwmSet < 255)) {
          lfoPwmSet += 1;
        }     
      } else {
        // falling (same as saw)
        lfoPwmSet = 255 - lfoCnt; // whole part
        // note dither is done in reverse for this waveform
        if ((lfoPwmFrac > ditherByte) && (lfoPwmSet > 0)) {
          lfoPwmSet -= 1;
        }
      }
      break;
    case SQR:
      if (lfoCnt & 0x80) {
        lfoPwmSet = 255;
      } else {
        lfoPwmSet = 0;
      }
      break;
    case SAMPLE_AND_HOLD:
      if (lfoCnt < lastLfoCnt) {
        lfoPwmSet = ditherByte; // random
      }
      break;
    default:
      break;
  }  
  LFO_PWM = lfoPwmSet;
  lastLfoCnt = lfoCnt;
  // handle Envelope DDS - note this design uses linear envelope, not exponential
  // AS3394 has a strange VCA, nothing happens at first.
  // starting at 0x40000000 and ramping up gives a resonable response
  switch (envState) {
    case WAIT:
      envPhaccu = 0;
      lastEnvCnt = 0;
      envCurrentLevel = 0;
      ENV_PWM = 0;
      break;
    case START_ATTACK:
      envPhaccu = 0;
      lastEnvCnt = 0;
      if (envCurrentLevel < 64) {
        envCurrentLevel = 64; //this gives reasonable response for AS3394 VCA
      }
      envMultFactor = 255 - envCurrentLevel;
      envStoredLevel = envCurrentLevel;
      envState = ATTACK;      
      break;
    case ATTACK:
      envPhaccu += envAttackTword; // increment phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt < lastEnvCnt) {
        envState = SUSTAIN; // end of attack stage when counter wraps
      } else {
        envCurrentLevel = ((envMultFactor * pgm_read_byte_near(envTable + envCnt)) >> 8) + envStoredLevel;
        ENV_PWM = envCurrentLevel;
        lastEnvCnt = envCnt;
      }
      break;
    case SUSTAIN:
      envPhaccu = 0;
      lastEnvCnt = 0;
      envCurrentLevel = 255;
      ENV_PWM = envCurrentLevel;
      break;
    case START_RELEASE:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      envMultFactor = envCurrentLevel;
      envStoredLevel = envCurrentLevel;
      envState = RELEASE;
      break;
    case RELEASE:
      envPhaccu += envReleaseTword; // increment phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt < lastEnvCnt) {
        envState = WAIT; // end of release stage when counter wraps
      } else {
        envCurrentLevel = envStoredLevel - ((envMultFactor * pgm_read_byte_near(envTable + envCnt)) >> 8);
        ENV_PWM = envCurrentLevel;
        lastEnvCnt = envCnt;
      }
      break;
    default:
      break;
  }
  // clear PB0 to measure timing
  PORTB &= ~0x01;
}

void handleNoteOn(uint8_t channel, uint8_t pitch, uint8_t velocity) { 
  // this function is called automatically when a note on message is received 
  keysPressedArray[pitch] = 1;
  synthNoteOn(pitch);
}

void handleNoteOff(uint8_t channel, uint8_t pitch, uint8_t velocity)
{
  keysPressedArray[pitch] = 0; //update the array holding the keys pressed 
  if (pitch == currentMidiNote) {
    //only act if the note released is the one currently playing, otherwise ignore it
    int highestKeyPressed = findHighestKeyPressed(); //search the array to find the highest key pressed, will return -1 if no keys pressed
    if (highestKeyPressed != -1) { 
      //there is another key pressed somewhere, so the note off becomes a note on for the highest note pressed
      synthNoteOn(highestKeyPressed);
    }    
    else  {
      //there are no other keys pressed so proper note off
      synthNoteOff();
    }
  }  
}

int findHighestKeyPressed(void) {
  //search the array to find the highest key pressed. Return -1 if no keys are pressed
  int highestKeyPressed = -1; 
  for (int count = 0; count < 127; count++) {
    //go through the array holding the keys pressed to find which is the highest (highest note has priority), and to find out if no keys are pressed
    if (keysPressedArray[count] == 1) {
      highestKeyPressed = count; //find the highest one
    }
  }
  return(highestKeyPressed);
}

void synthNoteOn(uint8_t note) {
  //starts playback of a note
  setNotePitch(note); //set the oscillator pitch
  currentMidiNote = note; //store the current note
  if (envState != ATTACK) {
    envState = START_ATTACK;
  }
  if ((lfoWaveform == ENV_UP) || (lfoWaveform == ENV_DOWN)) {
    lfoReset = true;
  }
}

void synthNoteOff(void) {
  envState = START_RELEASE;
}

void setNotePitch(uint8_t note) {
  //receive a midi note number and set the DAC voltage accordingly for the pitch CV
  dacWrite((note - MIDI_BASE_NOTE) * DAC_SCALE_PER_SEMITONE);
}

void dacWrite(uint16_t value) {
  // write to MCP4822 SPI DAC
  //send a value to DAC A (osc1)
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(0x10 | ((value >> 8) & 0x0F)); //bits 0..3 are bits 8..11 of 12 bit value, bits 4..7 are control data 
  SPI.transfer(value & 0xFF); //bits 0..7 of 12 bit value
  digitalWrite(SPI_CS_PIN, HIGH);
  //send same value to DAC B (osc2)
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(0x90 | ((value >> 8) & 0x0F)); //bits 0..3 are bits 8..11 of 12 bit value, bits 4..7 are control data 
  SPI.transfer(value & 0xFF); //bits 0..7 of 12 bit value
  digitalWrite(SPI_CS_PIN,HIGH);
}

void getLfoParams() {
  // read ADC to calculate the required DDS tuning word, log scale between 0.01Hz and 10Hz approx
  uint32_t tempVal = analogRead(LFO_FREQ_PIN);
  if (lfoWaveform == TRI) {
    lfoTword_m = (tempVal << 11) + 687; // gives about 0.01 - 8Hz range
  } else {
    lfoTword_m = (tempVal << 10) + 1374; // gives about 0.01 - 8Hz range    
  }
  // read ADC to get the LFO wave type
  int waveType = analogRead(LFO_WAVE_PIN) >> 7;
  switch (waveType) {
    case 0:
      lfoWaveform = RAMP;
      break;
    case 1:
      lfoWaveform = SAW;
      break;
    case 2:
      lfoWaveform = TRI;
      break;
    case 3:
      lfoWaveform = SQR;
      break;
    case 4:
      lfoWaveform = ENV_UP;
      break;
    case 5:
      lfoWaveform = ENV_DOWN;
      break;
    case 6:
      lfoWaveform = SAMPLE_AND_HOLD;
      break;
    case 7:
      lfoWaveform = SQR; // reserved
      break;
    default:
      lfoWaveform = RAMP;
      break;    
  }
}

void getEnvParams() {
  envAttackTword = 4294967296 / ((float(analogRead(ENV_ATTACK_PIN)) * 305.474) + 31.25); //gives 1ms to 10 secs approx
  envReleaseTword = 4294967296 / ((float(analogRead(ENV_RELEASE_PIN)) * 305.474) + 31.25); //gives 1ms to 10 secs approx
  //ENV_PWM = analogRead(ENV_ATTACK_PIN) >> 2;
}
