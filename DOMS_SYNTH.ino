/*
 * Dual Oscillator Mini-Synth (DOMS), by Peter Gaggs
 * MIDI input
 * Arduino Controlled
 * AS3394E IC main voice
 * AS3340 2nd oscillator
 * LFO and envelope implemented with PWM
 * duophonic - first oscillator plays highest note, 2nd oscillator plays lowest.
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
#define PITCH_BEND_FACTOR 8 // how much to respond to pitch bend, the smaller this number, the more we respond

//timer stuff
#define LFO_PWM OCR1A
#define ENV_PWM OCR1B

// exponential table for envelope attack phase only. starts with offset of 64 as nothing much happens below that
const uint8_t expTable[] PROGMEM = {
64,66,69,71,73,75,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,110,111,113,115,117,119,120,122,124,125,127,
128,130,132,133,135,136,138,139,141,142,144,145,146,148,149,150,152,153,154,156,157,158,159,161,162,163,164,165,
167,168,169,170,171,172,173,174,175,177,178,179,180,181,182,183,183,184,185,186,187,188,189,190,191,192,192,193,
194,195,196,197,197,198,199,200,201,201,202,203,203,204,205,206,206,207,208,208,209,210,210,211,212,212,213,213,
214,215,215,216,216,217,217,218,219,219,220,220,221,221,222,222,223,223,224,224,225,225,226,226,226,227,227,228,
228,229,229,230,230,230,231,231,232,232,232,233,233,233,234,234,235,235,235,236,236,236,237,237,237,238,238,238,
239,239,239,239,240,240,240,241,241,241,241,242,242,242,243,243,243,243,244,244,244,244,245,245,245,245,245,246,
246,246,246,247,247,247,247,247,248,248,248,248,248,249,249,249,249,249,250,250,250,250,250,250,251,251,251,251,
251,251,252,252,252,252,252,252,252,253,253,253,253,253,253,253,254,254,254,254,254,254,254,255,255,255,255};

// reverse exponential table, for establishing where to re-start attack part of envelope
const uint8_t revExpTable[] PROGMEM = {
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,1,1,2,2,3,3,3,4,4,5,5,6,6,7,7,8,8,8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,
18,19,19,20,21,21,22,22,23,23,24,24,25,26,26,27,27,28,28,29,30,30,31,31,32,33,33,34,35,35,36,37,37,38,39,39,40,
41,41,42,43,43,44,45,45,46,47,48,48,49,50,51,51,52,53,54,55,55,56,57,58,59,60,60,61,62,63,64,65,66,67,68,69,70,
70,71,72,73,74,75,77,78,79,80,81,82,83,84,85,86,88,89,90,91,92,94,95,96,98,99,100,102,103,105,106,108,109,111,
112,114,115,117,119,120,122,124,126,128,130,132,134,136,138,140,142,144,147,149,152,154,157,160,162,165,168,171,
175,178,181,185,189,193,197,201,206,211,216,221,227,234,241,248,255};

uint8_t osc1_midi_note, osc2_midi_note; //the note currently being played
uint8_t keysPressedArray[128] = {0}; //to keep track of which keys are pressed
uint8_t key_pressed_count = 0;

uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to start
uint8_t ditherByte; // random dither
uint8_t lfoPwmFrac; // fractional part of pwm
uint8_t lfoPwmSet; // whole part of pwm
uint8_t envPwmSet;
uint8_t envCurrentLevel;
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

int16_t midi_osc1_control, midi_osc2_control;
int16_t midiPitchBendControl = 0;

void setup() {
  //MIDI stuff
  MIDI.begin(MIDI_CHANNEL_OMNI);      
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandlePitchBend(handlePitchBend); 
  //SPI stuff
  pinMode (SPI_CS_PIN, OUTPUT);
  digitalWrite(SPI_CS_PIN, HIGH);
  SPI.begin(); 
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
  // handle Envelope DDS - attack phase uses exponential control, release phase is linear
  // This is the best way to control the '3394 VCA, to my ears anyway
  switch (envState) {
    case WAIT:
      envPhaccu = 0;
      lastEnvCnt = 0;
      envCurrentLevel = 0;
      ENV_PWM = 0;
      break;
    case START_ATTACK:
      // use the reverse exp table to work out where we need to re-start the attack phase
      envPhaccu = ((uint32_t) pgm_read_byte_near(revExpTable + envCurrentLevel)) << 24;
      lastEnvCnt = 0;
      envState = ATTACK;      
      break;
    case ATTACK:
      envPhaccu += envAttackTword; // increment phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt < lastEnvCnt) {
        envState = SUSTAIN; // end of attack stage when counter wraps
      } else {
        envCurrentLevel = pgm_read_byte_near(expTable + envCnt);
        ENV_PWM = envCurrentLevel;
        lastEnvCnt = envCnt;
      }
      break;
    case SUSTAIN:
      envPhaccu = 0xFFFFFFFF;
      lastEnvCnt = 0xFF;
      envCurrentLevel = 255;
      ENV_PWM = envCurrentLevel;
      break;
    case START_RELEASE:
      envPhaccu = ((uint32_t) envCurrentLevel) << 24;
      lastEnvCnt = 0xFF;
      envState = RELEASE;
      break;
    case RELEASE:
      envPhaccu -= envReleaseTword; // decrement phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt > lastEnvCnt) {
        envState = WAIT; // end of release stage when counter wraps
      } else {
        envCurrentLevel = envCnt; //linear decay
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
  key_pressed_count++;
  osc1_midi_note = findHighestKeyPressed();
  osc2_midi_note = findLowestKeyPressed();
  synthNoteOn();
}

void handleNoteOff(uint8_t channel, uint8_t pitch, uint8_t velocity) {
  keysPressedArray[pitch] = 0; //update the array holding the keys pressed
  key_pressed_count--;
  if (key_pressed_count == 0) {
    synthNoteOff();
  } else if (key_pressed_count > 1) {
  osc1_midi_note = findHighestKeyPressed();
  osc2_midi_note = findLowestKeyPressed();
  synthNoteOn();
  }
}


void updateNotePitch() {
  // update note pitch, taking into account midi note and midi pitchbend
  dacWrite(midi_osc1_control + midiPitchBendControl, midi_osc2_control + midiPitchBendControl);
}
 
void handlePitchBend (byte channel, int bend) {
  midiPitchBendControl = bend >> PITCH_BEND_FACTOR;
  updateNotePitch();
}

int findHighestKeyPressed(void) {
  //search the array to find the highest key pressed. Return -1 if no keys are pressed
  int highestKeyPressed = -1; 
  for (int count = 0; count < 128; count++) {
    //go through the array holding the keys pressed to find which is the highest (highest note has priority), and to find out if no keys are pressed
    if (keysPressedArray[count] == 1) {
      highestKeyPressed = count; //find the highest one
    }
  }
  return(highestKeyPressed);
}

int findLowestKeyPressed(void) {
  // search the array to find the lowest key pressed. Return -1 if no keys are pressed
  int lowestKeyPressed = -1;
  for (int count = 0; count < 128; count++) {
    //go through the array holding the keys pressed to find which is the highest (highest note has priority), and to find out if no keys are pressed
    if (keysPressedArray[count] == 1) {
      lowestKeyPressed = count; //find the highest one
      break;
    }
  }
  return(lowestKeyPressed);
}

void synthNoteOn(void) {
  //starts playback of a note
  midi_osc1_control = (((int16_t) osc1_midi_note) - MIDI_BASE_NOTE) * DAC_SCALE_PER_SEMITONE;
  midi_osc2_control = (((int16_t) osc2_midi_note) - MIDI_BASE_NOTE) * DAC_SCALE_PER_SEMITONE;
  updateNotePitch();
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

void dacWrite(uint16_t value_a, uint16_t value_b) {
  // write to MCP4822 SPI DAC
  //send a value to DAC A (osc1)
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(0x10 | ((value_a >> 8) & 0x0F)); //bits 0..3 are bits 8..11 of 12 bit value, bits 4..7 are control data 
  SPI.transfer(value_a & 0xFF); //bits 0..7 of 12 bit value
  digitalWrite(SPI_CS_PIN, HIGH);
  //send same value to DAC B (osc2)
  digitalWrite(SPI_CS_PIN, LOW);
  SPI.transfer(0x90 | ((value_b >> 8) & 0x0F)); //bits 0..3 are bits 8..11 of 12 bit value, bits 4..7 are control data 
  SPI.transfer(value_b & 0xFF); //bits 0..7 of 12 bit value
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
}
