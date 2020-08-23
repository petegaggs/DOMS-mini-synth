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

uint8_t currentMidiNote; //the note currently being played
uint8_t keysPressedArray[128] = {0}; //to keep track of which keys are pressed

uint32_t lfsr = 1; //32 bit LFSR, must be non-zero to start
uint8_t ditherByte; // random dither
uint8_t lfoPwmFrac; // fractional part of pwm
uint8_t lfoPwmSet; // whole part of pwm
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
  ATTACK,
  SUSTAIN,
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
  switch (envState) {
    case WAIT:
      envPhaccu = 0; // clear the accumulator
      lastEnvCnt = 0;
      ENV_PWM = 0;
      break;
    case ATTACK:
      envPhaccu += envAttackTword; // increment phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt < lastEnvCnt) {
        envState = SUSTAIN; // end of attack stage when counter wraps
      } else {
        ENV_PWM = envCnt;
        lastEnvCnt = envCnt;
      }
      break;
    case SUSTAIN:
      envPhaccu = 0xFFFFFFFF; // clear the accumulator
      lastEnvCnt = 0xFF;
      ENV_PWM = 255;
      break;
    case RELEASE:
      envPhaccu -= envReleaseTword; // decrement phase accumulator
      envCnt = envPhaccu >> 24;  // use upper 8 bits as index into table
      if (envCnt > lastEnvCnt) {
        envState = WAIT; // end of release stage when counter wraps
      } else {
        ENV_PWM = envCnt;
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
  envState = ATTACK;
  if ((lfoWaveform == ENV_UP) || (lfoWaveform == ENV_DOWN)) {
    lfoReset = true;
  }
}

void synthNoteOff(void) {
  envState = RELEASE;
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
  float envControlVoltage;
  // read ADC to calculate the required DDS tuning word, log scale between 1ms and 10s approx
  envControlVoltage = (1023 - analogRead(ENV_ATTACK_PIN)) * float(13)/float(1024); //gives 13 octaves range 1ms to 10s
  envAttackTword = float(13690) * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
  envControlVoltage = (1023 - analogRead(ENV_RELEASE_PIN)) * float(13)/float(1024); //gives 13 octaves range 1ms to 10s
  envReleaseTword = float(13690) * pow(2.0, envControlVoltage); //13690 sets the longest rise time to 10s
}
