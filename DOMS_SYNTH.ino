/*
 * Dual Oscillator Mini-Synth (DOMS), by Peter Gaggs
 * MIDI input
 * Arduino Controlled
 * AS3394E IC main voice
 * AS3340 2nd oscillator
 * Work in progress, not finished
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
#define DAC_SCALE_PER_SEMITONE 42
#define MIDI_BASE_NOTE 12 //C0

uint8_t currentMidiNote; //the note currently being played
uint8_t keysPressedArray[128] = {0}; //to keep track of which keys are pressed

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
  digitalWrite(LFO_PWM_PIN, LOW);
  pinMode(ENV_PWM_PIN, OUTPUT);
  digitalWrite(ENV_PWM_PIN, HIGH);
}

void loop() {
  MIDI.read();
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
}

void synthNoteOff(void) {
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
