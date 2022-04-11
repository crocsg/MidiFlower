/*
 MIT License

Copyright (c) 2022 S Godin (Climate Change Lab)
Copyright (c) 2016 electricityforprogress

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


This project is based on https://github.com/electricityforprogress/MIDIsprout great
work about biodata sonification
*/

#include <Arduino.h>
#include <BLEMidi.h>

#include "midinote.h"
#include "flower_music.h"
#include "sequence.h"
#include "flower_sensor.h"

#define LED 5

#define DESIRED_EVENT 6
#define ARRAYLEN(a) ((sizeof(a)) / (sizeof(a[0])))


// manage LEDs without delay() jgillick/arduino-LEDFader https://github.com/jgillick/arduino-LEDFader.git

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void flowersensor_measure (uint32_t min, uint32_t max, uint32_t averg, uint32_t delta, float stdevi, float stdevical);

const byte interruptPin = 12; // galvanometer input
unsigned long previousMillis = 0;
unsigned long currentMillis = 1;

// definition des tempos
#define BPM90 ((60 * 1000) / 90)
#define BPM120 ((60 * 1000) / 120)
#define BPM160 ((60 * 1000) / 160)
#define BPM BPM120

// nombre maxi de notes midi simultanees dans le sequencer
#define MAX_MIDI_NOTES 64

// sequenceur midi
CMidiFlowerSequencer sequencer = CMidiFlowerSequencer(MAX_MIDI_NOTES);

// definition des "pistes" midi
CSequence A = CSequence(60, 4, BPM, 40);
CSequence B = CSequence(120, 4, BPM, 50);
CSequence C = CSequence(120, 4, BPM / 4, 40);
CSequence D = CSequence(120, 4, BPM, 40);



static uint32_t chipId = 0;

static char bleserverid[64] = "";



// microcontroleur initialisation
void setup()
{

  // get esp32 serial
  for (int i = 0; i < 17; i = i + 8)
  {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  // initialize led output
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); // set led ON

  // start BLE Midi server
  sprintf(bleserverid, "BioData_%08lx MIDI device", chipId); // build BLE Midi name
  BLEMidiServer.begin(bleserverid);                          // initialize bluetooth midi
  // Serial.begin(115200);                 //initialize Serial for debug

  // register midi track
  sequencer.register_track(&A);
  sequencer.register_track(&B);
  sequencer.register_track(&C);
  sequencer.register_track(&D);
  
  // start flower sensor
  flower_sensor_init(interruptPin);
  flower_sensor_set_analyse_short(1);

  // define a function to get measures
  flower_sensor_set_callback (flowersensor_measure);
}

// 
void loop()
{
  
  sequencer.Loop ();
  // build flower measure
  // must be called often
  flower_sensor_build_mes();

  if (psequences[0]->getnbnotes() > 15)
  {
    flower_sensor_set_analyse_short(0);
  }

  
}

// min    valeur min
// max    valeur max
// averg  moyenne
// delta  max - min
// stdevi ecart-type
// stdevical stdevi * threshold
void flowersensor_measure (uint32_t min, uint32_t max, uint32_t averg, uint32_t delta, float stdevi, float stdevical)
{

}

void setNote(int value, int velocity, long duration, int notechannel)
{

  if (psequences[0]->getnbnotes() < psequences[0]->size() / 3)
    psequences[0]->addNote(currentMillis, value, velocity, duration, 1);
  else if (psequences[1]->getnbnotes() < psequences[1]->size() / 3)
    psequences[1]->addNote(currentMillis, value, velocity, duration, 2);
  /*else if (psequences[2]->getnbnotes() < psequences[2]->size () / 2)
    psequences[2]->addNote (currentMillis, value, velocity, duration, 3);*/
  else
  {
    uint16_t seq = currentMillis % psequences.size();
    psequences[seq]->setmode(CSequence::playmode::Learn);
    psequences[seq]->addNote(currentMillis, value, velocity, duration, seq + 1);
  }
}

// provide float map function
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// interrupt timing sample array
