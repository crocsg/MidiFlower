/*
 MIT License

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


This project is based on https://github.com/electricityforprogress/MIDIsprout great work about biodata sonification
*/

#include <Arduino.h>
#include <BLEMidi.h>

#include "midinote.h"
#include "midisequencer.h"
#include "sequence.h"

#define SAMPLESIZE  32
#define LED 5

#define DESIRED_EVENT 6
#define ARRAYLEN(a) ((sizeof(a))/(sizeof(a[0])))

//manage LEDs without delay() jgillick/arduino-LEDFader https://github.com/jgillick/arduino-LEDFader.git
void sample();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void analyzeSample();
void checkNote();
void checkControl();
void midiSerial(int type, int channel, int data1, int data2);
int scaleNote(int note, int scale[], int root);
void playloop ();
void addloop (int value, int velocity, long duration, int notechannel);
void checkloop ();
void playDrum ();
void addDrum (int value, int velocity, long duration, int notechannel);
void checkDrum ();
void addlongloop (int value, int velocity, long duration, int notechannel);
void playLongLoop ();
void checkLongloop ();

//******************************
//set scaled values, sorted array, first element scale length
int scaleMajor[]  = {7,1, 3, 5, 6, 8, 10, 12};
int scaleDiaMinor[]  = {7,1, 3, 4, 6, 8, 9, 11};
int scaleIndian[]  = {7,1, 2, 2, 5, 6, 9, 11};
int scaleMinor[]  = {7,1, 3, 4, 6, 8, 9, 11};
int scaleChrom[] = {12,1,2,3,4,5,6,7,8,9,10,11,12};
int *scaleSelect = scaleMajor; //initialize scaling
int root = 0; //initialize for root
static byte state;
//*******************************

const byte interruptPin = 12; //galvanometer input

byte samplesize = SAMPLESIZE / 2; //set sample array size
//const byte analysize = SAMPLESIZE - 1;  //trim for analysis array

const byte polyphony = 8; //above 8 notes may run out of ram
byte channel = 1;  //setting channel to 11 or 12 often helps simply computer midi routing setups
int noteMin = 36; //C2  - keyboard note minimum
int noteMax = 96; //C7  - keyboard note maximum






volatile unsigned long microseconds; //sampling timer
volatile byte sindex = 0;
volatile unsigned long samples[SAMPLESIZE];

float threshold = 1;  //change threshold multiplier


unsigned long previousMillis = 0;
unsigned long currentMillis = 1;
unsigned long batteryCheck = 0; //battery check delay timer
 
uint32_t threshold_last_millis = 0;
unsigned int  threshold_evt = 0;

#define BPM90 ((60*1000)/90)
#define BPM120 ((60*1000)/120)
#define BPM160 ((60*1000)/160)
#define BPM BPM120
CMidiSequencer sequencer = CMidiSequencer(64);

CSequence A = CSequence(60, 4, BPM, 40);
CSequence B = CSequence(120, 4, BPM, 50);
CSequence C = CSequence(120, 4, BPM/4, 40);
CSequence D = CSequence(120, 4, BPM, 40);
std::vector<CSequence*> psequences;

uint32_t sequence_time = 0;
uint16_t sequence_index = 0;
uint32_t chipId = 0;

char bleserverid[64] = "";
void setup()
{
  
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  
  pinMode (LED, OUTPUT);    // initilize led output
  digitalWrite(LED, HIGH);   // set led ON

  sprintf (bleserverid, "BioData_%08lx MIDI device", chipId);
  BLEMidiServer.begin(bleserverid); // initialize bluetooth midi
  //Serial.begin(115200);                       //initialize Serial for debug
   
  
  //MIDIpanic(); //dont panic, unless you are sure it is nessisary
  psequences.push_back(&A);
  psequences.push_back(&B);
  psequences.push_back(&C);
  //psequences.push_back(&D);
  attachInterrupt(interruptPin, sample, RISING);  //begin sampling data from interrupt
  
}

void loop()
{
  currentMillis = millis();   //manage time
  
  
  if (currentMillis - sequence_time >= (BPM / 8))
  {
    MIDImessage note;
    sequence_time = currentMillis;
    if ((sequence_index > 0 && psequences[sequence_index]->getnbnotes() > psequences[sequence_index]->size() / 4) || 
      sequence_index == 0)
    {
      if (psequences[sequence_index]->play(currentMillis, &note) == 1)
      {
          //Serial.printf ("play seq=%d milli=%lu\n", sequence_index, currentMillis);
          sequencer.Play (currentMillis, &note);
      }
    }
    sequence_index = (sequence_index + 1) % psequences.size ();
  }
  else
  {
    sequencer.Control (currentMillis);
  }

  if(sindex >= samplesize)  { analyzeSample(); }  //if samples array full, also checked in analyzeSample(), call sample analysis   
  //checkNote();  //turn off expired notes 
  //checkControl();
  
  if (currentMillis - threshold_last_millis > 15000)
  {
      
      if (threshold_evt < DESIRED_EVENT)
      {
        if (threshold > 0.001)
          threshold /= 1.4;
      }
      else
      {
        if (threshold < 10)
          threshold *= 1.4;
      }
      threshold_last_millis = currentMillis;
      threshold_evt = 0;    
      //Serial.println(threshold);
  }

  if (psequences[0]->getnbnotes () > 15)
    samplesize = SAMPLESIZE;
  previousMillis = currentMillis;   //manage time
  
}



void setNote(int value, int velocity, long duration, int notechannel)
{
  
        if (psequences[0]->getnbnotes() < psequences[0]->size () / 3)
          psequences[0]->addNote (currentMillis, value, velocity, duration, 1);
        else if (psequences[1]->getnbnotes() < psequences[1]->size () / 3)
          psequences[1]->addNote (currentMillis, value, velocity, duration, 2);
        /*else if (psequences[2]->getnbnotes() < psequences[2]->size () / 2)
          psequences[2]->addNote (currentMillis, value, velocity, duration, 3);*/
        else
        {
          uint16_t seq = currentMillis % psequences.size();
          psequences[seq]->setmode(CSequence::playmode::Learn);
          psequences[seq]->addNote (currentMillis, value, velocity, duration, seq + 1);
        }
      
    
  
}

void MIDIpanic()
{
  //120 - all sound off
  //123 - All Notes off
 // midiSerial(21, panicChannel, 123, 0); //123 kill all notes
  
  //brute force all notes Off
  for(byte i=1;i<128;i++) {
    delay(1); //don't choke on note offs!
    midiSerial(144, channel, i, 0); //clear notes on main channel
  }
  
  
}
#if 0
void midiSerial(int type, int channel, int data1, int data2) {

  
    //  Note type = 144 // noteon
    //  Control type = 176  // control change
    // remove MSBs on data
    data1 &= 0x7F;  //number
    data2 &= 0x7F;  //velocity
    
    byte statusbyte = (type | ((channel-1) & 0x0F));
    
    //Serial.write(statusbyte);
    //Serial.write(data1);
    //Serial.write(data2);
    switch (type)
    {
      case 128: // note off
        BLEMidiServer.noteOff(channel -1, data1, data2);
        break;
      case 144: // note on
        BLEMidiServer.noteOn(channel -1, data1, data2);
      break;
      case 176: // change note
        BLEMidiServer.controlChange(channel-1, data1, data2);
      break;
    }
  
}

#endif

//provide float map function
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//interrupt timing sample array
void sample()
{
  if(sindex < samplesize) {
    samples[sindex] = micros() - microseconds;
    microseconds = samples[sindex] + microseconds; //rebuild micros() value w/o recalling
    //micros() is very slow
    //try a higher precision counter
    //samples[sindex] = ((timer0_overflow_count << 8) + TCNT0) - microseconds;
    sindex += 1;
  }
  digitalWrite(LED, ((state) & 0x01) == 0 ? HIGH : LOW);
  state++;

    
}



void analyzeSample()
{
  
  //eating up memory, one long at a time!
  unsigned long averg = 0;
  unsigned long maxim = 0;
  unsigned long minim = 10000000;
  float stdevi = 0;
  unsigned long delta = 0;
  byte change = 0;
  
   digitalWrite(LED, ((state) & 0x01) == 0 ? HIGH : LOW);
  state++;
  if (sindex >= samplesize) { //array is full
    unsigned long sampanalysis[SAMPLESIZE];
    for (byte i=0; i < samplesize; i++){ 
      //skip first element in the array
      sampanalysis[i] = samples[i];  //load analysis table (due to volitle)
      //manual calculation
      if(sampanalysis[i] > maxim) { maxim = sampanalysis[i]; }
      if(sampanalysis[i] < minim) { minim = sampanalysis[i]; }
      averg += sampanalysis[i];
      
    }
    averg = averg / (samplesize);
    for (byte i = 0; i < samplesize; i++)
    {
      stdevi += (sampanalysis[i] - averg) * (sampanalysis[i] - averg) ;  //prep stdevi
    }

    //manual calculation
    
    stdevi = stdevi / (samplesize);
    if (stdevi < 1) { stdevi = 1.0; } //min stdevi of 1

    stdevi = sqrt(stdevi); //calculate stdevu
    
    delta = maxim - minim; 
    

    
    //Serial.printf("%ld %ld %ld %ld %f %f\r\n", minim, maxim, averg, delta, stdevi, stdevi * threshold);

    //**********perform change detection 
    if (delta > (stdevi * threshold)){
      change = 1;
      threshold_evt ++;
      
    }
    //*********
    
    if(change){// set note and control vector
        int dur = 200+(map(delta%127,1,127,100,2000)); //length of note
        int ramp = 3 + (dur%100) ; //control slide rate, min 25 (or 3 ;)
        

        //set scaling, root key, note
        int setnote = map(averg%127,1,127,noteMin,noteMax);  //derive note, min and max note
        setnote = scaleNote(setnote, scaleSelect, root);  //scale the note
        // setnote = setnote + root; // (apply root?)
        setNote(setnote, 100, dur, channel); 
        //derive control parameters and set    
        //setControl(controlNumber, controlMessage.value, delta%127, ramp); //set the ramp rate for the control
     }
     //reset array for next sample
    sindex = 0;
    
  }
}

int scaleSearch(int note, int scale[], int scalesize) {
 for(byte i=1;i<scalesize;i++) {
  if(note == scale[i]) { return note; }
  else { if(note < scale[i]) { return scale[i]; } } //highest scale value less than or equal to note
  //otherwise continue search
 }
 //didn't find note and didn't pass note value, uh oh!
 return 6;//give arbitrary value rather than fail
}


int scaleNote(int note, int scale[], int root) {
  //input note mod 12 for scaling, note/12 octave
  //search array for nearest note, return scaled*octave
  int scaled = note%12;
  int octave = note/12;
  int scalesize = (scale[0]);
  //search entire array and return closest scaled note
  scaled = scaleSearch(scaled, scale, scalesize);
  scaled = (scaled + (12 * octave)) + root; //apply octave and root
  return scaled;
}
