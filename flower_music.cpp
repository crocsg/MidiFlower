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

#include "flower_music.h"
#include "flower_sensor.h"
#include "sequence.h"

// get the sequencer from midiflower.ino
extern CMidiFlowerSequencer sequencer;  

uint32_t last_sample_check = millis ();

//******************************
//set scaled values, sorted array, first element scale length
int scaleMajor[]  = {7,1, 3, 5, 6, 8, 10, 12};
int scaleDiaMinor[]  = {7,1, 3, 4, 6, 8, 9, 11};
int scaleIndian[]  = {7,1, 2, 2, 5, 6, 9, 11};
int scaleMinor[]  = {7,1, 3, 4, 6, 8, 9, 11};
int scaleChrom[] = {12,1,2,3,4,5,6,7,8,9,10,11,12};
int *scaleSelect = scaleMajor; //initialize scaling
int root = 0;       //initialize for root
int noteMin = 36;   //C2  - keyboard note minimum
int noteMax = 96;   //C7  - keyboard note maximum


// Flower sensor measure callback. compute a note and pass it to midi sequencer
// min    min value
// max    max value
// averg  average
// delta  max - min
// stdevi standard deviation
// stdevical standard deviation * threshold
void BuildNoteFromMeasure (uint32_t currentmillis, uint32_t min, uint32_t max, uint32_t averg, uint32_t delta, float stdevi, float stdevical)
{
    int dur = 200+(map(delta%127,1,127,100,2000));      //length of note derived from delta
    int ramp = 3 + (dur%100) ; //control slide rate, min 25 (or 3 ;)
     

    //set scaling, root key, note
    int note = map(averg%127,1,127,noteMin,noteMax);    //derive note from average measure
    note = scaleNote(note, scaleSelect, root);          //scale the note (force the note in selected scale)
    
    setNote(currentmillis, note, 100, dur);                            // add the note in some sequencer channels
}

int scaleSearch(int note, int scale[], int scalesize) 
{
 for(uint8_t i=1;i<scalesize;i++) {
  if(note == scale[i]) { return note; }
  else { if(note < scale[i]) { return scale[i]; } } //highest scale value less than or equal to note
  //otherwise continue search
 }
 //didn't find note and didn't pass note value, uh oh!
 return 6;//give arbitrary value rather than fail
}


int scaleNote(int note, int scale[], int root) 
{
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

void setNote(uint32_t currentMillis, int value, int velocity, long duration)
{
    Serial.printf ("add note time =%lu\n", currentMillis);
  // if first track is not 33% full add the note to it
  if (sequencer.get_track_nbnote(0) < sequencer.get_track_size (0) / 3)
    sequencer.addNote(0, currentMillis, value, velocity, duration, 1);
  // if second track is not full at 33% add the note to it
  else if (sequencer.get_track_nbnote(1) < sequencer.get_track_size (1) / 3)
    sequencer.addNote(1, currentMillis, value, velocity, duration, 2);
  else
  {
    // find a track and add the note
    uint16_t seq = currentMillis % sequencer.get_nbtracks ();
    
    sequencer.addNote(seq, currentMillis, value, velocity, duration, seq + 1);
  }
}

void ControlMusic (void)
{
  // fade off music when the is no measure
  uint32_t currentMillis =  millis ();  // get time in ms

  if (currentMillis - last_sample_check >= 5000)
  {
    uint32_t last_samples = flower_sensor_get_last_sample_time_ms ();   // get last mesure time
    std::vector<CSequence *>& psequences = sequencer.get_tracks ();     // get tracks

    if (currentMillis - last_samples > 15000)
    {
        //no measure for 15s 
        
        psequences[2]->mute (80);   // track3 volume at 80%
        psequences[3]->clear ();    // clear track 4
    }
    if (currentMillis - last_samples > 30000)
    {
        //no measure for 30s 
        psequences[0]->mute (75);   // track1 volume at 75%
        psequences[1]->clear ();    // clear track
        psequences[2]->clear ();    // clear track
        psequences[3]->clear ();    // clear track
    }
    if (currentMillis - last_samples > 60000)
    {
        //no measure for 60s 
        psequences[0]->clear ();    // clear track
    }

    last_sample_check = currentMillis;
  }
}