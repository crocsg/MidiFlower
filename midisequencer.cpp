#include "midisequencer.h"

CMidiSequencer::CMidiSequencer (uint32_t size)
{
    m_playingnotes.resize (size);
}
void CMidiSequencer::Play (uint32_t time, MIDImessage* midi)
{
    
    midiSerial(144, midi->channel, midi->value, midi->velocity);

    // find a room
    for (auto pMidi = m_playingnotes.begin(); pMidi != m_playingnotes.end (); pMidi++)
    {
        if (pMidi->velocity == 0)
        {
            *pMidi = *midi;
            pMidi->time = time;
            break;
        }
    }
}

void CMidiSequencer::Control (uint32_t time)
{
    for (auto pMidi = m_playingnotes.begin(); pMidi != m_playingnotes.end (); pMidi++)
    {
        if (time - pMidi->time > pMidi->duration && pMidi->velocity != 0)
        {
            midiSerial(128, pMidi->channel, pMidi->value, pMidi->velocity);
            pMidi->velocity = 0;
        }
    }
}

void CMidiSequencer::midiSerial(int type, int channel, int data1, int data2)
{
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
