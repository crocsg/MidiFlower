/*
 MIT License

Copyright (c) 2022 S Godin (Climate Change Lab)

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


#ifndef __SEQUENCE_H
#define __SEQUENCE_H

#include "Arduino.h"
#include "midinote.h"
#include <vector>


class CSequence
{
public:
    CSequence (size_t size, uint32_t tempo, uint32_t noteratio, uint8_t* melody = NULL, size_t melodysize = 0);
    
    void addNote (uint32_t time, uint8_t value, uint8_t velocity, uint16_t duration, uint8_t notechannel);
    uint8_t play (uint32_t time, MIDImessage* mes);
    void clear (void);
    void mute (int vel);
    
    size_t   getnbnotes () { return m_cntnote;}
    size_t   size() {return m_seq.size();}
private:
    uint8_t play_seq (uint32_t time, MIDImessage* mes);
    

    
    
    std::vector<MIDImessage>    m_seq;
    std::vector<uint8_t>        m_melody;
    size_t                      m_size;
    uint32_t                    m_tempo;
    uint32_t                    m_noteratio;
    uint32_t                    m_lastplay;
    uint32_t                    m_cntnote;
};

#endif 
