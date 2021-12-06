#ifndef __SEQUENCE_H
#define __SEQUENCE_H

#include "Arduino.h"
#include "midinote.h"
#include <vector>


class CSequence
{
public:
    CSequence (size_t size, size_t block_size, uint32_t tempo, uint32_t noteratio, uint8_t* melody = NULL, size_t melodysize = 0);
    
    void addNote (uint32_t time, uint8_t value, uint8_t velocity, uint16_t duration, uint8_t notechannel);
    uint8_t play (uint32_t time, MIDImessage* mes);
    void check (uint32_t milli);

    typedef enum _playmode
    {
        Learn,
        Seq
    } playmode;

    playmode getmode () { return m_mode;}
    void setmode (playmode mode) {m_mode = mode;}
    size_t   getnbnotes () { return m_cntnote;}
    size_t   size() {return m_seq.size();}
private:
    uint8_t play_learn (uint32_t time, MIDImessage* mes);
    uint8_t play_seq (uint32_t time, MIDImessage* mes);

    
    
    std::vector<MIDImessage>    m_seq;
    std::vector<uint8_t>        m_melody;
    size_t                      m_size;
    uint32_t                    m_tempo;
    playmode                    m_mode;
    uint32_t                    m_noteratio;
    uint32_t                    m_lastplay;
    uint32_t                    m_cntnote;
};

#endif 
