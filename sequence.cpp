#include "sequence.h"


CSequence::CSequence (size_t size, size_t block_size, uint32_t tempo, uint32_t noteratio, uint8_t* melody, size_t melodysize)
{
    m_size = block_size;
    m_tempo = tempo;
    m_seq.resize (size);
    m_mode = CSequence::Learn;
    m_noteratio = noteratio;
    if (melody != NULL)
    {
      m_melody.resize (melodysize);
      memcpy (&m_melody[0], melody, melodysize);
    }
    m_lastplay = 0;
    m_cntnote = 0;
}

void CSequence::addNote (uint32_t time, uint8_t value, uint8_t velocity, uint16_t duration, uint8_t notechannel)
{
        
    if (velocity == 0)
        return;
    MIDImessage mes;
    mes.type = 0;
    mes.value = value;
    mes.velocity = velocity;
    mes.duration = duration;
    mes.channel = notechannel;

    size_t pos = (time / m_tempo) % m_seq.size ();
    if (m_seq[pos].velocity == 0)
        m_cntnote++;
    m_seq[pos] = mes;
    Serial.printf("addNote pos=%d, size=%d, ch=%d, nbnote=%d\n", pos, m_seq.size(), notechannel, m_cntnote);
    
    // delete a note if sequence is full
    if (m_cntnote > (m_seq.size() * 100) / m_noteratio)
    {
        for (auto it = m_seq.begin () + pos + 1; it != m_seq.end (); it++)
        {
            if (it->velocity == 1)
            {
                it->velocity = 0;
                m_cntnote--;
            }
        }
    }
}

uint8_t CSequence::play (uint32_t time, MIDImessage* mes)
{
    if (time - m_lastplay < m_tempo)
        return (0);

    m_lastplay = time;    
    if (m_mode == CSequence::Learn || m_melody.size () == 0)
        return play_learn (time, mes);
    else
        return play_seq(time, mes);    
}

uint8_t CSequence::play_learn (uint32_t time, MIDImessage* mes)
{
    size_t pos = (time / m_tempo) % m_seq.size ();
    
    if (m_seq[pos].velocity != 0)
    {
        //Serial.printf ("node pos=%d, ch=%d, val=%d, vel=%d\n", pos, m_seq[pos].channel, m_seq[pos].value, m_seq[pos].velocity);
        *mes = m_seq[pos];
        return (1);
    }
    return (0);
}

uint8_t CSequence::play_seq (uint32_t time, MIDImessage* mes)
{
    size_t pos = (time / m_tempo) % (m_melody.size () * m_size);
    size_t pmelody = pos / m_size;
    size_t pnote = pos % m_size;
    MIDImessage* ptnote  = &m_seq[m_melody[pmelody] * m_size + pnote];

    if (ptnote->velocity != 0)
    {

        //Serial.printf ("seq pos=%d, ch=%d, val=%d, vel=%d\n", pos, ptnote->channel, ptnote->value, ptnote->velocity);
        *mes = *ptnote;
        return (1);
    }
    return (0);
}
