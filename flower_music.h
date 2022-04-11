/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 S Godin (Climate Change Lab)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */


#ifndef __FLOWER_MUSIC_H
#define __FLOWER_MUSIC_H

#include "midisequencer.h"
#include "sequence.h"

class CMidiFlowerSequencer : public CMidiSequencer
{
public:
    CMidiFlowerSequencer (uint32_t size) : CMidiSequencer (size), m_sequence_index(0), m_sequence_time(0)
    {
        m_previousMillis = 0;
    }

    Loop (void);

    void register_track (CSequence* pseq)
    {
        m_ptrack.push_back (pseq);
    }
    std::vector<CSequence *> get_tracks ()
    {
        return m_ptracks;
    }

private:
    std::vector<CSequence *> m_ptracks;

    uint32_t m_sequence_time;
    uint16_t m_sequence_index;
    uint32_t m_previousMillis;
}
#endif
