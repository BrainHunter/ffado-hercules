/*
 * Copyright (C) 2005-2008 by Jonathan Woithe
 * Copyright (C) 2005-2008 by Pieter Palmers
 *
 * This file is part of FFADO
 * FFADO = Free FireWire (pro-)audio drivers for Linux
 *
 * FFADO is based upon FreeBoB.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __FFADO_MOTUTRANSMITSTREAMPROCESSOR__
#define __FFADO_MOTUTRANSMITSTREAMPROCESSOR__

/**
 * This class implements MOTU based streaming
 */

#include "debugmodule/debugmodule.h"

#include "../generic/StreamProcessor.h"
#include "../util/cip.h"

namespace Streaming {

class Port;
class MotuAudioPort;
class MotuMidiPort;
/*!
\brief The Base Class for an MOTU transmit stream processor

 This class implements a TransmitStreamProcessor that multiplexes Ports
 into MOTU streams.

*/
class MotuTransmitStreamProcessor
    : public StreamProcessor
{

public:
    /**
     * Create a MOTU transmit StreamProcessor
     */
    MotuTransmitStreamProcessor(FFADODevice &parent, unsigned int event_size);
    virtual ~MotuTransmitStreamProcessor() {};

    enum eChildReturnValue generatePacketHeader(unsigned char *data, unsigned int *length,
                                                unsigned char *tag, unsigned char *sy,
                                                uint32_t pkt_ctr);
    enum eChildReturnValue generatePacketData(unsigned char *data, unsigned int *length);
    enum eChildReturnValue generateEmptyPacketHeader(unsigned char *data, unsigned int *length,
                                                     unsigned char *tag, unsigned char *sy,
                                                     uint32_t pkt_ctr);
    enum eChildReturnValue generateEmptyPacketData(unsigned char *data, unsigned int *length);
    enum eChildReturnValue generateSilentPacketHeader(unsigned char *data, unsigned int *length,
                                                      unsigned char *tag, unsigned char *sy,
                                                      uint32_t pkt_ctr);
    enum eChildReturnValue generateSilentPacketData(unsigned char *data, unsigned int *length);
    virtual bool prepareChild();

public:
    virtual unsigned int getEventSize() 
                {return m_event_size;};
    virtual unsigned int getMaxPacketSize();
    virtual unsigned int getEventsPerFrame() 
                    { return 1; };
    virtual unsigned int getNominalFramesPerPacket();

protected:
    bool processWriteBlock(char *data, unsigned int nevents, unsigned int offset);
    bool transmitSilenceBlock(char *data, unsigned int nevents, unsigned int offset);

private:
    unsigned int fillNoDataPacketHeader(quadlet_t *data, unsigned int* length);
    unsigned int fillDataPacketHeader(quadlet_t *data, unsigned int* length, uint32_t ts);

    int transmitBlock(char *data, unsigned int nevents,
                        unsigned int offset);

    bool encodePacketPorts(quadlet_t *data, unsigned int nevents,
                           unsigned int dbc);

    int encodePortToMotuEvents(MotuAudioPort *, quadlet_t *data,
                                unsigned int offset, unsigned int nevents);
    int encodeSilencePortToMotuEvents(MotuAudioPort *, quadlet_t *data,
                                unsigned int offset, unsigned int nevents);

    int encodePortToMotuMidiEvents(
                       MotuMidiPort *p, quadlet_t *data,
                       unsigned int offset, unsigned int nevents);
    int encodeSilencePortToMotuMidiEvents(
                       MotuMidiPort *p, quadlet_t *data,
                       unsigned int offset, unsigned int nevents);

    /*
     * An iso packet mostly consists of multiple events.  m_event_size
     * is the size of a single 'event' in bytes.
     */
    unsigned int m_event_size;

    // To save time in the fast path, the number of pad bytes is stored
    // explicitly.
    unsigned int m_event_pad_bytes;

    signed int m_motu_model;

    // Keep track of transmission data block count
    unsigned int m_tx_dbc;

    // A simple circular buffer for outgoing MIDI data to allow
    // a rate control to be implemented on the data to suit the MOTU
    // devices.  Note that this buffer's size is forced to be a power
    // of 2 to allow for buffer manipulation optimisations.
    #define MIDIBUFFER_SIZE_EXP 10
    #define MIDIBUFFER_SIZE     (1<<MIDIBUFFER_SIZE_EXP)
    unsigned int midibuffer[MIDIBUFFER_SIZE];
    unsigned int mb_head, mb_tail;
    unsigned int midi_lock;
    unsigned int midi_tx_period; /* Measured in audio clock periods */
};

} // end of namespace Streaming

#endif /* __FFADO_MOTUTRANSMITSTREAMPROCESSOR__ */

