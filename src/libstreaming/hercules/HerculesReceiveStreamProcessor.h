/*
 * Copyright (C) 2005-2008 by Olivier Cloirec
 * Copyright (C) 2005-2008 by Pieter Palmers
 *
 * This file is part of FFADO
 * FFADO = Free Firewire (pro-)audio drivers for linux
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

#ifndef __FFADO_HERCULESRECEIVESTREAMPROCESSOR__
#define __FFADO_HERCULESRECEIVESTREAMPROCESSOR__

/**
 * This class implements HERCULES streaming
 */

#include "debugmodule/debugmodule.h"

#include "../generic/StreamProcessor.h"
#include "../util/cip.h"

namespace Streaming {

#define HERCULESFW_MAX_MIXBUSES        4
#define HERCULESFW_MAX_MIXBUS_CHANNELS 20

#define HERCULES_CHANNEL_NORMAL        0x00
#define HERCULES_CHANNEL_MUTE          0x01
#define HERCULES_CHANNEL_SOLO          0x02
#define HERCULES_CHANNEL_PAIRED        0x08

#define HERCULES_CHANNEL_PAN_LEFT      0x00
#define HERCULES_CHANNEL_PAN_RIGHT     0x80
#define HERCULES_CHANNEL_PAN_CENTRE    0x40

#define HERCULES_DEST_DISABLED         0x00
#define HERCULES_DEST_HEADPHONE        0x01
#define HERCULES_DEST_ANALOG1_2        0x02
#define HERCULES_DEST_ANALOG3_4        0x03
#define HERCULES_DEST_ANALOG5_6        0x04
#define HERCULES_DEST_ANALOG7_8        0x05
#define HERCULES_DEST_AESEBU           0x06
#define HERCULES_DEST_SPDIF            0x07
#define HERCULES_DEST_ADAT1_1_2        0x08
#define HERCULES_DEST_ADAT1_3_4        0x09
#define HERCULES_DEST_ADAT1_5_6        0x0a
#define HERCULES_DEST_ADAT1_7_8        0x0b
#define HERCULES_DEST_MUTE             0x10

enum EHerculesDevCtrlStatus {
    HERCULES_DEVCTRL_INVALID           = 0x00,
    HERCULES_DEVCTRL_SYNCING           = 0x01,
    HERCULES_DEVCTRL_INIT              = 0x02,
    HERCULES_DEVCTRL_VALID             = 0x03,
};

struct HerculesDevControls {
    unsigned int status;
    unsigned int input_6dB_boost;
    unsigned int input_ref_level;
    unsigned int input_20dB_pad;
    unsigned int input_gaintrim[HERCULESFW_MAX_MIXBUS_CHANNELS];
    unsigned char input_gaintrim_index;
    struct MixBus {
        unsigned char channel_gain[HERCULESFW_MAX_MIXBUS_CHANNELS];
        unsigned char channel_gain_index;
        unsigned char channel_pan[HERCULESFW_MAX_MIXBUS_CHANNELS];
        unsigned char channel_pan_index;
        unsigned char channel_control[HERCULESFW_MAX_MIXBUS_CHANNELS];
        unsigned char channel_control_index;
        unsigned char bus_gain;
        unsigned char bus_dest;
    } mixbus[HERCULESFW_MAX_MIXBUSES];
    unsigned char mixbus_index;
    unsigned char main_out_volume;
    unsigned char phones_volume;
    unsigned char phones_assign;
    unsigned char n_mixbuses;
    unsigned char n_channels;
};

enum HerculesCtrlKeys {
    HERCULES_KEY_MIDI            = 0x01,
    HERCULES_KEY_SEQ_SYNC        = 0x0c,
    HERCULES_KEY_CHANNEL_GAIN    = 0x14,
    HERCULES_KEY_CHANNEL_PAN     = 0x1c,
    HERCULES_KEY_CHANNEL_CTRL    = 0x24,
    HERCULES_KEY_MIXBUS_GAIN     = 0x2c,
    HERCULES_KEY_MIXBUS_DEST     = 0x34,
    HERCULES_KEY_MAINOUT_VOL     = 0x3c,
    HERCULES_KEY_PHONES_VOL      = 0x44,
    HERCULES_KEY_PHONES_DEST     = 0x4c,
    HERCULES_KEY_INPUT_6dB_BOOST = 0x6c,
    HERCULES_KEY_INPUT_REF_LEVEL = 0x74,
    HERCULES_KEY_MASK_MIDI       = 0x01,
};

enum EHerculesSeqSyncMixbuses {
    HERCULES_KEY_SEQ_SYNC_MIXBUS1 = 0x00,
    HERCULES_KEY_SEQ_SYNC_MIXBUS2 = 0x20,
    HERCULES_KEY_SEQ_SYNC_MIXBUS3 = 0x40,
    HERCULES_KEY_SEQ_SYNC_MIXBUS4 = 0x60,
};

class HerculesAudioPort;
class HerculesMidiPort;
/*!
 * \brief The Base Class for a HERCULES receive stream processor
 *
 * This class implements the outgoing stream processing for
 * hercules devices
 *
 */
class HerculesReceiveStreamProcessor
    : public StreamProcessor
{

public:
    /**
     * Create a HERCULES receive StreamProcessor
     * @param port 1394 port
     * @param dimension number of substreams in the ISO stream
     *                  (midi-muxed is only one stream)
     */
    HerculesReceiveStreamProcessor(FFADODevice &parent, unsigned int event_size);
    virtual ~HerculesReceiveStreamProcessor() {};

    enum eChildReturnValue processPacketHeader(unsigned char *data, unsigned int length,
                                               unsigned char tag, unsigned char sy,
                                               uint32_t pkt_ctr);
    enum eChildReturnValue processPacketData(unsigned char *data, unsigned int length);

    virtual bool prepareChild();

public:
    virtual unsigned int getEventSize() 
                {return m_event_size;};
    virtual unsigned int getMaxPacketSize();
    virtual unsigned int getEventsPerFrame() 
                    { return 1; };
    virtual unsigned int getNominalFramesPerPacket();

protected:
    bool processReadBlock(char *data, unsigned int nevents, unsigned int offset);

private:
    bool decodePacketPorts(quadlet_t *data, unsigned int nevents, unsigned int dbc);

    int decodeHerculesEventsToPort(HerculesAudioPort *, quadlet_t *data, unsigned int offset, unsigned int nevents);
    int decodeHerculesMidiEventsToPort(HerculesMidiPort *, quadlet_t *data, unsigned int offset, unsigned int nevents);
    int decodeHerculesCtrlEvents(char *data, unsigned int nevents);

    /*
     * An iso packet mostly consists of multiple events.  m_event_size
     * is the size of a single 'event' in bytes.
     */
    unsigned int m_event_size;

    struct HerculesDevControls m_devctrls;

    /* A small MIDI buffer to cover for the case where we need to span a
     * period.  This can only occur if more than one MIDI byte is sent per
     * packet, but this has been observed with some MOTUs (eg: 828MkII). 
     * Since the long-term average data rate must be close to the MIDI spec
     * since it's coming from a physical MIDI port this buffer doesn't have
     * to be particularly large.  The size is a power of 2 for optimisation
     * reasons.
     */
    #define RX_MIDIBUFFER_SIZE_EXP 6
    #define RX_MIDIBUFFER_SIZE     (1<<RX_MIDIBUFFER_SIZE_EXP)
    unsigned int midibuffer[RX_MIDIBUFFER_SIZE];
    unsigned mb_head, mb_tail;
};


} // end of namespace Streaming

#endif /* __FFADO_HERCULESRECEIVESTREAMPROCESSOR__ */

