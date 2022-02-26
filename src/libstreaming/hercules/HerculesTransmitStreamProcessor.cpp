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

#include "config.h"

#include "libutil/float_cast.h"

#include "HerculesTransmitStreamProcessor.h"
#include "HerculesPort.h"
#include "../StreamProcessorManager.h"
#include "devicemanager.h"

#include "libieee1394/ieee1394service.h"
#include "libieee1394/IsoHandlerManager.h"
#include "libieee1394/cycletimer.h"

#include "libutil/ByteSwap.h"

#include <cstring>
#include <assert.h>

// Set to 1 to enable the generation of a 1 kHz test tone in analog output 1.  Even with
// this defined to 1 the test tone will now only be produced if run with a non-zero 
// debug level.
#define TESTTONE 1

#if TESTTONE
#include <math.h>
#endif

/* Provide more intuitive access to GCC's branch predition built-ins */
#define likely(x)   __builtin_expect((x),1)
#define unlikely(x) __builtin_expect((x),0)

namespace Streaming
{

// A macro to extract specific bits from a native endian quadlet
#define get_bits(_d,_start,_len) (((_d)>>((_start)-(_len)+1)) & ((1<<(_len))-1))

/* transmit */
HerculesTransmitStreamProcessor::HerculesTransmitStreamProcessor(FFADODevice &parent, unsigned int event_size )
        : StreamProcessor(parent, ePT_Transmit )
        , m_event_size( event_size )
        , m_tx_dbc( 0 )
        , mb_head( 0 )
        , mb_tail( 0 )
        , midi_lock( 0 )
{
  int srate = m_Parent.getDeviceManager().getStreamProcessorManager().getNominalRate();
}

unsigned int
HerculesTransmitStreamProcessor::getMaxPacketSize() {
    return getNominalFramesPerPacket() * m_event_size + 8;
}

unsigned int
HerculesTransmitStreamProcessor::getNominalFramesPerPacket() {
    unsigned int n_events = 0;
    int framerate = m_Parent.getDeviceManager().getStreamProcessorManager().getNominalRate();
    switch (framerate)
    {
      case 32000:
        n_events = 5;
	break;
      case 44100:
        n_events = 6;
	break;
      case 48000:
        n_events = 7;
	break;
      case 88200:
        n_events = 12;
	break;
      case 96000:
        n_events = 13;
	break;
      default:
        // invalid framerate
        break;
    }
    return n_events;
}

enum StreamProcessor::eChildReturnValue
HerculesTransmitStreamProcessor::generatePacketHeader (
    unsigned char *data, unsigned int *length,
    unsigned char *tag, unsigned char *sy,
    uint32_t pkt_ctr )
{
    unsigned int cycle = CYCLE_TIMER_GET_CYCLES(pkt_ctr);

    // The number of events per packet expected by the Hercules is solely
    // dependent on the current sample rate.  An 'event' is one sample from
    // all channels plus possibly other midi and control data.
    signed n_events = getNominalFramesPerPacket();

    // Do housekeeping expected for all packets sent to the Hercules, even
    // for packets containing no audio data.
    *sy = 0x00;
    *tag = 1;      // All HERCULES packets have a CIP-like header
    *length = n_events * m_event_size + 8;

    signed int fc;
    uint64_t presentation_time;
    unsigned int presentation_cycle;
    int cycles_until_presentation;

    uint64_t transmit_at_time;
    unsigned int transmit_at_cycle;
    int cycles_until_transmit;

    debugOutput ( DEBUG_LEVEL_ULTRA_VERBOSE, "Try for cycle %d\n", cycle );
    // check whether the packet buffer has packets for us to send.
    // the base timestamp is the one of the next sample in the buffer
    ffado_timestamp_t ts_head_tmp;
    m_data_buffer->getBufferHeadTimestamp ( &ts_head_tmp, &fc ); // thread safe

    // the timestamp gives us the time at which we want the sample block
    // to be output by the device
    presentation_time = ( uint64_t ) ts_head_tmp;

    // now we calculate the time when we have to transmit the sample block
    transmit_at_time = substractTicks ( presentation_time, MOTU_TRANSMIT_TRANSFER_DELAY );

    // calculate the cycle this block should be presented in
    // (this is just a virtual calculation since at that time it should
    //  already be in the device's buffer)
    presentation_cycle = ( unsigned int ) ( TICKS_TO_CYCLES ( presentation_time ) );

    // calculate the cycle this block should be transmitted in
    transmit_at_cycle = ( unsigned int ) ( TICKS_TO_CYCLES ( transmit_at_time ) );

    // we can check whether this cycle is within the 'window' we have
    // to send this packet.
    // first calculate the number of cycles left before presentation time
    cycles_until_presentation = diffCycles ( presentation_cycle, cycle );

    // we can check whether this cycle is within the 'window' we have
    // to send this packet.
    // first calculate the number of cycles left before presentation time
    cycles_until_transmit = diffCycles ( transmit_at_cycle, cycle );

    // two different options:
    // 1) there are not enough frames for one packet
    //      => determine wether this is a problem, since we might still
    //         have some time to send it
    // 2) there are enough packets
    //      => determine whether we have to send them in this packet
    if ( fc < ( signed int ) getNominalFramesPerPacket() )
    {
        // not enough frames in the buffer,

        // we can still postpone the queueing of the packets
        // if we are far enough ahead of the presentation time
        if ( cycles_until_presentation <= MOTU_MIN_CYCLES_BEFORE_PRESENTATION )
        {
            debugOutput ( DEBUG_LEVEL_VERBOSE,
                        "Insufficient frames (P): N=%02d, CY=%04u, TC=%04u, CUT=%04d\n",
                        fc, cycle, transmit_at_cycle, cycles_until_transmit );
            // we are too late
            return eCRV_XRun;
        }
        else
        {
            debugOutput ( DEBUG_LEVEL_VERY_VERBOSE,
                        "Insufficient frames (NP): N=%02d, CY=%04u, TC=%04u, CUT=%04d\n",
                        fc, cycle, transmit_at_cycle, cycles_until_transmit );
            // there is still time left to send the packet
            // we want the system to give this packet another go at a later time instant
            return eCRV_Again;
        }
    }
    else
    {
        // there are enough frames, so check the time they are intended for
        // all frames have a certain 'time window' in which they can be sent
        // this corresponds to the range of the timestamp mechanism:
        // we can send a packet 15 cycles in advance of the 'presentation time'
        // in theory we can send the packet up till one cycle before the presentation time,
        // however this is not very smart.

        // There are 3 options:
        // 1) the frame block is too early
        //      => send an empty packet
        // 2) the frame block is within the window
        //      => send it
        // 3) the frame block is too late
        //      => discard (and raise xrun?)
        //         get next block of frames and repeat

        if(cycles_until_transmit < 0)
        {
            // we are too late
            debugOutput(DEBUG_LEVEL_VERBOSE,
                        "Too late: CY=%04u, TC=%04u, CUT=%04d, TSP=%011llu (%04u)\n",
                        cycle,
                        transmit_at_cycle, cycles_until_transmit,
                        presentation_time, (unsigned int)TICKS_TO_CYCLES(presentation_time) );

            // however, if we can send this sufficiently before the presentation
            // time, it could be harmless.
            // NOTE: dangerous since the device has no way of reporting that it didn't get
            //       this packet on time.
            if(cycles_until_presentation >= MOTU_MIN_CYCLES_BEFORE_PRESENTATION)
            {
                // we are not that late and can still try to transmit the packet
                m_tx_dbc++;
                m_last_timestamp = presentation_time;
                if (m_tx_dbc > 0xff)
                    m_tx_dbc -= 0x100;
                fillDataPacketHeader ( (quadlet_t *)data, length ,0);
                return eCRV_Packet;
            }
            else   // definitely too late
            {
                return eCRV_XRun;
            }
        }
        else if(cycles_until_transmit <= MOTU_MAX_CYCLES_TO_TRANSMIT_EARLY)
        {
            // it's time send the packet
            m_tx_dbc++;
            m_last_timestamp = presentation_time;
            if (m_tx_dbc > 0xff)
                m_tx_dbc -= 0x100;
            fillDataPacketHeader ( (quadlet_t *)data, length ,0);
            return eCRV_Packet;
        }
        else
        {
            debugOutput ( DEBUG_LEVEL_VERY_VERBOSE,
                        "Too early: CY=%04u, TC=%04u, CUT=%04d, TST=%011llu (%04u), TSP=%011llu (%04u)\n",
                        cycle,
                        transmit_at_cycle, cycles_until_transmit,
                        transmit_at_time, ( unsigned int ) TICKS_TO_CYCLES ( transmit_at_time ),
                        presentation_time, ( unsigned int ) TICKS_TO_CYCLES ( presentation_time ) );
#ifdef DEBUG
            if ( cycles_until_transmit > MOTU_MAX_CYCLES_TO_TRANSMIT_EARLY + 1 )
            {
                debugOutput ( DEBUG_LEVEL_VERY_VERBOSE,
                            "Way too early: CY=%04u, TC=%04u, CUT=%04d, TST=%011llu (%04u), TSP=%011llu (%04u)\n",
                            cycle,
                            transmit_at_cycle, cycles_until_transmit,
                            transmit_at_time, ( unsigned int ) TICKS_TO_CYCLES ( transmit_at_time ),
                            presentation_time, ( unsigned int ) TICKS_TO_CYCLES ( presentation_time ) );
            }
#endif
            // we are too early, send only an empty packet
            return eCRV_EmptyPacket;
        }
    }
    return eCRV_Invalid;
}

enum StreamProcessor::eChildReturnValue
HerculesTransmitStreamProcessor::generatePacketData (
    unsigned char *data, unsigned int *length)
{
    quadlet_t *quadlet = (quadlet_t *)data;
    quadlet += 2; // skip the header

    // The number of events per packet expected by the HERCULES is solely
    // dependent on the current sample rate.  An 'event' is one sample from
    // all channels plus possibly other midi and control data.
    signed n_events = getNominalFramesPerPacket();

    if (m_data_buffer->readFrames(n_events, (char *)(data + 8))) {
        float ticks_per_frame = m_Parent.getDeviceManager().getStreamProcessorManager().getSyncSource().getTicksPerFrame();

#if TESTTONE
        /* Now things are beginning to stabilise, make things easier for others by only playing
         * the test tone when run with a non-zero debug level.
         */
        if (getDebugLevel() > 0) {
            // FIXME: remove this hacked in 1 kHz test signal to
            // analog-1 when testing is complete.
            signed int i, int_tpf = lrintf(ticks_per_frame);
            unsigned char *sample = data+8;
            for (i=0; i<n_events; i++, sample+=m_event_size) {
                static signed int a_cx = 0;
                // Each sample is 3 bytes with MSB in lowest address (ie: 
                // network byte order).  After byte order swap, the 24-bit
                // MSB is in the second byte of val.
                signed int val = CondSwapToBus32(lrintf(0x7fffff*sin((1000.0*2.0*M_PI/24576000.0)*a_cx)));
                memcpy(sample,((char *)&val)+1,3);
                if ((a_cx+=int_tpf) >= 24576000) {
                    a_cx -= 24576000;
                }
            }
        }
#endif

        return eCRV_OK;
    }
    else return eCRV_XRun;
}

enum StreamProcessor::eChildReturnValue
HerculesTransmitStreamProcessor::generateEmptyPacketHeader (
    unsigned char *data, unsigned int *length,
    unsigned char *tag, unsigned char *sy,
    uint32_t pkt_ctr )
{
    debugOutput ( DEBUG_LEVEL_VERY_VERBOSE, "XMIT EMPTY: CY=%04u, TSP=%011llu (%04u)\n",
                CYCLE_TIMER_GET_CYCLES(pkt_ctr), m_last_timestamp, 
                ( unsigned int ) TICKS_TO_CYCLES ( m_last_timestamp ) );

    // Do housekeeping expected for all packets sent to the MOTU, even
    // for packets containing no audio data.
    *sy = 0x00;
    *tag = 1;      // All MOTU packets have a CIP-like header
    *length = 8;

    m_tx_dbc += fillNoDataPacketHeader ( (quadlet_t *)data, length );
    return eCRV_OK;
}

enum StreamProcessor::eChildReturnValue
HerculesTransmitStreamProcessor::generateEmptyPacketData (
    unsigned char *data, unsigned int *length)
{
    return eCRV_OK; // no need to do anything
}

enum StreamProcessor::eChildReturnValue
HerculesTransmitStreamProcessor::generateSilentPacketHeader (
    unsigned char *data, unsigned int *length,
    unsigned char *tag, unsigned char *sy,
    uint32_t pkt_ctr )
{
    unsigned int cycle = CYCLE_TIMER_GET_CYCLES(pkt_ctr);

    debugOutput( DEBUG_LEVEL_VERY_VERBOSE, "XMIT SILENT: CY=%04u, TSP=%011llu (%04u)\n",
                 cycle, m_last_timestamp,
                 ( unsigned int ) TICKS_TO_CYCLES ( m_last_timestamp ) );

    // A "silent" packet is identical to a regular data packet except all
    // audio data is set to zero.  The MOTU expects valid timestamps and
    // rate control in silent packets, so much of the timing logic from
    // generatePacketHeader() is needed here too - the main difference being
    // the source of the packet timestamp.

    // The number of events per packet expected by the MOTU is solely
    // dependent on the current sample rate.  An 'event' is one sample from
    // all channels plus possibly other midi and control data.
    signed n_events = getNominalFramesPerPacket();

    // Do housekeeping expected for all packets sent to the MOTU, even
    // for packets containing no audio data.
    *sy = 0x00;
    *tag = 1;      // All MOTU packets have a CIP-like header

    /* Assume the packet will have audio data.  If it turns out we need an empty packet
     * the length will be overridden by fillNoDataPacketHeader().
     */
    *length = n_events*m_event_size + 8;

    uint64_t presentation_time;
    unsigned int presentation_cycle;
    int cycles_until_presentation;
 
    uint64_t transmit_at_time;
    unsigned int transmit_at_cycle;
    int cycles_until_transmit;

    /* The sample buffer is not necessarily running when silent packets are
     * needed, so use m_last_timestamp (the timestamp of the previously sent
     * data packet) as the basis for the presentation time of the next
     * packet.  Since we're only writing zeros we don't have to deal with
     * buffer xruns.
     */
    float ticks_per_frame = m_Parent.getDeviceManager().getStreamProcessorManager().getSyncSource().getTicksPerFrame();
    presentation_time = addTicks(m_last_timestamp, (unsigned int)lrintf(n_events * ticks_per_frame));

    transmit_at_time = substractTicks(presentation_time, MOTU_TRANSMIT_TRANSFER_DELAY);
    presentation_cycle = (unsigned int)(TICKS_TO_CYCLES(presentation_time));
    transmit_at_cycle = (unsigned int)(TICKS_TO_CYCLES(transmit_at_time));
    cycles_until_presentation = diffCycles(presentation_cycle, cycle);
    cycles_until_transmit = diffCycles(transmit_at_cycle, cycle);

    if (cycles_until_transmit < 0)
    {
        if (cycles_until_presentation >= MOTU_MIN_CYCLES_BEFORE_PRESENTATION)
        {
            m_last_timestamp = presentation_time;
            m_tx_dbc++;
            if (m_tx_dbc > 0xff)
                m_tx_dbc -= 0x100;
            return eCRV_Packet;
        }
        else
        {
            return eCRV_XRun;
        }
    }
    else if (cycles_until_transmit <= MOTU_MAX_CYCLES_TO_TRANSMIT_EARLY)
    {
        m_last_timestamp = presentation_time;
        m_tx_dbc++;
        if (m_tx_dbc > 0xff)
            m_tx_dbc -= 0x100;
        return eCRV_Packet;
    }
    else
    {
        return eCRV_EmptyPacket;
    }
    return eCRV_Invalid;
}

enum StreamProcessor::eChildReturnValue
HerculesTransmitStreamProcessor::generateSilentPacketData (
    unsigned char *data, unsigned int *length )
{
    // Simply set all audio data to zero since that's what's meant by
    // a "silent" packet.  Note that m_event_size is in bytes for MOTU.

    quadlet_t *quadlet = (quadlet_t *)data;
    quadlet += 2; // skip the header

    // The number of events per packet expected by the MOTU is solely
    // dependent on the current sample rate.  An 'event' is one sample from
    // all channels plus possibly other midi and control data.
    signed n_events = getNominalFramesPerPacket();

    memset(quadlet, 0, n_events*m_event_size);

    return eCRV_OK;
}

unsigned int HerculesTransmitStreamProcessor::fillDataPacketHeader (
    quadlet_t *data, unsigned int* length,
    uint32_t ts )
{
    quadlet_t *quadlet = (quadlet_t *)data;
    // Size of a single data frame in quadlets.  For data sent TO the
    // Ultralite this is not strictly true (with m_event_size 52, dbs is set
    // to 13, even though data sent by the Ultralite uses 19 as one would
    // expect from a 52-byte event).  Even so, we'll run with the assumption
    // that a different dbs will be fine unless proven otherwise.
    unsigned dbs = m_event_size / 4;

    // The number of events per packet expected by the HERCULES is solely
    // dependent on the current sample rate.  An 'event' is one sample from
    // all channels plus possibly other midi and control data.
    signed n_events = getNominalFramesPerPacket();

    //debugOutput ( DEBUG_LEVEL_VERBOSE, "test1 dbc= %u\n", (unsigned int)m_tx_dbc);
    //debugOutput ( DEBUG_LEVEL_VERBOSE, "test1 dbs= %u", (unsigned int)dbs);

    // construct the packet CIP-like header.  Even if this is a data-less
    // packet the dbs field is still set as if there were data blocks
    // present.  For data-less packets the dbc is the same as the previously
    // transmitted block.
    *quadlet = CondSwapToBus32(((m_Parent.get1394Service().getLocalNodeId()&0x3f)<<24) | m_tx_dbc | (dbs<<16));
    quadlet++;
    *quadlet = CondSwapToBus32(0xa0000000);
    quadlet++;
    return n_events;
}

unsigned int HerculesTransmitStreamProcessor::fillNoDataPacketHeader (
    quadlet_t *data, unsigned int* length )
{
    quadlet_t *quadlet = (quadlet_t *)data;
    // Size of a single data frame in quadlets.  See comment in
    // fillDataPacketHeader() regarding the Ultralite.
    unsigned dbs = m_event_size / 4;
    dbs=0xdb;	// on empty packets dbs is allways 0xdb
    
    // construct the packet CIP-like header.  Even if this is a data-less
    // packet the dbs field is still set as if there were data blocks
    // present.  For data-less packets the dbc is the same as the previously
    // transmitted block.
    static unsigned char dbc= 0;	// Todo: Clean up here...
    dbc++;
    debugOutput ( DEBUG_LEVEL_VERBOSE, "test2 dbc= %u\n", (unsigned int)m_tx_dbc);
    //debugOutput ( DEBUG_LEVEL_VERBOSE, "test2 dbc= %u\n", (unsigned int)dbc);
    debugOutput ( DEBUG_LEVEL_VERBOSE, "test2 dbs= %u\n", (unsigned int)dbs);
    
    
    //*quadlet = CondSwapToBus32(((m_Parent.get1394Service().getLocalNodeId()&0x3f)<<24) | dbc | (dbs<<16));
     
    
    *quadlet = CondSwapToBus32(((m_Parent.get1394Service().getLocalNodeId()&0x3f)<<24) | m_tx_dbc | (dbs<<16));
    // debugOutput ( DEBUG_LEVEL_VERBOSE, "test2 quadlet= 0x%08x\n", (unsigned int)*quadlet);   
    
    quadlet++;
    *quadlet = CondSwapToBus32(0xa0000000);
    quadlet++;
    *length = 8;
    return 0;
}

bool HerculesTransmitStreamProcessor::prepareChild()
{
    debugOutput ( DEBUG_LEVEL_VERBOSE, "Preparing (%p)...\n", this );
    return true;
}

/*
* compose the event streams for the packets from the port buffers
*/
bool HerculesTransmitStreamProcessor::processWriteBlock(char *data,
                       unsigned int nevents, unsigned int offset) {
    bool no_problem=true;
    unsigned int i;

    // Start with MIDI and control streams all zeroed.  Due to the sparce nature
    // of these streams it is best to simply fill them in on an as-needs basis.
    for (i=0; i<nevents; i++) {
        memset(data+4+i*m_event_size, 0x00, 6);
    }

    for ( PortVectorIterator it = m_Ports.begin();
      it != m_Ports.end();
      ++it ) {
        // If this port is disabled, unconditionally send it silence.
        if((*it)->isDisabled()) {
          if (encodeSilencePortToHerculesEvents(static_cast<HerculesAudioPort *>(*it), (quadlet_t *)data, offset, nevents)) {
            debugWarning("Could not encode silence for disabled port %s to Hercules events\n",(*it)->getName().c_str());
            // Don't treat this as a fatal error at this point
          }
          continue;
        }

        Port *port=(*it);

        switch(port->getPortType()) {

        case Port::E_Audio:
            if (encodePortToHerculesEvents(static_cast<HerculesAudioPort *>(*it), (quadlet_t *)data, offset, nevents)) {
                debugWarning("Could not encode port %s to Hercules events\n",(*it)->getName().c_str());
                no_problem=false;
            }
            break;
        case Port::E_Midi:
             if (encodePortToHerculesMidiEvents(static_cast<HerculesMidiPort *>(*it), (quadlet_t *)data, offset, nevents)) {
                 debugWarning("Could not encode port %s to Midi events\n",(*it)->getName().c_str());
                 no_problem=false;
             }
            break;
        default: // ignore
            break;
        }
    }
    return no_problem;
}

bool
HerculesTransmitStreamProcessor::transmitSilenceBlock(char *data,
                       unsigned int nevents, unsigned int offset) {
    // This is the same as the non-silence version, except that is
    // doesn't read from the port buffers.
    bool no_problem = true;
    for ( PortVectorIterator it = m_Ports.begin();
      it != m_Ports.end();
      ++it ) {
        Port *port=(*it);

        switch(port->getPortType()) {

        case Port::E_Audio:
            if (encodeSilencePortToHerculesEvents(static_cast<HerculesAudioPort *>(*it), (quadlet_t *)data, offset, nevents)) {
                debugWarning("Could not encode port %s to MBLA events\n",(*it)->getName().c_str());
                no_problem = false;
            }
            break;
        case Port::E_Midi:
            if (encodeSilencePortToHerculesMidiEvents(static_cast<HerculesMidiPort *>(*it), (quadlet_t *)data, offset, nevents)) {
                debugWarning("Could not encode port %s to Midi events\n",(*it)->getName().c_str());
                no_problem = false;
            }
            break;
        default: // ignore
            break;
        }
    }
    return no_problem;
}

int HerculesTransmitStreamProcessor::encodePortToHerculesEvents(HerculesAudioPort *p, quadlet_t *data,
                       unsigned int offset, unsigned int nevents) {
// Encodes nevents worth of data from the given port into the given buffer.  The
// format of the buffer is precisely that which will be sent to the MOTU.
// The basic idea:
//   iterate over the ports
//     * get port buffer address
//     * loop over events
//         - pick right sample in event based upon PortInfo
//         - convert sample from Port format (E_Int24, E_Float, ..) to MOTU
//           native format
//
// We include the ability to start the transfer from the given offset within
// the port (expressed in frames) so the 'efficient' transfer method can be
// utilised.

    unsigned int j = 0;

    // Use char here since the target address won't necessarily be
    // aligned; use of an unaligned quadlet_t may cause issues on certain
    // architectures.  Besides, the target (data going directly to the MOTU)
    // isn't structured in quadlets anyway; it mainly consists of packed
    // 24-bit integers.
    unsigned char *target;
    target = (unsigned char *)data + p->getPosition();

    switch(m_StreamProcessorManager.getAudioDataType()) {
        default:
        case StreamProcessorManager::eADT_Int24:
            {
                quadlet_t *buffer=(quadlet_t *)(p->getBufferAddress());

                assert(nevents + offset <= p->getBufferSize());

                // Offset is in frames, but each port is only a single
                // channel, so the number of frames is the same as the
                // number of quadlets to offset (assuming the port buffer
                // uses one quadlet per sample, which is the case currently).
                buffer+=offset;

                for(j = 0; j < nevents; j += 1) { // Decode nsamples
                    *target = (*buffer >> 16) & 0xff;
                    *(target+1) = (*buffer >> 8) & 0xff;
                    *(target+2) = (*buffer) & 0xff;

                    buffer++;
                    target+=m_event_size;
                }
            }
            break;
        case StreamProcessorManager::eADT_Float:
            {
                const float multiplier = (float)(0x7FFFFF);
                float *buffer=(float *)(p->getBufferAddress());

                assert(nevents + offset <= p->getBufferSize());

                buffer+=offset;

                for(j = 0; j < nevents; j += 1) { // decode max nsamples
                    float in = *buffer;
#if MOTU_CLIP_FLOATS
                    if (unlikely(in > 1.0)) in = 1.0;
                    if (unlikely(in < -1.0)) in = -1.0;
#endif
                    unsigned int v = lrintf(in * multiplier);
                    *target = (v >> 16) & 0xff;
                    *(target+1) = (v >> 8) & 0xff;
                    *(target+2) = v & 0xff;

                    buffer++;
                    target+=m_event_size;
                }
            }
            break;
    }

    return 0;
}

int HerculesTransmitStreamProcessor::encodeSilencePortToHerculesEvents(HerculesAudioPort *p, quadlet_t *data,
                       unsigned int offset, unsigned int nevents) {
    unsigned int j=0;
    unsigned char *target = (unsigned char *)data + p->getPosition();

    switch (m_StreamProcessorManager.getAudioDataType()) {
    default:
        case StreamProcessorManager::eADT_Int24:
        case StreamProcessorManager::eADT_Float:
        for (j = 0; j < nevents; j++) {
            *target = *(target+1) = *(target+2) = 0;
            target += m_event_size;
        }
        break;
    }

    return 0;
}

int HerculesTransmitStreamProcessor::encodePortToHerculesMidiEvents(
                       HerculesMidiPort *p, quadlet_t *data,
                       unsigned int offset, unsigned int nevents) {

    unsigned int j;
    quadlet_t *src = (quadlet_t *)p->getBufferAddress();
    src += offset;
    unsigned char *target = (unsigned char *)data + p->getPosition();

    // Send a MIDI byte if there is one to send.  MOTU MIDI data is sent using
    // a 3-byte sequence within a frame starting at the port's position.
    // A non-zero MSB indicates there is MIDI data to send.

    for (j=0; j<nevents; j++, src++, target+=m_event_size) {
        if (midi_lock)
            midi_lock--;

        /* FFADO's MIDI subsystem dictates that at the most there will be one
         * MIDI byte every 8th's sample, making a MIDI byte "unlikely".
         */
        if (unlikely(*src & 0xff000000)) { 
            /* A MIDI byte is ready to send - buffer it */
            midibuffer[mb_head++] = *src;
            mb_head &= MIDIBUFFER_SIZE-1;
            if (unlikely(mb_head == mb_tail)) {
            /* Buffer overflow - dump oldest byte. */
            /* Ideally this would dump an entire MIDI message, but this is only
             * feasible if it's possible to determine the message size easily.
             */
                mb_tail = (mb_tail+1) & (MIDIBUFFER_SIZE-1);
                debugWarning("MOTU MIDI buffer overflow\n");
            }
            debugOutput(DEBUG_LEVEL_VERY_VERBOSE,"Buffered MIDI byte %d\n", *src & 0xff);
        }

        /* Send the MIDI byte at the tail of the buffer if enough time has elapsed
         * since the last MIDI byte was sent.  For most iterations through the loop
         * this condition will be false.
         */
        if (unlikely(mb_head!=mb_tail && !midi_lock)) {
            *(target) = 0x01;
            *(target+1) = 0x00;
            *(target+2) = midibuffer[mb_tail] & 0xff;
            debugOutput(DEBUG_LEVEL_VERY_VERBOSE,"Sent MIDI byte %d (j=%d)\n", midibuffer[mb_tail], j);
            mb_tail = (mb_tail+1) & (MIDIBUFFER_SIZE-1);
            midi_lock = midi_tx_period;
        }
    }

    return 0;
}

int HerculesTransmitStreamProcessor::encodeSilencePortToHerculesMidiEvents(
                       HerculesMidiPort *p, quadlet_t *data,
                       unsigned int offset, unsigned int nevents) {

    unsigned int j;
    unsigned char *target = (unsigned char *)data + p->getPosition();

    // For now, a "silent" MIDI event contains nothing but zeroes.  This
    // may have to change if we find this isn't for some reason appropriate.
    for (j=0; j<nevents; j++, target+=m_event_size) {
       memset(target, 0, 3);
    }

    return 0;
}

} // end of namespace Streaming
