/*
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

#include "config.h"

#include "AmdtpTransmitStreamProcessor.h"
#include "AmdtpPort.h"
#include "../StreamProcessorManager.h"
#include "devicemanager.h"

#include "libutil/Time.h"
#include "libutil/float_cast.h"

#include "libieee1394/ieee1394service.h"
#include "libieee1394/IsoHandlerManager.h"
#include "libieee1394/cycletimer.h"

#include "libutil/ByteSwap.h"
#include <assert.h>
#include <cstring>

#define likely(x)   __builtin_expect((x),1)
#define unlikely(x) __builtin_expect((x),0)

#define AMDTP_FLOAT_MULTIPLIER (1.0f * ((1<<23) - 1))
namespace Streaming
{

/* transmit */
AmdtpTransmitStreamProcessor::AmdtpTransmitStreamProcessor(FFADODevice &parent, int dimension)
        : StreamProcessor(parent, ePT_Transmit)
        , m_dimension( dimension )
        , m_dbc( 0 )
#if AMDTP_ALLOW_PAYLOAD_IN_NODATA_XMIT
        , m_send_nodata_payload ( AMDTP_SEND_PAYLOAD_IN_NODATA_XMIT_BY_DEFAULT )
#endif
        , m_max_cycles_to_transmit_early ( AMDTP_MAX_CYCLES_TO_TRANSMIT_EARLY )
        , m_transmit_transfer_delay ( AMDTP_TRANSMIT_TRANSFER_DELAY )
        , m_min_cycles_before_presentation ( AMDTP_MIN_CYCLES_BEFORE_PRESENTATION )
        , m_nb_audio_ports( 0 )
        , m_nb_midi_ports( 0 )
{}

enum StreamProcessor::eChildReturnValue
AmdtpTransmitStreamProcessor::generatePacketHeader (
    unsigned char *data, unsigned int *length,
    unsigned char *tag, unsigned char *sy,
    uint32_t pkt_ctr )
{
    __builtin_prefetch(data, 1, 0); // prefetch events for write, no temporal locality
    struct iec61883_packet *packet = (struct iec61883_packet *)data;
    /* Our node ID can change after a bus reset, so it is best to fetch
    * our node ID for each packet. */
    packet->sid = m_local_node_id;

    packet->eoh0 = 0;
    packet->dbs = m_dimension;
    packet->fn = 0;
    packet->qpc = 0;
    packet->sph = 0;
    packet->reserved = 0;
    packet->dbc = m_dbc;
    packet->eoh1 = 2;
    packet->fmt = IEC61883_FMT_AMDTP;

    *tag = IEC61883_TAG_WITH_CIP;
    *sy = 0;

    signed int fc;
    uint64_t presentation_time;
    unsigned int presentation_cycle;
    int cycles_until_presentation;

    uint64_t transmit_at_time;
    unsigned int transmit_at_cycle;
    int cycles_until_transmit;

    debugOutputExtreme( DEBUG_LEVEL_ULTRA_VERBOSE,
                        "Try for cycle %d\n", (int) CYCLE_TIMER_GET_CYCLES(pkt_ctr) );
    // check whether the packet buffer has packets for us to send.
    // the base timestamp is the one of the next sample in the buffer
    ffado_timestamp_t ts_head_tmp;
    m_data_buffer->getBufferHeadTimestamp( &ts_head_tmp, &fc ); // thread safe

    // the timestamp gives us the time at which we want the sample block
    // to be output by the device
    presentation_time = ( uint64_t ) ts_head_tmp;

    // now we calculate the time when we have to transmit the sample block
    transmit_at_time = substractTicks( presentation_time, m_transmit_transfer_delay );

    // calculate the cycle this block should be presented in
    // (this is just a virtual calculation since at that time it should
    //  already be in the device's buffer)
    presentation_cycle = ( unsigned int ) ( TICKS_TO_CYCLES ( presentation_time ) );

    // calculate the cycle this block should be transmitted in
    transmit_at_cycle = ( unsigned int ) ( TICKS_TO_CYCLES ( transmit_at_time ) );

    // we can check whether this cycle is within the 'window' we have
    // to send this packet.
    // first calculate the number of cycles left before presentation time
    cycles_until_presentation = diffCycles ( presentation_cycle, CYCLE_TIMER_GET_CYCLES(pkt_ctr) );

    // we can check whether this cycle is within the 'window' we have
    // to send this packet.
    // first calculate the number of cycles left before presentation time
    cycles_until_transmit = diffCycles ( transmit_at_cycle, CYCLE_TIMER_GET_CYCLES(pkt_ctr) );

    // two different options:
    // 1) there are not enough frames for one packet
    //      => determine wether this is a problem, since we might still
    //         have some time to send it
    // 2) there are enough packets
    //      => determine whether we have to send them in this packet
    if ( fc < ( signed int ) m_syt_interval )
    {
        // not enough frames in the buffer,

        // we can still postpone the queueing of the packets
        // if we are far enough ahead of the presentation time
        if ( cycles_until_presentation <= m_min_cycles_before_presentation )
        {
            debugOutput( DEBUG_LEVEL_NORMAL,
                         "Insufficient frames (P): N=%02d, CY=%04d, TC=%04u, CUT=%04d\n",
                         fc, (int)CYCLE_TIMER_GET_CYCLES(pkt_ctr), 
                         transmit_at_cycle, cycles_until_transmit );
            // we are too late
            return eCRV_XRun;
        }
        else
        {
            #if DEBUG_EXTREME
            unsigned int now_cycle = ( unsigned int ) ( TICKS_TO_CYCLES ( m_1394service.getCycleTimerTicks() ) );

            debugOutputExtreme(DEBUG_LEVEL_VERBOSE,
                               "Insufficient frames (NP): N=%02d, CY=%04d, TC=%04u, CUT=%04d, NOW=%04d\n",
                               fc, (int)CYCLE_TIMER_GET_CYCLES(pkt_ctr),
                               transmit_at_cycle, cycles_until_transmit, now_cycle );
            #endif

            // there is still time left to send the packet
            // we want the system to give this packet another go at a later time instant
            return eCRV_Again; // note that the raw1394 again system doesn't work as expected

            // we could wait here for a certain time before trying again. However, this
            // is not going to work since we then block the iterator thread, hence also 
            // the receiving code, meaning that we are not processing received packets,
            // and hence there is no progression in the number of frames available.

            // for example:
            // SleepRelativeUsec(125); // one cycle
            // goto try_block_of_frames;

            // or more advanced, calculate how many cycles we are ahead of 'now' and
            // base the sleep on that.

            // note that this requires that there is one thread for each IsoHandler,
            // otherwise we're in the deadlock described above.
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
                        "Too late: CY=%04d, TC=%04u, CUT=%04d, TSP=%011" PRIu64 " (%04u)\n",
                        (int)CYCLE_TIMER_GET_CYCLES(pkt_ctr),
                        transmit_at_cycle, cycles_until_transmit,
                        presentation_time, (unsigned int)TICKS_TO_CYCLES(presentation_time) );
            //debugShowBackLogLines(200);
            // however, if we can send this sufficiently before the presentation
            // time, it could be harmless.
            // NOTE: dangerous since the device has no way of reporting that it didn't get
            //       this packet on time.
            if(cycles_until_presentation >= m_min_cycles_before_presentation)
            {
                // we are not that late and can still try to transmit the packet
                m_dbc += fillDataPacketHeader(packet, length, presentation_time);
                m_last_timestamp = presentation_time;
                return (fc < (signed)(2*m_syt_interval) ? eCRV_Defer : eCRV_Packet);
            }
            else   // definitely too late
            {
                return eCRV_XRun;
            }
        }
        else if(cycles_until_transmit <= m_max_cycles_to_transmit_early)
        {
            // it's time send the packet
            m_dbc += fillDataPacketHeader(packet, length, presentation_time);
            m_last_timestamp = presentation_time;

            // for timestamp tracing
            debugOutputExtreme(DEBUG_LEVEL_VERY_VERBOSE,
                               "XMIT PKT: TSP= %011" PRIu64 " (%04u) (%04u) (%04u)\n",
                               presentation_time,
                               (unsigned int)CYCLE_TIMER_GET_CYCLES(pkt_ctr),
                               presentation_cycle, transmit_at_cycle);

            return (fc < (signed)(m_syt_interval) ? eCRV_Defer : eCRV_Packet);
        }
        else
        {
            debugOutputExtreme(DEBUG_LEVEL_VERY_VERBOSE,
                               "Too early: CY=%04u, TC=%04u, CUT=%04d, TST=%011" PRIu64 " (%04u), TSP=%011" PRId64 " (%04u)\n",
                               (int)CYCLE_TIMER_GET_CYCLES(pkt_ctr),
                               transmit_at_cycle, cycles_until_transmit,
                               transmit_at_time, (unsigned int)TICKS_TO_CYCLES(transmit_at_time),
                               presentation_time, (unsigned int)TICKS_TO_CYCLES(presentation_time));
#ifdef DEBUG
            if ( cycles_until_transmit > m_max_cycles_to_transmit_early + 1 )
            {
                debugOutputExtreme(DEBUG_LEVEL_VERY_VERBOSE,
                                   "Way too early: CY=%04u, TC=%04u, CUT=%04d, TST=%011" PRIu64 " (%04u), TSP=%011" PRId64 "(%04u)\n",
                                   (int)CYCLE_TIMER_GET_CYCLES(pkt_ctr),
                                   transmit_at_cycle, cycles_until_transmit,
                                   transmit_at_time, (unsigned int)TICKS_TO_CYCLES(transmit_at_time),
                                   presentation_time, (unsigned int)TICKS_TO_CYCLES(presentation_time));
            }
#endif
            // we are too early, send only an empty packet
            return eCRV_EmptyPacket;
        }
    }
    return eCRV_Invalid;
}

enum StreamProcessor::eChildReturnValue
AmdtpTransmitStreamProcessor::generatePacketData (
    unsigned char *data, unsigned int *length )
{
    if (m_data_buffer->readFrames(m_syt_interval, (char *)(data + 8)))
    {
        debugOutputExtreme(DEBUG_LEVEL_VERBOSE,
                           "XMIT DATA: TSP= %011" PRIu64 " (%04u)\n",
                           m_last_timestamp,
                           (unsigned int)TICKS_TO_CYCLES(m_last_timestamp));
        #if 0
        // debug code to output the packet content
        char tmpbuff[8192];
        int cnt=0;
        quadlet_t *tmp = (quadlet_t *)((char *)(data + 8));

        for(int i=0; i<m_syt_interval; i++) {
            cnt += snprintf(tmpbuff + cnt, 8192-cnt, "[%02d] ", i);
            for(int j=0; j<m_dimension; j++) {
                cnt += snprintf(tmpbuff + cnt, 8192-cnt, "%08X ", *tmp);
                tmp++;
            }
            cnt += snprintf(tmpbuff + cnt, 8192-cnt, "\n");
        }
        debugOutput(DEBUG_LEVEL_VERBOSE, "\n%s\n", tmpbuff);
        #endif
        return eCRV_OK;
    }
    else return eCRV_XRun;
}

enum StreamProcessor::eChildReturnValue
AmdtpTransmitStreamProcessor::generateSilentPacketHeader (
    unsigned char *data, unsigned int *length,
    unsigned char *tag, unsigned char *sy,
    uint32_t pkt_ctr )
{
    struct iec61883_packet *packet = ( struct iec61883_packet * ) data;
    debugOutputExtreme(DEBUG_LEVEL_ULTRA_VERBOSE,
                       "XMIT SILENT (cy %04d): TSP=%011" PRIu64 " (%04u)\n",
                       (int)CYCLE_TIMER_GET_CYCLES(pkt_ctr), m_last_timestamp,
                       (unsigned int)TICKS_TO_CYCLES(m_last_timestamp));

    packet->sid = m_local_node_id;

    packet->eoh0 = 0;
    packet->dbs = m_dimension;
    packet->fn = 0;
    packet->qpc = 0;
    packet->sph = 0;
    packet->reserved = 0;
    packet->dbc = m_dbc;
    packet->eoh1 = 2;
    packet->fmt = IEC61883_FMT_AMDTP;

    *tag = IEC61883_TAG_WITH_CIP;
    *sy = 0;

    m_dbc += fillNoDataPacketHeader(packet, length);
    return eCRV_Packet;
}

enum StreamProcessor::eChildReturnValue
AmdtpTransmitStreamProcessor::generateSilentPacketData (
    unsigned char *data, unsigned int *length )
{
    return eCRV_OK; // no need to do anything
}

enum StreamProcessor::eChildReturnValue
AmdtpTransmitStreamProcessor::generateEmptyPacketHeader (
    unsigned char *data, unsigned int *length,
    unsigned char *tag, unsigned char *sy,
    uint32_t pkt_ctr )
{
    struct iec61883_packet *packet = ( struct iec61883_packet * ) data;
    debugOutputExtreme(DEBUG_LEVEL_ULTRA_VERBOSE,
                       "XMIT EMPTY (cy %04d): TSP=%011" PRIu64 " (%04u)\n",
                       (int)CYCLE_TIMER_GET_CYCLES(pkt_ctr), m_last_timestamp,
                       (unsigned int)TICKS_TO_CYCLES(m_last_timestamp) );
    packet->sid = m_local_node_id;

    packet->eoh0 = 0;
    packet->dbs = m_dimension;
    packet->fn = 0;
    packet->qpc = 0;
    packet->sph = 0;
    packet->reserved = 0;
    packet->dbc = m_dbc;
    packet->eoh1 = 2;
    packet->fmt = IEC61883_FMT_AMDTP;

    *tag = IEC61883_TAG_WITH_CIP;
    *sy = 0;

    m_dbc += fillNoDataPacketHeader(packet, length);
    return eCRV_OK;
}

enum StreamProcessor::eChildReturnValue
AmdtpTransmitStreamProcessor::generateEmptyPacketData (
    unsigned char *data, unsigned int *length )
{
    return eCRV_OK; // no need to do anything
}

unsigned int AmdtpTransmitStreamProcessor::fillDataPacketHeader (
    struct iec61883_packet *packet, unsigned int* length,
    uint32_t ts )
{

    packet->fdf = m_fdf;

    // convert the timestamp to SYT format
    uint16_t timestamp_SYT = TICKS_TO_SYT ( ts );
    packet->syt = CondSwapToBus16 ( timestamp_SYT );

    // FIXME: use a precomputed value here
    *length = m_syt_interval*sizeof ( quadlet_t ) *m_dimension + 8;

    return m_syt_interval;
}

unsigned int AmdtpTransmitStreamProcessor::fillNoDataPacketHeader (
    struct iec61883_packet *packet, unsigned int* length )
{
    // no-data packets have syt=0xFFFF
    // and (can) have the usual amount of events as dummy data
    // DBC is not increased
    packet->fdf = IEC61883_FDF_NODATA;
    packet->syt = 0xffff;

#if AMDTP_ALLOW_PAYLOAD_IN_NODATA_XMIT
    if ( m_send_nodata_payload )
    { // no-data packets with payload (NOTE: DICE-II doesn't like that)
        *length = 2*sizeof ( quadlet_t ) + m_syt_interval * m_dimension * sizeof ( quadlet_t );
        return m_syt_interval;
    } else { // no-data packets without payload
        *length = 2*sizeof ( quadlet_t );
        return 0;
    }
#else
    // no-data packets without payload
    *length = 2*sizeof ( quadlet_t );
    return 0;
#endif
}

unsigned int
AmdtpTransmitStreamProcessor::getSytInterval() {
    switch (m_StreamProcessorManager.getNominalRate()) {
        case 32000:
        case 44100:
        case 48000:
            return 8;
        case 88200:
        case 96000:
            return 16;
        case 176400:
        case 192000:
            return 32;
        default:
            debugError("Unsupported rate: %d\n", m_StreamProcessorManager.getNominalRate());
            return 0;
    }
}

unsigned int
AmdtpTransmitStreamProcessor::getFDF() {
    switch (m_StreamProcessorManager.getNominalRate()) {
        case 32000: return IEC61883_FDF_SFC_32KHZ;
        case 44100: return IEC61883_FDF_SFC_44K1HZ;
        case 48000: return IEC61883_FDF_SFC_48KHZ;
        case 88200: return IEC61883_FDF_SFC_88K2HZ;
        case 96000: return IEC61883_FDF_SFC_96KHZ;
        case 176400: return IEC61883_FDF_SFC_176K4HZ;
        case 192000: return IEC61883_FDF_SFC_192KHZ;
        default:
            debugError("Unsupported rate: %d\n", m_StreamProcessorManager.getNominalRate());
            return 0;
    }
}

bool AmdtpTransmitStreamProcessor::prepareChild()
{
    debugOutput ( DEBUG_LEVEL_VERBOSE, "Preparing (%p)...\n", this );
    m_syt_interval = getSytInterval();
    m_fdf = getFDF();

    debugOutput ( DEBUG_LEVEL_VERBOSE, " SYT interval / FDF             : %d / %d\n", m_syt_interval, m_fdf );
#if AMDTP_ALLOW_PAYLOAD_IN_NODATA_XMIT
    debugOutput ( DEBUG_LEVEL_VERBOSE, " Send payload in No-Data packets: %s \n", m_send_nodata_payload?"Yes":"No" );
#endif
    debugOutput ( DEBUG_LEVEL_VERBOSE, " Max early transmit cycles      : %d\n", m_max_cycles_to_transmit_early );
    debugOutput ( DEBUG_LEVEL_VERBOSE, " Transfer delay                 : %d\n", m_transmit_transfer_delay );
    debugOutput ( DEBUG_LEVEL_VERBOSE, " Min cycles before presentation : %d\n", m_min_cycles_before_presentation );

    iec61883_cip_init (
        &m_cip_status,
        IEC61883_FMT_AMDTP,
        m_fdf,
        m_StreamProcessorManager.getNominalRate(),
        m_dimension,
        m_syt_interval );

    if (!initPortCache()) {
        debugError("Could not init port cache\n");
        return false;
    }

    return true;
}

/*
* compose the event streams for the packets from the port buffers
*/
bool AmdtpTransmitStreamProcessor::processWriteBlock ( char *data,
        unsigned int nevents, unsigned int offset )
{
    // update the variable parts of the cache
    updatePortCache();

    // encode audio data
    switch(m_StreamProcessorManager.getAudioDataType()) {
        case StreamProcessorManager::eADT_Int24:
            encodeAudioPortsInt24((quadlet_t *)data, offset, nevents);
            break;
        case StreamProcessorManager::eADT_Float:
            encodeAudioPortsFloat((quadlet_t *)data, offset, nevents);
            break;
    }

    // do midi ports
    encodeMidiPorts((quadlet_t *)data, offset, nevents);
    return true;
}

bool
AmdtpTransmitStreamProcessor::transmitSilenceBlock(
    char *data, unsigned int nevents, unsigned int offset)
{
    // no need to update the port cache when transmitting silence since
    // no dynamic values are used to do so.
    encodeAudioPortsSilence((quadlet_t *)data, offset, nevents);
    encodeMidiPortsSilence((quadlet_t *)data, offset, nevents);
    return true;
}

/**
 * @brief encodes all audio ports in the cache to events (silent data)
 * @param data 
 * @param offset 
 * @param nevents 
 */
void
AmdtpTransmitStreamProcessor::encodeAudioPortsSilence(quadlet_t *data,
                                                      unsigned int offset,
                                                      unsigned int nevents)
{
    unsigned int j;
    quadlet_t *target_event;
    int i;

    for (i = 0; i < m_nb_audio_ports; i++) {
        target_event = (quadlet_t *)(data + i);

        for (j = 0;j < nevents; j += 1)
        {
            *target_event = CONDSWAPTOBUS32_CONST(0x40000000);
            target_event += m_dimension;
        }
    }
}

#ifdef __SSE2__
#include <emmintrin.h>

// There's no obvious reason to warn about this anymore - jwoithe.
// #warning SSE2 build

/**
 * @brief mux all audio ports to events
 * @param data 
 * @param offset 
 * @param nevents 
 */
void
AmdtpTransmitStreamProcessor::encodeAudioPortsFloat(quadlet_t *data,
                                                    unsigned int offset,
                                                    unsigned int nevents)
{
    unsigned int j;
    quadlet_t *target_event;
    int i;

    float * client_buffers[4];
    float tmp_values[4] __attribute__ ((aligned (16)));
    uint32_t tmp_values_int[4] __attribute__ ((aligned (16)));

    // prepare the scratch buffer
    assert(m_scratch_buffer_size_bytes > nevents * 4);
    memset(m_scratch_buffer, 0, nevents * 4);

    const __m128i label = _mm_set_epi32 (0x40000000, 0x40000000, 0x40000000, 0x40000000);
    const __m128i mask = _mm_set_epi32 (0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF);
    const __m128 mult = _mm_set_ps(AMDTP_FLOAT_MULTIPLIER, AMDTP_FLOAT_MULTIPLIER, AMDTP_FLOAT_MULTIPLIER, AMDTP_FLOAT_MULTIPLIER);

#if AMDTP_CLIP_FLOATS
    const __m128 v_max = _mm_set_ps(1.0, 1.0, 1.0, 1.0);
    const __m128 v_min = _mm_set_ps(-1.0, -1.0, -1.0, -1.0);
#endif

    // this assumes that audio ports are sorted by position,
    // and that there are no gaps
    for (i = 0; i < ((int)m_nb_audio_ports)-4; i += 4) {
        struct _MBLA_port_cache *p;

        // get the port buffers
        for (j=0; j<4; j++) {
            p = &(m_audio_ports.at(i+j));
            if(likely(p->buffer && p->enabled)) {
                client_buffers[j] = (float *) p->buffer;
                client_buffers[j] += offset;
            } else {
                // if a port is disabled or has no valid
                // buffer, use the scratch buffer (all zero's)
                client_buffers[j] = (float *) m_scratch_buffer;
            }
        }

        // the base event for this position
        target_event = (quadlet_t *)(data + i);
        // process the events
        for (j=0;j < nevents; j += 1)
        {
            // read the values
            tmp_values[0] = *(client_buffers[0]);
            tmp_values[1] = *(client_buffers[1]);
            tmp_values[2] = *(client_buffers[2]);
            tmp_values[3] = *(client_buffers[3]);

            // now do the SSE based conversion/labeling
            __m128 v_float = *((__m128*)tmp_values);
            __m128i *target = (__m128i*)target_event;
            __m128i v_int;

            // clip
#if AMDTP_CLIP_FLOATS
            // do SSE clipping
            v_float = _mm_max_ps(v_float, v_min);
            v_float = _mm_min_ps(v_float, v_max);
#endif

            // multiply
            v_float = _mm_mul_ps(v_float, mult);
            // convert to signed integer
            v_int = _mm_cvttps_epi32( v_float );
            // mask
            v_int = _mm_and_si128( v_int, mask );
            // label it
            v_int = _mm_or_si128( v_int, label );

            // do endian conversion (SSE is always little endian)
            // do first swap
            v_int = _mm_or_si128( _mm_slli_epi16( v_int, 8 ), _mm_srli_epi16( v_int, 8 ) );
            // do second swap
            v_int = _mm_or_si128( _mm_slli_epi32( v_int, 16 ), _mm_srli_epi32( v_int, 16 ) );
            // store the packed int
            // (target misalignment is assumed since we don't know the m_dimension)
            _mm_storeu_si128 (target, v_int);

            // increment the buffer pointers
            client_buffers[0]++;
            client_buffers[1]++;
            client_buffers[2]++; 
            client_buffers[3]++;

            // go to next target event position
            target_event += m_dimension;
        }
    }

    // do remaining ports
    // NOTE: these can be time-SSE'd
    for (; i < (int)m_nb_audio_ports; i++) {
        struct _MBLA_port_cache &p = m_audio_ports.at(i);
        target_event = (quadlet_t *)(data + i);
#ifdef DEBUG
        assert(nevents + offset <= p.buffer_size );
#endif

        if(likely(p.buffer && p.enabled)) {
            float *buffer = (float *)(p.buffer);
            buffer += offset;
    
            for (j = 0;j < nevents; j += 4)
            {
                // read the values
                tmp_values[0] = *buffer;
                buffer++;
                tmp_values[1] = *buffer;
                buffer++;
                tmp_values[2] = *buffer;
                buffer++;
                tmp_values[3] = *buffer;
                buffer++;

                // now do the SSE based conversion/labeling
                __m128 v_float = *((__m128*)tmp_values);
                __m128i v_int;

#if AMDTP_CLIP_FLOATS
                // do SSE clipping
                v_float = _mm_max_ps(v_float, v_min);
                v_float = _mm_min_ps(v_float, v_max);
#endif
                // multiply
                v_float = _mm_mul_ps(v_float, mult);
                // convert to signed integer
                v_int = _mm_cvttps_epi32( v_float );
                // mask
                v_int = _mm_and_si128( v_int, mask );
                // label it
                v_int = _mm_or_si128( v_int, label );
    
                // do endian conversion (SSE is always little endian)
                // do first swap
                v_int = _mm_or_si128( _mm_slli_epi16( v_int, 8 ), _mm_srli_epi16( v_int, 8 ) );
                // do second swap
                v_int = _mm_or_si128( _mm_slli_epi32( v_int, 16 ), _mm_srli_epi32( v_int, 16 ) );

                // store the packed int
                _mm_store_si128 ((__m128i *)(&tmp_values_int), v_int);

                // increment the buffer pointers
                *target_event = tmp_values_int[0];
                target_event += m_dimension;
                *target_event = tmp_values_int[1];
                target_event += m_dimension;
                *target_event = tmp_values_int[2];
                target_event += m_dimension;
                *target_event = tmp_values_int[3];
                target_event += m_dimension;
            }

            // do the remainder of the events
            for(;j < nevents; j += 1) {
                float *in = (float *)buffer;
#if AMDTP_CLIP_FLOATS
                // clip directly to the value of a maxed event
                if(unlikely(*in > 1.0)) {
                    *target_event = CONDSWAPTOBUS32_CONST(0x407FFFFF);
                } else if(unlikely(*in < -1.0)) {
                    *target_event = CONDSWAPTOBUS32_CONST(0x40800001);
                } else {
                    float v = (*in) * AMDTP_FLOAT_MULTIPLIER;
                    unsigned int tmp = ((int) v);
                    tmp = ( tmp & 0x00FFFFFF ) | 0x40000000;
                    *target_event = CondSwapToBus32((quadlet_t)tmp);
                }
#else
                float v = (*in) * AMDTP_FLOAT_MULTIPLIER;
                unsigned int tmp = ((int) v);
                tmp = ( tmp & 0x00FFFFFF ) | 0x40000000;
                *target_event = CondSwapToBus32((quadlet_t)tmp);
#endif
                buffer++;
                target_event += m_dimension;
            }

        } else {
            for (j = 0;j < nevents; j += 1)
            {
                // hardcoded byte swapped
                *target_event = 0x00000040;
                target_event += m_dimension;
            }
        }
    }
}


/**
 * @brief mux all audio ports to events
 * @param data 
 * @param offset 
 * @param nevents 
 */
void
AmdtpTransmitStreamProcessor::encodeAudioPortsInt24(quadlet_t *data,
                                                    unsigned int offset,
                                                    unsigned int nevents)
{
    unsigned int j;
    quadlet_t *target_event;
    int i;

    uint32_t *client_buffers[4];
    uint32_t tmp_values[4] __attribute__ ((aligned (16)));

    // prepare the scratch buffer
    assert(m_scratch_buffer_size_bytes > nevents * 4);
    memset(m_scratch_buffer, 0, nevents * 4);

    const __m128i label = _mm_set_epi32 (0x40000000, 0x40000000, 0x40000000, 0x40000000);
    const __m128i mask  = _mm_set_epi32 (0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF, 0x00FFFFFF);

    // this assumes that audio ports are sorted by position,
    // and that there are no gaps
    for (i = 0; i < ((int)m_nb_audio_ports)-4; i += 4) {
        struct _MBLA_port_cache *p;

        // get the port buffers
        for (j=0; j<4; j++) {
            p = &(m_audio_ports.at(i+j));
            if(likely(p->buffer && p->enabled)) {
                client_buffers[j] = (uint32_t *) p->buffer;
                client_buffers[j] += offset;
            } else {
                // if a port is disabled or has no valid
                // buffer, use the scratch buffer (all zero's)
                client_buffers[j] = (uint32_t *) m_scratch_buffer;
            }
        }

        // the base event for this position
        target_event = (quadlet_t *)(data + i);

        // process the events
        for (j=0;j < nevents; j += 1)
        {
            // read the values
            tmp_values[0] = *(client_buffers[0]);
            tmp_values[1] = *(client_buffers[1]);
            tmp_values[2] = *(client_buffers[2]);
            tmp_values[3] = *(client_buffers[3]);

            // now do the SSE based conversion/labeling
            __m128i *target = (__m128i*)target_event;
            __m128i v_int = *((__m128i*)tmp_values);;

            // mask
            v_int = _mm_and_si128( v_int, mask );
            // label it
            v_int = _mm_or_si128( v_int, label );

            // do endian conversion (SSE is always little endian)
            // do first swap
            v_int = _mm_or_si128( _mm_slli_epi16( v_int, 8 ), _mm_srli_epi16( v_int, 8 ) );
            // do second swap
            v_int = _mm_or_si128( _mm_slli_epi32( v_int, 16 ), _mm_srli_epi32( v_int, 16 ) );

            // store the packed int
            // (target misalignment is assumed since we don't know the m_dimension)
            _mm_storeu_si128 (target, v_int);

            // increment the buffer pointers
            client_buffers[0]++;
            client_buffers[1]++;
            client_buffers[2]++; 
            client_buffers[3]++;

            // go to next target event position
            target_event += m_dimension;
        }
    }

    // do remaining ports
    // NOTE: these can be time-SSE'd
    for (; i < ((int)m_nb_audio_ports); i++) {
        struct _MBLA_port_cache &p = m_audio_ports.at(i);
        target_event = (quadlet_t *)(data + i);
#ifdef DEBUG
        assert(nevents + offset <= p.buffer_size );
#endif

        if(likely(p.buffer && p.enabled)) {
            uint32_t *buffer = (uint32_t *)(p.buffer);
            buffer += offset;
    
            for (j = 0;j < nevents; j += 4)
            {
                // read the values
                tmp_values[0] = *buffer;
                buffer++;
                tmp_values[1] = *buffer;
                buffer++;
                tmp_values[2] = *buffer;
                buffer++;
                tmp_values[3] = *buffer;
                buffer++;

                // now do the SSE based conversion/labeling
                __m128i v_int = *((__m128i*)tmp_values);;

                // mask
                v_int = _mm_and_si128( v_int, mask );
                // label it
                v_int = _mm_or_si128( v_int, label );

                // do endian conversion (SSE is always little endian)
                // do first swap
                v_int = _mm_or_si128( _mm_slli_epi16( v_int, 8 ), _mm_srli_epi16( v_int, 8 ) );
                // do second swap
                v_int = _mm_or_si128( _mm_slli_epi32( v_int, 16 ), _mm_srli_epi32( v_int, 16 ) );

                // store the packed int
                _mm_store_si128 ((__m128i *)(&tmp_values), v_int);

                // increment the buffer pointers
                *target_event = tmp_values[0];
                target_event += m_dimension;
                *target_event = tmp_values[1];
                target_event += m_dimension;
                *target_event = tmp_values[2];
                target_event += m_dimension;
                *target_event = tmp_values[3];
                target_event += m_dimension;
            }

            // do the remainder of the events
            for(;j < nevents; j += 1) {
                uint32_t in = (uint32_t)(*buffer);
                *target_event = CondSwapToBus32((quadlet_t)((in & 0x00FFFFFF) | 0x40000000));
                buffer++;
                target_event += m_dimension;
            }

        } else {
            for (j = 0;j < nevents; j += 1)
            {
                // hardcoded byte swapped
                *target_event = 0x00000040;
                target_event += m_dimension;
            }
        }
    }
}

#else

/**
 * @brief mux all audio ports to events
 * @param data 
 * @param offset 
 * @param nevents 
 */
void
AmdtpTransmitStreamProcessor::encodeAudioPortsInt24(quadlet_t *data,
                                                    unsigned int offset,
                                                    unsigned int nevents)
{
    unsigned int j;
    quadlet_t *target_event;
    int i;

    for (i = 0; i < m_nb_audio_ports; i++) {
        struct _MBLA_port_cache &p = m_audio_ports.at(i);
        target_event = (quadlet_t *)(data + i);
#ifdef DEBUG
        assert(nevents + offset <= p.buffer_size );
#endif

        if(likely(p.buffer && p.enabled)) {
            quadlet_t *buffer = (quadlet_t *)(p.buffer);
            buffer += offset;
    
            for (j = 0;j < nevents; j += 1)
            {
                uint32_t in = (uint32_t)(*buffer);
                *target_event = CondSwapToBus32((quadlet_t)((in & 0x00FFFFFF) | 0x40000000));
                buffer++;
                target_event += m_dimension;
            }
        } else {
            for (j = 0;j < nevents; j += 1)
            {
                *target_event = CONDSWAPTOBUS32_CONST(0x40000000);
                target_event += m_dimension;
            }
        }
    }
}

/**
 * @brief mux all audio ports to events
 * @param data 
 * @param offset 
 * @param nevents 
 */
void
AmdtpTransmitStreamProcessor::encodeAudioPortsFloat(quadlet_t *data,
                                                    unsigned int offset,
                                                    unsigned int nevents)
{
    unsigned int j;
    quadlet_t *target_event;
    int i;

    for (i = 0; i < m_nb_audio_ports; i++) {
        struct _MBLA_port_cache &p = m_audio_ports.at(i);
        target_event = (quadlet_t *)(data + i);
#ifdef DEBUG
        assert(nevents + offset <= p.buffer_size );
#endif

        if(likely(p.buffer && p.enabled)) {
            quadlet_t *buffer = (quadlet_t *)(p.buffer);
            buffer += offset;
    
            for (j = 0;j < nevents; j += 1)
            {
                float *in = (float *)buffer;
#if AMDTP_CLIP_FLOATS
                // clip directly to the value of a maxed event
                if(unlikely(*in > 1.0)) {
                    *target_event = CONDSWAPTOBUS32_CONST(0x407FFFFF);
                } else if(unlikely(*in < -1.0)) {
                    *target_event = CONDSWAPTOBUS32_CONST(0x40800001);
                } else {
                    float v = (*in) * AMDTP_FLOAT_MULTIPLIER;
                    unsigned int tmp = ((int) v);
                    tmp = ( tmp & 0x00FFFFFF ) | 0x40000000;
                    *target_event = CondSwapToBus32((quadlet_t)tmp);
                }
#else
                float v = (*in) * AMDTP_FLOAT_MULTIPLIER;
                unsigned int tmp = ((int) v);
                tmp = ( tmp & 0x00FFFFFF ) | 0x40000000;
                *target_event = CondSwapToBus32((quadlet_t)tmp);
#endif
                buffer++;
                target_event += m_dimension;
            }
        } else {
            for (j = 0;j < nevents; j += 1)
            {
                *target_event = CONDSWAPTOBUS32_CONST(0x40000000);
                target_event += m_dimension;
            }
        }
    }
}
#endif

/**
 * @brief encodes all midi ports in the cache to events (silence)
 * @param data 
 * @param offset 
 * @param nevents 
 */
void
AmdtpTransmitStreamProcessor::encodeMidiPortsSilence(quadlet_t *data,
                                                     unsigned int offset,
                                                     unsigned int nevents)
{
    quadlet_t *target_event;
    int i;
    unsigned int j;

    for (i = 0; i < m_nb_midi_ports; i++) {
        struct _MIDI_port_cache &p = m_midi_ports.at(i);

        for (j = p.location;j < nevents; j += 8) {
            target_event = (quadlet_t *) (data + ((j * m_dimension) + p.position));
            *target_event = CondSwapToBus32(IEC61883_AM824_SET_LABEL(0, IEC61883_AM824_LABEL_MIDI_NO_DATA));
        }
    }
}

/**
 * @brief encodes all midi ports in the cache to events
 * @param data 
 * @param offset 
 * @param nevents 
 */
void
AmdtpTransmitStreamProcessor::encodeMidiPorts(quadlet_t *data,
                                              unsigned int offset,
                                              unsigned int nevents)
{
    quadlet_t *target_event;
    int i;
    unsigned int j;

    for (i = 0; i < m_nb_midi_ports; i++) {
        struct _MIDI_port_cache &p = m_midi_ports.at(i);
        if (p.buffer && p.enabled) {
            uint32_t *buffer = (quadlet_t *)(p.buffer);
            buffer += offset;

            for (j = p.location;j < nevents; j += 8) {
                target_event = (quadlet_t *) (data + ((j * m_dimension) + p.position));

                if ( *buffer & 0xFF000000 )   // we can send a byte
                {
                    quadlet_t tmpval;
                    tmpval = ((*buffer)<<16) & 0x00FF0000;
                    tmpval = IEC61883_AM824_SET_LABEL(tmpval, IEC61883_AM824_LABEL_MIDI_1X);
                    *target_event = CondSwapToBus32(tmpval);

                    debugOutputExtreme( DEBUG_LEVEL_VERBOSE, "MIDI port %s, pos=%u, loc=%u, nevents=%u, dim=%d\n",
                               p.port->getName().c_str(), p.position, p.location, nevents, m_dimension );
                    debugOutputExtreme( DEBUG_LEVEL_VERBOSE, "base=%p, target=%p, value=%08X\n",
                               data, target_event, tmpval );
                } else {
                    // can't send a byte, either because there is no byte,
                    // or because this would exceed the maximum rate
                    // FIXME: this can be ifdef optimized since it's a constant
                    *target_event = CondSwapToBus32(IEC61883_AM824_SET_LABEL(0, IEC61883_AM824_LABEL_MIDI_NO_DATA));
                }
                buffer+=8;
            }
        } else {
            for (j = p.location;j < nevents; j += 8) {
                target_event = (quadlet_t *)(data + ((j * m_dimension) + p.position));
                __builtin_prefetch(target_event, 1, 0); // prefetch events for write, no temporal locality
                *target_event = CondSwapToBus32(IEC61883_AM824_SET_LABEL(0, IEC61883_AM824_LABEL_MIDI_NO_DATA));
            }
        }
    }
}

bool
AmdtpTransmitStreamProcessor::initPortCache() {
    // make use of the fact that audio ports are the first ports in
    // the cluster as per AMDTP. so we can sort the ports by position
    // and have very efficient lookups:
    // m_float_ports.at(i).buffer -> audio stream i buffer
    // for midi ports we simply cache all port info since they are (usually) not
    // that numerous
    m_nb_audio_ports = 0;
    m_audio_ports.clear();
    
    m_nb_midi_ports = 0;
    m_midi_ports.clear();
    
    for(PortVectorIterator it = m_Ports.begin();
        it != m_Ports.end();
        ++it )
    {
        AmdtpPortInfo *pinfo=dynamic_cast<AmdtpPortInfo *>(*it);
        assert(pinfo); // this should not fail!!

        switch( pinfo->getFormat() )
        {
            case AmdtpPortInfo::E_MBLA:
                m_nb_audio_ports++;
                break;
            case AmdtpPortInfo::E_SPDIF: // still unimplemented
                break;
            case AmdtpPortInfo::E_Midi:
                m_nb_midi_ports++;
                break;
            default: // ignore
                break;
        }
    }

    int idx;
    for (idx = 0; idx < m_nb_audio_ports; idx++) {
        for(PortVectorIterator it = m_Ports.begin();
            it != m_Ports.end();
            ++it )
        {
            AmdtpPortInfo *pinfo=dynamic_cast<AmdtpPortInfo *>(*it);
            debugOutput(DEBUG_LEVEL_VERY_VERBOSE,
                        "idx %u: looking at port %s at position %u\n",
                        idx, (*it)->getName().c_str(), pinfo->getPosition());
            if(pinfo->getPosition() == (unsigned int)idx) {
                struct _MBLA_port_cache p;
                p.port = dynamic_cast<AmdtpAudioPort *>(*it);
                if(p.port == NULL) {
                    debugError("Port is not an AmdtpAudioPort!\n");
                    return false;
                }
                p.buffer = NULL; // to be filled by updatePortCache
                #ifdef DEBUG
                p.buffer_size = (*it)->getBufferSize();
                #endif

                m_audio_ports.push_back(p);
                debugOutput(DEBUG_LEVEL_VERBOSE,
                            "Cached port %s at position %u\n",
                            p.port->getName().c_str(), idx);
                goto next_index;
            }
        }
        debugError("No MBLA port found for position %d\n", idx);
        return false;
next_index:
        continue;
    }

    for(PortVectorIterator it = m_Ports.begin();
        it != m_Ports.end();
        ++it )
    {
        AmdtpPortInfo *pinfo=dynamic_cast<AmdtpPortInfo *>(*it);
        debugOutput(DEBUG_LEVEL_VERY_VERBOSE,
                    "idx %u: looking at port %s at position %u, location %u\n",
                    idx, (*it)->getName().c_str(), pinfo->getPosition(), pinfo->getLocation());
        if ((*it)->getPortType() == Port::E_Midi) {
            struct _MIDI_port_cache p;
            p.port = dynamic_cast<AmdtpMidiPort *>(*it);
            if(p.port == NULL) {
                debugError("Port is not an AmdtpMidiPort!\n");
                return false;
            }
            p.position = pinfo->getPosition();
            p.location = pinfo->getLocation();
            p.buffer = NULL; // to be filled by updatePortCache
            #ifdef DEBUG
            p.buffer_size = (*it)->getBufferSize();
            #endif

            m_midi_ports.push_back(p);
            debugOutput(DEBUG_LEVEL_VERBOSE,
                        "Cached port %s at position %u, location %u\n",
                        p.port->getName().c_str(), p.position, p.location);
        }
    }

    return true;
}

//FIXME: DRY. Needs to be refactored with AmdtpReceiveStreamProcessor
void
AmdtpTransmitStreamProcessor::updatePortCache() {
    int idx;
    for (idx = 0; idx < m_nb_audio_ports; idx++) {
        struct _MBLA_port_cache& p = m_audio_ports.at(idx);
        AmdtpAudioPort *port = p.port;
        p.buffer = port->getBufferAddress();
        p.enabled = !port->isDisabled();
#ifdef DEBUG
	p.buffer_size = port->getBufferSize();
#endif
    }
    for (idx = 0; idx < m_nb_midi_ports; idx++) {
        struct _MIDI_port_cache& p = m_midi_ports.at(idx);
        AmdtpMidiPort *port = p.port;
        p.buffer = port->getBufferAddress();
        p.enabled = !port->isDisabled();
#ifdef DEBUG
	p.buffer_size = port->getBufferSize();
#endif
    }
}

} // end of namespace Streaming
