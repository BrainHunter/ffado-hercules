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

#warning Hercules support is currently useless

#include "config.h"

#include "hercules/hercules_avdevice.h"

#include "libieee1394/configrom.h"
#include "libieee1394/ieee1394service.h"

#include "libavc/avc_definitions.h"

#include "debugmodule/debugmodule.h"

#include "libstreaming/hercules/HerculesReceiveStreamProcessor.h"
#include "libstreaming/hercules/HerculesTransmitStreamProcessor.h"
#include "libstreaming/hercules/HerculesPort.h"

#include "libutil/Time.h"

#include "devicemanager.h"

#include <string>
#include <stdint.h>
#include <assert.h>
#include "libutil/ByteSwap.h"
#include <iostream>
#include <sstream>

#include <libraw1394/csr.h>

namespace Hercules {

// Ports declarations
const PortEntry Ports_Hercules[] =
{
    {"In01", HERCULES_PA_IN, 0},
    {"In02", HERCULES_PA_IN, 3},
    {"In03", HERCULES_PA_IN, 6},
    {"In04", HERCULES_PA_IN, 9},
    {"In05", HERCULES_PA_IN, 12},
    {"In06", HERCULES_PA_IN, 15},
    {"In07", HERCULES_PA_IN, 18},
    {"In08", HERCULES_PA_IN, 21},
    {"In09", HERCULES_PA_IN, 24},
    {"In10", HERCULES_PA_IN, 27},
    {"In11", HERCULES_PA_IN, 30},
    {"In12", HERCULES_PA_IN, 33},
    {"InSPDIF1-L", HERCULES_PA_IN, 36},
    {"InSPDIF1-R", HERCULES_PA_IN, 40},
    {"InSPDIF2-L", HERCULES_PA_IN, 44},
    {"InSPDIF2-R", HERCULES_PA_IN, 48},
    {"Out01", HERCULES_PA_OUT, 0},
    {"Out02", HERCULES_PA_OUT, 3},
    {"Out03", HERCULES_PA_OUT, 6},
    {"Out04", HERCULES_PA_OUT, 9},
    {"Out05", HERCULES_PA_OUT, 12},
    {"Out06", HERCULES_PA_OUT, 15},
    {"Out07", HERCULES_PA_OUT, 18},
    {"Out08", HERCULES_PA_OUT, 21},
    {"OutSPDIF1-L", HERCULES_PA_OUT, 36},
    {"OutSPDIF1-R", HERCULES_PA_OUT, 40},
    {"OutSPDIF2-L", HERCULES_PA_OUT, 44},
    {"OutSPDIF2-R", HERCULES_PA_OUT, 48},
};


Device::Device( DeviceManager& d,
                        std::auto_ptr<ConfigRom>( configRom ))
    : FFADODevice( d, configRom )
    , m_iso_recv_channel ( 1 )
    , m_iso_send_channel ( 0 )
    , m_rx_bandwidth ( -1 )
    , m_tx_bandwidth ( -1 )
    , m_receiveProcessor ( 0 )
    , m_transmitProcessor ( 0 )
{
    debugOutput( DEBUG_LEVEL_VERBOSE, "Created Hercules::Device (NodeID %d)\n",
                 getConfigRom().getNodeId() );
}

Device::~Device()
{

}

bool
Device::probe( Util::Configuration& c, ConfigRom& configRom, bool generic )
{
    if (generic) {
        return false;
    } else {
        // check if device is in supported devices list
        unsigned int vendorId = configRom.getNodeVendorId();
        unsigned int modelId = configRom.getModelId();

        Util::Configuration::VendorModelEntry vme = c.findDeviceVME( vendorId, modelId );
        return c.isValid(vme) && vme.driver == Util::Configuration::eD_Hercules;
    }
}

FFADODevice *
Device::createDevice( DeviceManager& d,
                          std::auto_ptr<ConfigRom>( configRom ))
{
    return new Device(d, configRom );
}

bool
Device::discover()
{
    unsigned int vendorId = getConfigRom().getNodeVendorId();
    unsigned int modelId = getConfigRom().getModelId();

    Util::Configuration &c = getDeviceManager().getConfiguration();
    Util::Configuration::VendorModelEntry vme = c.findDeviceVME( vendorId, modelId );

    if (c.isValid(vme) && vme.driver == Util::Configuration::eD_Hercules) {
        debugOutput( DEBUG_LEVEL_VERBOSE, "found %s %s\n",
                     vme.vendor_name.c_str(),
                     vme.model_name.c_str());
    } else {
        debugWarning("Using generic Hercules support for unsupported device '%s %s'\n",
                     getConfigRom().getVendorName().c_str(), getConfigRom().getModelName().c_str());
    }

    return true;
}

bool
Device::sendFunction( fb_quadlet_t function,
                      fb_quadlet_t param,
		      fb_quadlet_t *result)
{
    bool retval = get1394Service().lockCompareSwap(0xffc0 | getNodeId(),
                                                   HERCULES_BASE_ADDR, function,
				                   param, result);

    if (!retval) {
        debugError("lockCompareSwap failed\n");
    }
	    
    return retval;
}

bool
Device::readRegistry( int reg,
                      fb_quadlet_t *result )
{
     bool retval = get1394Service().read_quadlet(0xffc0 | getNodeId(),
                                                 HERCULES_REG_ADDR, result);

     if (!retval)
         debugError("read_quadlet failed\n");

     return retval;
}

int
Device::getSamplingFrequency( ) {
     fb_quadlet_t rate_value;
     int rate = 0;

     readRegistry(HERCULES_RATE, &rate_value);

     switch (rate_value) {
       case RATE_32000:
         rate = 32000;
	 break;
       case RATE_44100:
         rate = 44100;
	 break;
       case RATE_48000:
         rate = 48000;
	 break;
       case RATE_88200:
         rate = 88200;
	 break;
       case RATE_96000:
         rate = 96000;
	 break;
       default:
         // Non existant!
	 break;
     }
     
     return rate;
}

std::vector<int>
Device::getSupportedSamplingFrequencies()
{
    std::vector<int> frequencies;
    frequencies.push_back(32000);
    frequencies.push_back(44100);
    frequencies.push_back(48000);
    frequencies.push_back(88200);
    frequencies.push_back(96000);
    return frequencies;
}

FFADODevice::ClockSourceVector
Device::getSupportedClockSources() {
    FFADODevice::ClockSourceVector r;
    return r;
}

bool
Device::setActiveClockSource(ClockSource s) {
    return false;
}

FFADODevice::ClockSource
Device::getActiveClockSource() {
    ClockSource s;
    return s;
}


int
Device::getConfigurationId( ) {
    return 0;
}

bool
Device::setSamplingFrequency( int samplingFrequency )
{
     fb_quadlet_t set_rate;
     fb_quadlet_t result[2];

     switch (samplingFrequency) {
       case 32000:
         set_rate = RATE_32000;
	 break;
       case 44100:
         set_rate = RATE_44100;
	 break;
       case 48000:
         set_rate = RATE_48000;
	 break;
       case 88200:
         set_rate = RATE_88200;
	 break;
       case 96000:
         set_rate = RATE_96000;
	 break;
       default:
         // Non existant!
	 break;
     }
     
     sendFunction(HERCULES_SET_RATE, set_rate, result);

     // we are interested in 32 lsb part of result
     //if (result[1] != set_rate) {
     //    debugError("Sampling frequency not set : 0x%08lX\n", result);
     //    return false;
     ///}

     return true;
}

bool
Device::lock() {

    return true;
}


bool
Device::unlock() {

    return true;
}

void
Device::showDevice()
{
    unsigned int vendorId = getConfigRom().getNodeVendorId();
    unsigned int modelId = getConfigRom().getModelId();

    Util::Configuration &c = getDeviceManager().getConfiguration();
    Util::Configuration::VendorModelEntry vme = c.findDeviceVME( vendorId, modelId );

    debugOutput(DEBUG_LEVEL_VERBOSE,
        "%s %s at node %d\n", vme.vendor_name.c_str(), vme.model_name.c_str(), getNodeId());
}

unsigned int
Device::getNominalFramesPerPacket(int framerate) {
    unsigned int n_events = 0;
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

// returns the size of a sample for all channels
// it's the same for playback and recording
signed int
Device::getEventSize() {
    return 0x40;
}

bool
Device::prepare() {

    int samp_freq = getSamplingFrequency();
    unsigned int event_size_in = getEventSize();
    unsigned int event_size_out = getEventSize();

	debugOutput(DEBUG_LEVEL_NORMAL, "Preparing Hercules device...\n" );
	WriteRegister(0, HERCULES_INIT);
	// set to 48 kHz
	LockRegister(0, HERCULES_SET_SYNC, RATE_48000_SET);
	ReadRegister(HERCULES_RATE);
	ReadRegister(HERCULES_RATE);

    // Allocate bandwidth if not previously done.
    // FIXME: The bandwidth allocation calculation can probably be
    // refined somewhat since this is currently based on a rudimentary
    // understanding of the ieee1394 iso protocol.
    // Currently we assume the following.
    //   * Ack/iso gap = 0.05 us
    //   * DATA_PREFIX = 0.16 us
    //   * DATA_END    = 0.26 us
    // These numbers are the worst-case figures given in the ieee1394
    // standard.  This gives approximately 0.5 us of overheads per packet -
    // around 25 bandwidth allocation units (from the ieee1394 standard 1
    // bandwidth allocation unit is 125/6144 us).  We further assume the
    // MOTU is running at S400 (which it should be) so one allocation unit
    // is equivalent to 1 transmitted byte; thus the bandwidth allocation
    // required for the packets themselves is just the size of the packet.
    // We used to allocate based on the maximum packet size (1160 bytes at
    // 192 kHz for the traveler) but now do this based on the actual device
    // state by utilising the result from getEventSize() and remembering
    // that each packet has an 8 byte CIP header.  Note that bandwidth is
    // allocated on a *per stream* basis - it must be allocated for both the
    // transmit and receive streams.  While most MOTU modules are close to
    // symmetric in terms of the number of in/out channels there are
    // exceptions, so we deal with receive and transmit bandwidth separately.
    signed int n_events_per_packet = getNominalFramesPerPacket(samp_freq);
    m_rx_bandwidth = 25 + (n_events_per_packet*event_size_in);
    m_tx_bandwidth = 25 + (n_events_per_packet*event_size_out);
	    
    // Assign iso channels if not already done
    if (m_iso_recv_channel < 0)
        m_iso_recv_channel = get1394Service().allocateIsoChannelGeneric(m_rx_bandwidth);

    if (m_iso_send_channel < 0)
        m_iso_send_channel = get1394Service().allocateIsoChannelGeneric(m_tx_bandwidth);

    debugOutput(DEBUG_LEVEL_VERBOSE, "recv channel = %d, send channel = %d\n",
        m_iso_recv_channel, m_iso_send_channel);

    if (m_iso_recv_channel<0 || m_iso_send_channel<0) {
        // be nice and deallocate
        if (m_iso_recv_channel >= 0)
            get1394Service().freeIsoChannel(m_iso_recv_channel);
        if (m_iso_send_channel >= 0)
            get1394Service().freeIsoChannel(m_iso_send_channel);

        debugFatal("Could not allocate iso channels!\n");
        return false;
    }
									
    // get the device specific and/or global SP configuration
    Util::Configuration &config = getDeviceManager().getConfiguration();
    // base value is the config.h value
    float recv_sp_dll_bw = STREAMPROCESSOR_DLL_BW_HZ;
    float xmit_sp_dll_bw = STREAMPROCESSOR_DLL_BW_HZ;

    // we can override that globally
    config.getValueForSetting("streaming.spm.recv_sp_dll_bw", recv_sp_dll_bw);
    config.getValueForSetting("streaming.spm.xmit_sp_dll_bw", xmit_sp_dll_bw);

    // or override in the device section
    config.getValueForDeviceSetting(getConfigRom().getNodeVendorId(), getConfigRom().getModelId(), "recv_sp_dll_bw", recv_sp_dll_bw);
    config.getValueForDeviceSetting(getConfigRom().getNodeVendorId(), getConfigRom().getModelId(), "xmit_sp_dll_bw", xmit_sp_dll_bw);
					    
    // construct the streamprocessor
    m_receiveProcessor = new Streaming::HerculesReceiveStreamProcessor(*this, event_size_in);

    // The first thing is to initialize the processor.  This creates the
    // data structures.
    if (!m_receiveProcessor->init()) {
        debugFatal("Could not initialize receive processor!\n");
	return false;
    }

    if (!m_receiveProcessor->setDllBandwidth(recv_sp_dll_bw)) {
        debugFatal("Could not set DLL bandwidth\n");
        delete m_receiveProcessor;
        m_receiveProcessor = NULL;
        return false;
    }
					
    // Now we add ports to the processor
    debugOutput(DEBUG_LEVEL_VERBOSE,"Adding ports to receive processor\n");

    // Add audio capture ports
    if (!addDirPorts(Streaming::Port::E_Capture, samp_freq)) {
        return false;
    }
    
    // Do the same for the transmit processor
    m_transmitProcessor = new Streaming::HerculesTransmitStreamProcessor(*this, event_size_out);

    m_transmitProcessor->setVerboseLevel(getDebugLevel());

    if (!m_transmitProcessor->init()) {
        debugFatal("Could not initialize transmit processor!\n");
	return false;
    }

    if (!m_transmitProcessor->setDllBandwidth(xmit_sp_dll_bw)) {
        debugFatal("Could not set DLL bandwidth\n");
	delete m_transmitProcessor;
	m_transmitProcessor = NULL;
	return false;
    }
    
    // Now we add ports to the processor
    debugOutput(DEBUG_LEVEL_VERBOSE,"Adding ports to transmit processor\n");

    // Add audio playback ports
    if (!addDirPorts(Streaming::Port::E_Playback, samp_freq)) {
        return false;
    }
    
    return true;
}

int
Device::getStreamCount() {
    return 2;
}

Streaming::StreamProcessor *
Device::getStreamProcessorByIndex(int i) {

    switch (i) {
    case 0:
        return m_receiveProcessor;
    case 1:
        return m_transmitProcessor;
    default:
        return NULL;
    }
    return 0;
}

bool
Device::startStreamByIndex(int i) {
    switch (i) {
    case 0:
        m_receiveProcessor->setChannel(m_iso_recv_channel);
        break;
    case 1:
        m_transmitProcessor->setChannel(m_iso_send_channel);
	break;
    default:
        return false;
    }
    return true;
}

bool
Device::stopStreamByIndex(int i) {
    return true;
}

signed int
Device::getIsoRecvChannel(void) {
    return m_iso_recv_channel;
}

signed int
Device::getIsoSendChannel(void) {
    return m_iso_send_channel;
}
	

bool Device::addPort(Streaming::StreamProcessor *s_processor,
  char *name, enum Streaming::Port::E_Direction direction,
  int position, int size) {
/*
 * Internal helper function to add a Hercules port to a given stream processor.
 * This just saves the unnecessary replication of what is essentially
 * boilerplate code.  Note that the port name is freed by this function
 * prior to exit.
 */
    Streaming::Port *p = NULL;

    p = new Streaming::HerculesAudioPort(*s_processor, name, direction, position, size);

    if (!p) {
        debugOutput(DEBUG_LEVEL_VERBOSE, "Skipped port %s\n", name);
    }
    free(name);
    return true;
}

bool Device::addDirPorts(enum Streaming::Port::E_Direction direction, unsigned int sample_rate) {
/*
 * Internal helper method: adds all required ports for the given direction
 * based on the indicated sample rate and optical mode.
 *
 * Notes: currently ports are not created if they are disabled due to sample
 * rate or optical mode.  However, it might be better to unconditionally
 * create all ports and just disable those which are not active.
 */

    Streaming::StreamProcessor *s_processor;
    unsigned int i;
    unsigned int dir;
    char *buff;

    if (direction == Streaming::Port::E_Capture) {
        dir = HERCULES_PA_IN;
        s_processor = m_receiveProcessor;
    } else {
        dir = HERCULES_PA_OUT;
        s_processor = m_transmitProcessor;
    }

    for (i = 0; i < N_ELEMENTS( Ports_Hercules ); i++) {
        if (Ports_Hercules[i].port_flags & dir) {
	    asprintf(&buff,"%s", Ports_Hercules[i].port_name);
	    if (!addPort(s_processor, buff, direction, Ports_Hercules[i].port_offset, 0))
	        return false;
	}
    }

    return true;
}

unsigned int Device::ReadRegister(fb_nodeaddr_t reg) {
/*
 * Attempts to read the requested register from the HERCULES.
 */

    quadlet_t quadlet = 0;

    /* If the supplied register has no upper bits set assume it's a
     * register which is assumed to be relative to HERCULES_REG_ADDR.
     */
    if ((reg & HERCULES_REG_ADDR) == 0)
        reg |= HERCULES_REG_ADDR;

    // Note: 1394Service::read() expects a physical ID, not the node id
    if (get1394Service().read(0xffc0 | getNodeId(), reg, 1, &quadlet) <= 0) {
        debugError("Error doing hercules read from register 0x%012"PRId64"\n",reg);
    }

    return CondSwapFromBus32(quadlet);
}

signed int Device::WriteRegister(fb_nodeaddr_t reg, quadlet_t data) {
/*
 * Attempts to write the given data to the requested HERCULES register.
 */

    unsigned int err = 0;
    data = CondSwapToBus32(data);

    /* If the supplied register has no upper bits set assume it's a
     * register which is assumed to be relative to HERCULES_REG_ADDR.
     */
    if ((reg & HERCULES_REG_ADDR) == 0)
        reg |= HERCULES_REG_ADDR;

    // Note: 1394Service::write() expects a physical ID, not the node id
    if (get1394Service().write(0xffc0 | getNodeId(), reg, 1, &data) <= 0) {
        err = 1;
        debugError("Error doing hercules write to register 0x%012"PRIx64"\n",reg);
    }

    SleepRelativeUsec(100);
    return (err==0)?0:-1;
}

signed int Device::LockRegister(fb_nodeaddr_t reg, quadlet_t compare_value, quadlet_t swap_value) {
/*
 * Attempts to lock the given data to the requested HERCULES register.
 */
	fb_quadlet_t result[2];
	int err = 0;

    /* If the supplied register has no upper bits set assume it's a
     * register which is assumed to be relative to HERCULES_REG_ADDR.
     */
    if ((reg & HERCULES_BASE_ADDR) == 0)
        reg |= HERCULES_BASE_ADDR;


	if (!get1394Service().lockCompareSwap(0xffc0 | getNodeId(), reg, compare_value, swap_value, result)) {
		err = 1;
		debugError("Error doing hercules lock to register 0x%012"PRIx64"\n",reg);
	}

	SleepRelativeUsec(100);
	return (err==0)?0:-1;
}

}
