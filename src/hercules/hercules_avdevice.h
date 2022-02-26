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
#ifndef HERCULES_DEVICE_H
#define HERCULES_DEVICE_H

#include "ffadodevice.h"

#include "debugmodule/debugmodule.h"
#include "libavc/avc_definitions.h"
#include "libutil/Configuration.h"

#include "libstreaming/hercules/HerculesReceiveStreamProcessor.h"
#include "libstreaming/hercules/HerculesTransmitStreamProcessor.h"

#define HERCULES_BASE_ADDR           0xffff00000000ULL
#define HERCULES_REG_ADDR            (0x80000000|HERCULES_BASE_ADDR)

/* Port Active Flags (ports declaration) */
#define HERCULES_PA_IN               0x0200
#define HERCULES_PA_OUT              0x0400

#define NB_IN_CHANNELS               16

#define HERCULES_INIT                0x161200ad

#define HERCULES_SET_INPUT_LEVEL     0x16120003
#define HERCULES_SET_OUTPUT_LEVEL    0x16120004
#define HERCULES_SET_RATE            0x16120009
#define HERCULES_SET_MONITORING      0x1612000e
#define HERCULES_SET_SYM             0x16120013
#define HERCULES_SET_COPY_COND       0x16120017
#define HERCULES_SET_REWIRE          0x16120041
#define HERCULES_SET_SYNC            0x16120070

/* Read locations */
#define HERCULES_FW_VER              0x00
#define HERCULES_RATE                0x04
#define HERCULES_SYM                 0x0c
#define HERCULES_INPUT_LEVEL_1_8     0x10
#define HERCULES_INPUT_LEVEL_9_12    0x14
#define HERCULES_OUTPUT_LEVEL_1_4    0x18
#define HERCULES_OUTPUT_LEVEL_5_8    0x1c
#define HERCULES_COPY_COND           0x24
#define HERCULES_SERIAL_NUMBER       0x28
#define HERCULES_REWIRE              0x2c
#define HERCULES_VUMETERS_1_4        0x30
#define HERCULES_VUMETERS_5_8        0x34
#define HERCULES_VUMETERS_9_12       0x38

#define RATE_32000                   0x0ef20e0e
#define RATE_44100                   0x0ef10e0e
#define RATE_48000                   0x0ef00e0e
#define RATE_88200                   0x0ef50e0e
#define RATE_96000                   0x0ef40e0e

#define RATE_32000_SET               0x00020100
#define RATE_44100_SET               0x00010100
#define RATE_48000_SET               0x00000100
#define RATE_88200_SET               0x00050100
#define RATE_96000_SET               0x00040100

class ConfigRom;
class Ieee1394Service;

namespace Hercules {

struct PortEntry {
    const char *port_name;
    unsigned int port_flags;
    unsigned int port_offset;
};

/* Macro to calculate the size of an array */
#define N_ELEMENTS(_array) (sizeof(_array) / sizeof((_array)[0]))

class Device : public FFADODevice {
public:
    Device( DeviceManager& d,
                std::auto_ptr<ConfigRom>( configRom ));
    virtual ~Device();

    static bool probe( Util::Configuration& c, ConfigRom& configRom, bool generic = false );
    static FFADODevice * createDevice( DeviceManager& d,
                                       std::auto_ptr<ConfigRom>( configRom ));
    static int getConfigurationId();
    virtual bool discover();

    virtual void showDevice();

    virtual bool setSamplingFrequency( int );
    virtual int getSamplingFrequency( );
    virtual std::vector<int> getSupportedSamplingFrequencies();

    virtual ClockSourceVector getSupportedClockSources();
    virtual bool setActiveClockSource(ClockSource);
    virtual ClockSource getActiveClockSource();

    virtual int getStreamCount();
    virtual Streaming::StreamProcessor *getStreamProcessorByIndex(int i);

    virtual bool prepare();
    virtual bool lock();
    virtual bool unlock();

    virtual bool startStreamByIndex(int i);
    virtual bool stopStreamByIndex(int i);

    signed int getIsoRecvChannel(void);
    signed int getIsoSendChannel(void);

	unsigned int ReadRegister(fb_nodeaddr_t reg);
	signed int WriteRegister(fb_nodeaddr_t reg, quadlet_t data);
	signed int LockRegister(fb_nodeaddr_t reg, quadlet_t compare_value, quadlet_t swap_value);

protected:
    signed int m_iso_recv_channel, m_iso_send_channel;
    signed int m_rx_bandwidth, m_tx_bandwidth;

    Streaming::HerculesReceiveStreamProcessor *m_receiveProcessor;
    Streaming::HerculesTransmitStreamProcessor *m_transmitProcessor;

	
private:
    bool sendFunction(fb_quadlet_t function, fb_quadlet_t param, fb_quadlet_t* result);
    bool readRegistry(int reg, fb_quadlet_t* result);

    int getEventSize();
    unsigned int getNominalFramesPerPacket(int framerate);
    bool addPort(Streaming::StreamProcessor*, char*, Streaming::Port::E_Direction, int, int);
    bool addDirPorts(Streaming::Port::E_Direction, unsigned int);
};

}

#endif // HERCULES_DEVICE_H
