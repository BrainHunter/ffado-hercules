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

#ifndef __FFADO_HERCULESPORT__
#define __FFADO_HERCULESPORT__

/**
 * This file implements the ports used in Hercules devices
 */

#include "HerculesPortInfo.h"
#include "../generic/Port.h"

#include "debugmodule/debugmodule.h"

namespace Streaming {

/*!
\brief The Base Class for Hercules Audio Port


*/
class HerculesAudioPort
    : public AudioPort, public HerculesPortInfo
{

public:

    HerculesAudioPort(PortManager &m,
                  std::string name,
                  enum E_Direction direction,
                  int position,
                  int size)
    : AudioPort(m, name, direction),
      HerculesPortInfo( position, size) // TODO: add more port information parameters here if nescessary
    {};

    virtual ~HerculesAudioPort() {};
};

/*!
\brief The Base Class for an Hercules Midi Port


*/
class HerculesMidiPort
    : public MidiPort, public HerculesPortInfo
{

public:

    HerculesMidiPort(PortManager &m,
                 std::string name,
                 enum E_Direction direction,
                 int position)
        : MidiPort(m, name, direction),
          HerculesPortInfo(position, 0)  // TODO: add more port information parameters here if nescessary
    {};

    virtual ~HerculesMidiPort() {};
};

/*!
\brief The Base Class for an Hercules Control Port


*/
class HerculesControlPort
    : public ControlPort, public HerculesPortInfo
{

public:

    HerculesControlPort(PortManager &m,
                    std::string name,
                    enum E_Direction direction,
                    int position)
        : ControlPort(m, name, direction),
          HerculesPortInfo(position, 2) // TODO: add more port information parameters here if nescessary
    {};

    virtual ~HerculesControlPort() {};
};

} // end of namespace Streaming

#endif /* __FFADO_HERCULESPORT__ */
