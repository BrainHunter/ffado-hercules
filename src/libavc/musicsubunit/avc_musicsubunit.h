/*
 * Copyright (C) 2005-2008 by Pieter Palmers
 * Copyright (C) 2005-2008 by Daniel Wagner
 *
 * This file is part of FFADO
 * FFADO = Free FireWire (pro-)audio drivers for Linux
 *
 * FFADO is based upon FreeBoB
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

#ifndef AVC_MUSICSUBUNIT_H
#define AVC_MUSICSUBUNIT_H

#include <vector>
#include "libutil/serialize.h"

namespace AVC {

class Unit;
class Plug;
class AVCMusicStatusDescriptor;

// /////////////////////////////

class SubunitMusic: public Subunit {
 public:
    SubunitMusic( Unit& avDevice,
                  subunit_t id );
    SubunitMusic();
    virtual ~SubunitMusic();
    
    virtual bool discover();
    virtual bool initPlugFromDescriptor( Plug& plug );

    virtual bool loadDescriptors();
    
    virtual void showMusicPlugs();
    
    virtual void setVerboseLevel(int l);
    virtual const char* getName();
protected:
    virtual bool serializeChild( std::string basePath,
                                 Util::IOSerialize& ser ) const;
    virtual bool deserializeChild( std::string basePath,
                                   Util::IODeserialize& deser,
                                   Unit& avDevice );
    virtual bool deserializeUpdateChild( std::string basePath,
                                         Util::IODeserialize& deser );

    class AVCMusicStatusDescriptor*  m_status_descriptor;
};

}

#endif
