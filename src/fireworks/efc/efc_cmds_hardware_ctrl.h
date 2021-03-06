/*
 * Copyright (C) 2005-2008 by Pieter Palmers
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

#ifndef FIREWORKS_EFC_CMD_HARDWARE_CTRL_H
#define FIREWORKS_EFC_CMD_HARDWARE_CTRL_H

#include "efc_cmd.h"

namespace FireWorks {

#define FIREWORKS_EFC_FLAG_MIXER_ENABLED    1
#define FIREWORKS_EFC_FLAG_SPDIF_PRO        2
#define FIREWORKS_EFC_FLAG_SPDIF_RAW        4

class EfcGetClockCmd : public EfcCmd
{
public:
    EfcGetClockCmd();
    virtual ~EfcGetClockCmd() {};

    virtual bool serialize( Util::Cmd::IOSSerialize& se );
    virtual bool deserialize( Util::Cmd::IISDeserialize& de );

    virtual const char* getCmdName() const
    { return "EfcGetClockCmd"; }

    virtual void showEfcCmd();

    uint32_t    m_clock;
    uint32_t    m_samplerate;
    uint32_t    m_index;
};

class EfcSetClockCmd : public EfcCmd
{
public:
    EfcSetClockCmd();
    virtual ~EfcSetClockCmd() {};

    virtual bool serialize( Util::Cmd::IOSSerialize& se );
    virtual bool deserialize( Util::Cmd::IISDeserialize& de );

    virtual const char* getCmdName() const
    { return "EfcSetClockCmd"; }

    virtual void showEfcCmd();

    uint32_t    m_clock;
    uint32_t    m_samplerate;
    uint32_t    m_index;
};

class EfcPhyReconnectCmd : public EfcCmd
{
public:
    EfcPhyReconnectCmd();
    virtual ~EfcPhyReconnectCmd() {};

    virtual bool serialize( Util::Cmd::IOSSerialize& se );
    virtual bool deserialize( Util::Cmd::IISDeserialize& de );

    virtual const char* getCmdName() const
    { return "EfcPhyReconnectCmd"; }

    virtual void showEfcCmd();
};


class EfcGetFlagsCmd : public EfcCmd
{
public:
    EfcGetFlagsCmd();
    virtual ~EfcGetFlagsCmd() {};

    virtual bool serialize( Util::Cmd::IOSSerialize& se );
    virtual bool deserialize( Util::Cmd::IISDeserialize& de );

    virtual const char* getCmdName() const
    { return "EfcGetFlagsCmd"; }

    virtual void showEfcCmd();

    uint32_t    m_flags;
};

class EfcChangeFlagsCmd : public EfcCmd
{
public:
    EfcChangeFlagsCmd();
    virtual ~EfcChangeFlagsCmd() {};

    virtual bool serialize( Util::Cmd::IOSSerialize& se );
    virtual bool deserialize( Util::Cmd::IISDeserialize& de );

    virtual const char* getCmdName() const
    { return "EfcChangeFlagsCmd"; }

    virtual void showEfcCmd();

    uint32_t    m_setmask;
    uint32_t    m_clearmask;
};

class EfcIdentifyCmd : public EfcCmd
{
public:
    EfcIdentifyCmd();
    virtual ~EfcIdentifyCmd() {};

    virtual bool serialize( Util::Cmd::IOSSerialize& se );
    virtual bool deserialize( Util::Cmd::IISDeserialize& de );

    virtual const char* getCmdName() const
    { return "EfcIdentifyCmd"; }

    virtual void showEfcCmd();

};

} // namespace FireWorks

#endif // FIREWORKS_EFC_CMD_HARDWARE_CTRL_H
