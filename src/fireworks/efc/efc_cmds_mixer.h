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

#ifndef FIREWORKS_EFC_CMD_MIXER_H
#define FIREWORKS_EFC_CMD_MIXER_H

#include "efc_cmd.h"

namespace FireWorks {

class EfcGenericMixerCmd : public EfcCmd
{
public:
    EfcGenericMixerCmd(enum eMixerTarget, enum eMixerCommand);
    EfcGenericMixerCmd(enum eMixerTarget, enum eMixerCommand, int channel);
    virtual ~EfcGenericMixerCmd() {};

    virtual bool serialize( Util::Cmd::IOSSerialize& se );
    virtual bool deserialize( Util::Cmd::IISDeserialize& de );

    virtual void showEfcCmd();
    
    bool setType( enum eCmdType type );
    enum eCmdType getType() {return m_type;};
    bool setTarget( enum eMixerTarget target );
    enum eMixerTarget getTarget() {return m_target;};
    bool setCommand( enum eMixerCommand cmd );
    enum eMixerCommand getCommand() {return m_command;};
    
    virtual const char* getCmdName() const
        { return "EfcGenericMixerCmd"; }

    int32_t     m_channel;
    uint32_t    m_value;

private:
    enum eCmdType       m_type;
    enum eMixerTarget   m_target;
    enum eMixerCommand  m_command;
};

} // namespace FireWorks

#endif // FIREWORKS_EFC_CMD_MIXER_H
