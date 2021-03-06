/*
 * Copyright (C) 2009 by Pieter Palmers
 * Copyright (C) 2009 by Arnold Krille
 * Copyright (C) 2015 by Hector Martin
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

#ifndef DICE_FOCUSRITE_SAFFIRE_PRO26_H
#define DICE_FOCUSRITE_SAFFIRE_PRO26_H

#include "dice/dice_avdevice.h"

#include "libieee1394/configrom.h"

#include "focusrite_eap.h"

/**
 *  Saffire Pro26 application space
 */
// Versioning registers
#define SAFFIRE_PRO26_REGISTER_APP_VERSION 0x00
#define SAFFIRE_PRO26_REGISTER_APP_RELEASE 0x04
#define SAFFIRE_PRO26_REGISTER_APP_BUILDNR 0x08

// Nickname register
#define SAFFIRE_PRO26_REGISTER_APP_NICK_NAME 0x44
// NOTE: in bytes
#define SAFFIRE_PRO26_APP_NICK_NAME_SIZE 16

// Global monitor registers (application space)
#define SAFFIRE_PRO26_REGISTER_APP_GLOBAL_MUTE_SWITCH 0x10
#define SAFFIRE_PRO26_REGISTER_APP_GLOBAL_DIM_SWITCH  0x14
#define SAFFIRE_PRO26_REGISTER_APP_GLOBAL_DIM_VOLUME 0x58
#define SAFFIRE_PRO26_REGISTER_APP_GLOBAL_MONITOR_VOLUME 0x28

// Per line/out monitor volume and switches: registers are expected to be one after the other
//  each register controlling two output lines
// The whole number of physical analog output is thus 2*SAFFIRE_PRO26_APP_STEREO_LINEOUT_SIZE
#define SAFFIRE_PRO26_APP_STEREO_LINEOUT_SIZE 3

// Volume and switch monitor register
#define SAFFIRE_PRO26_REGISTER_APP_LINEOUT_MONITOR_VOLUME 0x18
#define SAFFIRE_PRO26_REGISTER_APP_LINEOUT_MONITOR_SWITCH 0x2C

// Switch control (per line/out mute, dim and mono)
#define SAFFIRE_PRO26_REGISTER_APP_LINEOUT_SWITCH_CONTROL 0x40

// Message set
//   The location of the message register and the values for each setting
#define SAFFIRE_PRO26_REGISTER_APP_MESSAGE_SET 0x0c
#define SAFFIRE_PRO26_MESSAGE_SET_NO_MESSAGE 0
#define SAFFIRE_PRO26_MESSAGE_SET_LINEOUT_MONITOR_VOLUME 1
#define SAFFIRE_PRO26_MESSAGE_SET_GLOBAL_DIM_MUTE_SWITCH 2
#define SAFFIRE_PRO26_MESSAGE_SET_LINEOUT_SWITCH_CONTROL 3
#define SAFFIRE_PRO26_MESSAGE_SET_INSTLINE 4
#define SAFFIRE_PRO26_MESSAGE_SET_MESSAGE_END 5

namespace Dice {
namespace Focusrite {

class SaffirePro26 : public Dice::Device {
public:
    SaffirePro26( DeviceManager& d,
                  ffado_smartptr<ConfigRom>( configRom ));
    ~SaffirePro26();

    bool discover();

    void showDevice();

    bool canChangeNickname() { return true; }
    bool setNickname( std::string name );
    std::string getNickname();

private:

    class SaffirePro26EAP : public FocusriteEAP
    {
    public:
        SaffirePro26EAP(Dice::Device& dev) : FocusriteEAP(dev) {
        }

        void setupSources_low();
        void setupDestinations_low();
        void setupSources_mid();
        void setupDestinations_mid();
        void setupSources_high();
        void setupDestinations_high();
        void setupDefaultRouterConfig_low();
        void setupDefaultRouterConfig_mid();
        void setupDefaultRouterConfig_high();

        class MonitorSection : public Control::Container
        {
        public:
          MonitorSection(Dice::Focusrite::FocusriteEAP*, std::string);
        private:
          Dice::Focusrite::FocusriteEAP* m_eap;
        };
    };
    Dice::EAP* createEAP();

};

}
}

#endif
// vim: et
