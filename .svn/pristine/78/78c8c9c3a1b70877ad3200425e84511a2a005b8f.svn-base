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

#ifndef CONTROLCLIENT_H
#define CONTROLCLIENT_H

#include "debugmodule/debugmodule.h"

/* Work around a bug in dbus-c++ 0.9.0 which prevents compilation under gcc7 */
#ifndef DBUS_HAS_RECURSIVE_MUTEX
#define DBUS_HAS_RECURSIVE_MUTEX
#endif

#include <dbus-c++/dbus.h>

#include "controlclient-glue.h"

namespace DBusControl {

// simple fader element
class ContinuousClient
: public org::ffado::Control::Element::Continuous_proxy,
  public DBus::IntrospectableProxy,
  public DBus::ObjectProxy
{
public:

    ContinuousClient( DBus::Connection& connection, const char* path, const char* name );

private:
    DECLARE_DEBUG_MODULE;
};

}

#endif //CONTROLCLIENT_H

