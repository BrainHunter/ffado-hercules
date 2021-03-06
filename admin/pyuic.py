#!/usr/bin/python
#
# Copyright (C) 2007-2008 Arnold Krille
#
# This file is part of FFADO
# FFADO = Free FireWire (pro-)audio drivers for Linux
#
# FFADO is based upon FreeBoB.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) version 3 of the License.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import imp

def pyuic_action( target, source, env ):
	env.Execute( "pyuic " + str( source[0] ) + " > " + str( target[0] ) )
	return 0

def pyuic_string( target, source, env ):
	return "building '%s' from '%s'" % ( str(target[0]), str( source[0] ) )

def PyQtCheck( context ):
	context.Message( "Checking for pyuic (by checking for the python module pyqtconfig) " )
	ret = True
	try:
		imp.find_module( "pyqtconfig" )
	except ImportError:
		ret = False
	context.Result( ret )
	return ret

def generate( env, **kw ):
	env['BUILDERS']['PyUIC'] = env.Builder( action=pyuic_action, src_suffix=".ui", single_source=True )
	env['PYUIC_TESTS'] = { "PyQtCheck" : PyQtCheck }


def exists( env ):
	return 1

