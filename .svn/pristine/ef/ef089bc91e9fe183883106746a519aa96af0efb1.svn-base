#
# Copyright (C) 2017 by Jonathan Woithe
#
# This file is part of FFADO
# FFADO = Free FireWire (pro-)audio drivers for Linux
#
# FFADO is based upon FreeBoB.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# This module handles the importing of PyQt modules for both PyQt4 and PyQt5
# under Python2 or Python3.  If Python3 is installed it is assumed that
# PyQt5 is in use (this is reasonable because PyQt5 is what everyone wants
# to use under Python3).  Otherwise (that is, under Python2), an import of
# PyQt4 is tried first; if an import error occurs then PyQt5 is assumed.
#
# All modules used by any part of ffado-mixer are imported.  This greatly
# simplifies the process.  Otherwise the modules to import would be delivered
# by string variables, and there isn't a supported way to do this across 
# Python2 and Python3.

import sys
ffado_python3 = sys.version_info >= (3,)

if ffado_python3:
    ffado_pyqt_version = 5
else:
    try:
        from PyQt4 import QtGui
        ffado_pyqt_version = 4
    except ImportError:
        ffado_pyqt_version = 5

if ffado_pyqt_version == 4:
    from PyQt4 import QtGui, QtCore, Qt, uic
    from PyQt4.QtCore import QByteArray, QObject, QTimer, Qt, pyqtSignal, QString, pyqtSlot
    from PyQt4.QtGui import *
else:
    from PyQt5 import QtGui, Qt, QtCore, Qt, QtWidgets, uic
    from PyQt5.QtCore import QByteArray, QObject, pyqtSignal, pyqtSlot, QTimer, Qt
    from PyQt5.QtGui import *
    from PyQt5.QtWidgets import *
