#
# Copyright (C) 2008 by Arnold Krille
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

# from PyQt4.QtGui import QTextEdit, QAbstractSlider, QColor
# from PyQt4.QtCore import QObject, pyqtSignal, QString
from ffado.import_pyqt import *

import logging
log = logging.getLogger('logginghandler')

class QStatusLogger( QObject, logging.Handler ):
    log = pyqtSignal(QString if ffado_pyqt_version == 4 else str, int, name='log')
    def __init__( self, parent, statusbar, level=logging.NOTSET ):
        QObject.__init__( self, parent )
        logging.Handler.__init__( self, level )
        self.setFormatter( logging.Formatter( "%(name)s: %(message)s" ) )
        self.log.connect(statusbar.showMessage)

    def emit( self, record ):
        self.log.emit('%s: %s'.format(record.name, record.getMessage()), 5000)

class QTextLogger( logging.Handler ):
    def __init__( self, parent, level=logging.NOTSET ):
        logging.Handler.__init__( self, level )

        self.textedit = QTextEdit( parent )

        self.textedit.setReadOnly( True )
        self.textedit.setAcceptRichText( True )

    def emit( self, record ):
        color = QColor( "#000000" )
        if record.levelno > 20:
            color = QColor( "#ffff00" )
        if record.levelno > 30:
            color = QColor( "#ff0000" )
        if record.levelno <= 10:
            color = QColor( "#808080" )
        self.textedit.setTextColor( color )
        tmp = "%s %s: %s" % (record.asctime, record.name, record.getMessage())
        self.textedit.append( tmp )
        self.textedit.verticalScrollBar().triggerAction( QAbstractSlider.SliderToMaximum )

#
# vim: et
#
