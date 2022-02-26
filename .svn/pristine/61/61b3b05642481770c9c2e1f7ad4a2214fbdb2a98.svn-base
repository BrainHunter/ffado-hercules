# coding=utf8
#
# Copyright (C) 2009 by Arnold Krille
#               2013 by Philippe Carriere
#
# This file is part of FFADO
# FFADO = Free FireWire (pro-)audio drivers for Linux
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

# from PyQt4 import QtGui, QtCore, Qt
# from PyQt4.QtCore import pyqtSignal
# from PyQt4.QtGui import QColor, QAbstractSlider, QDoubleSpinBox, QWidgetAction
# from PyQt4.QtGui import QAction, QPainter, QWidget, QGridLayout, QLabel
# from PyQt4.QtGui import QLayout, QSlider, QLineEdit, QPalette
# from PyQt4.QtGui import QVBoxLayout, QHBoxLayout, QTabWidget, QToolBar
# from PyQt4.QtGui import QComboBox, QScrollArea, QPushButton, QSizePolicy
from ffado.import_pyqt import *

import dbus, math, decimal

import ffado.config

import logging
log = logging.getLogger("matrixmixer")

def toDBvalue(value):
    n = int(value)
    c2p14 = 16384.0
    if n > 164:
        return round(20.0*math.log10(float(n)/c2p14), 2)
    else:
        return -40.0

def fromDBvalue(value):
    v = float(value)
    c2p14 = 16384.0
    if (v > -40):
        return int(round(math.pow(10.0, (value/20.0))*c2p14, 0))
    else:
        return 0

# v, vl, vr in linear scale
# b range in [-1:1]
def getVolumeLeft(v, b):
    return int(round(0.5*v*(1.0-b),0))
def getVolumeRight(v, b):
    return v-int(round(0.5*v*(1.0-b),0))
def getStereoVolume(vl, vr):
    return int(round(vl+vr,0))
def getStereoBalance(vl, vr):
    if ((vl+vr) == 0):
        return 0
    else:
        return round(float(vr-vl)/float(vr+vl),2)

class ColorForNumber:
    def __init__(self):
        self.colors = dict()

    def addColor(self, n, color):
        self.colors[n] = color

    def getColor(self, n):
        #print( "ColorForNumber.getColor( %g )" % (n) )
        keys = sorted(self.colors.keys())
        low = keys[-1]
        high = keys[-1]
        for i in range(len(keys)-1):
            if keys[i] <= n and keys[i+1] > n:
                low = keys[i]
                high = keys[i+1]
        #print( "%g is between %g and %g" % (n, low, high) )
        f = 0
        if high != low:
            f = (n-low) / (high-low)
        lc = self.colors[low]
        hc = self.colors[high]
        return QColor(
                int((1-f)*lc.red()   + f*hc.red()),
                int((1-f)*lc.green() + f*hc.green()),
                int((1-f)*lc.blue()  + f*hc.blue()) )

class BckgrdColorForNumber(ColorForNumber):
    def __init__(self):
        ColorForNumber.__init__(self)
        self.addColor(             0.0, QColor(  0,   0,   0))
        self.addColor(             1.0, QColor(  0,   0, 128))
        self.addColor(   math.pow(2,6), QColor(  0, 255,   0))
        self.addColor(  math.pow(2,14), QColor(255, 255,   0))
        self.addColor(math.pow(2,16)-1, QColor(255,   0,   0))

    def getFrgdColor(self, color):
        if color.valueF() < 0.6:
            return QColor(255, 255, 255)
        else:
            return QColor(0, 0, 0)
    
class MixerNode(QAbstractSlider):
    nodeValueChanged = pyqtSignal(tuple)
    def __init__(self, input, output, value, max, muted, inverted, parent, matrix_obj):
        QAbstractSlider.__init__(self, parent)
        #log.debug("MixerNode.__init__( %i, %i, %i, %i, %s )" % (input, output, value, max, str(parent)) )

        # Store a direct link back to the underlying matrix object so the mute
        # and invert interfaces can be easily found.  By the time the matrix 
        # has been set into the full widget hierarchy, its parent is unlikely
        # to still be the top-level matrix object.
        self.matrix_obj = matrix_obj;

        self.pos = QtCore.QPointF(0, 0)
        self.input = input
        self.output = output
        self.setOrientation(Qt.Vertical)
        if max == -1:
            max = pow(2, 16)-1
        self.setRange(0, max)
        self.setValue(int(value))
        self.valueChanged.connect(self.internalValueChanged)

        self.setSmall(False)

        self.bgcolors = BckgrdColorForNumber()

        self.setContextMenuPolicy(Qt.ActionsContextMenu)
        self.mapper = QtCore.QSignalMapper(self)
        self.mapper.mapped['QString'].connect(self.directValues)

        self.spinbox = QDoubleSpinBox(self)
        self.spinbox.setRange(-40, 12)
        self.spinbox.setSuffix(" dB")
        if value != 0:
            self.spinbox.setValue(toDBvalue(value))            

        self.spinbox.valueChanged.connect(self.directValues)
        action = QWidgetAction(self)
        action.setDefaultWidget(self.spinbox)
        self.addAction(action)

        for text in ["3 dB", "0 dB", "-3 dB", "-20 dB", "-inf dB"]:
            action = QAction(text, self)
            action.triggered.connect(self.mapper.map)
            self.mapper.setMapping(action, text)
            self.addAction(action)

        # Only show the mute menu item if a value has been supplied
        self.mute_action = None
        if (muted != None):
            action = QAction(text, self)
            action.setSeparator(True)
            self.addAction(action)
            self.mute_action = QAction("Mute", self)
            self.mute_action.setCheckable(True)
            self.mute_action.setChecked(muted)
            self.mute_action.triggered.connect(self.mapper.map)
            self.mapper.setMapping(self.mute_action, "Mute")
            self.addAction(self.mute_action)

        # Similarly, only show a phase inversion menu item if in use
        self.inv_action = None
        if (inverted != None):
            if (muted == None):
                action = QAction(text, self)
                action.setSeparator(True)
                self.addAction(action)
            self.inv_action = QAction("Invert", self)
            self.inv_action.setCheckable(True)
            self.inv_action.setChecked(inverted)
            self.inv_action.triggered.connect(self.mapper.map)
            self.mapper.setMapping(self.inv_action, "Invert")
            self.addAction(self.inv_action)

    def directValues(self,text):
        #log.debug("MixerNode.directValues( '%s' )" % text)
        if text == "Mute":
            #log.debug("Mute %d" % self.mute_action.isChecked())
            self.update()
            self.matrix_obj.mutes_interface.setValue(self.output, self.input, self.mute_action.isChecked())
        elif text == "Invert":
            #log.debug("Invert %d" % self.inv_action.isChecked())
            self.update()
            self.matrix_obj.inverts_interface.setValue(self.output, self.input, self.inv_action.isChecked())
        else:
            text = str(text).split(" ")[0].replace(",",".")
            n = fromDBvalue(float(text))
            #log.debug("  linear value: %g" % n)
            self.setValue(n)

    def mousePressEvent(self, ev):
        if ev.buttons() & Qt.LeftButton:
            self.pos = ev.posF() if ffado_pyqt_version == 4 else ev.localPos()
            self.tmpvalue = self.value()
            ev.accept()
            #log.debug("MixerNode.mousePressEvent() %s" % str(self.pos))

    def mouseMoveEvent(self, ev):
        if hasattr(self, "tmpvalue") and self.pos is not QtCore.QPointF(0, 0):
            newpos = ev.posF() if ffado_pyqt_version == 4 else ev.localPos()
            change = newpos.y() - self.pos.y()
            #log.debug("MixerNode.mouseReleaseEvent() change %s" % (str(change)))
            self.setValue( self.tmpvalue - math.copysign(pow(abs(change), 2), change) )
            ev.accept()

    def mouseReleaseEvent(self, ev):
        if hasattr(self, "tmpvalue") and self.pos is not QtCore.QPointF(0, 0):
            newpos = ev.posF() if ffado_pyqt_version == 4 else ev.localPos()
            change = newpos.y() - self.pos.y()
            #log.debug("MixerNode.mouseReleaseEvent() change %s" % (str(change)))
            self.setValue( self.tmpvalue - math.copysign(pow(abs(change), 2), change) )
            self.pos = QtCore.QPointF(0, 0)
            del self.tmpvalue
            ev.accept()

    # Wheel event is mainly for scrolling inside the mixer window
    #   Additionnaly press Control key for wheel controling the values
    def wheelEvent (self, ev):
        if (ev.modifiers() & Qt.ControlModifier):
            tmpvalue = self.value()
            change = ev.delta()/8
            self.setValue( tmpvalue + math.copysign(pow(abs(change), 2), change) )
            ev.accept()
        else:
            ev.ignore()

    def paintEvent(self, ev):
        p = QPainter(self)
        rect = self.rect()
        v = self.value()
        if (self.mute_action!=None and self.mute_action.isChecked()):
            color = QColor(64, 64, 64)
        else:
            color = self.bgcolors.getColor(v)
        p.fillRect(rect, color)

        if self.small:
            return

        p.setPen(self.bgcolors.getFrgdColor(color))

        lv=decimal.Decimal('-Infinity')
        if v != 0:
            lv = toDBvalue(v)
            #log.debug("new value is %g dB" % lv)
        text = "%.2g dB" % lv
        if v == 0:
            symb_inf = u"\u221E"
            text = "-" + symb_inf + " dB"
        if ffado_python3 or ffado_pyqt_version == 5:
            # Python3 uses native python UTF strings rather than QString.
            # This therefore appears to be the correct way to display this
            # UTF8 string, but testing may prove otherwise.
            p.drawText(rect, Qt.AlignCenter, text)
        else:
            p.drawText(rect, Qt.AlignCenter, QString.fromUtf8(text))
        if (self.inv_action!=None and self.inv_action.isChecked()):
            if ffado_python3 or ffado_pyqt_version == 5:
                # Refer to the comment about about Python UTF8 strings.
                p.drawText(rect, Qt.AlignLeft|Qt.AlignTop, " ϕ")
            else:
                p.drawText(rect, Qt.AlignLeft|Qt.AlignTop, QString.fromUtf8(" ϕ"))

    def internalValueChanged(self, value):
        #log.debug("MixerNode.internalValueChanged( %i )" % value)
        if value != 0:
            dB = toDBvalue(value)
            if self.spinbox.value() is not dB:
                self.spinbox.setValue(dB)
        self.nodeValueChanged.emit((self.input, self.output, value))
        self.update()

    def setSmall(self, small):
        self.small = small
        if small:
            self.setMinimumSize(10, 10)
        else:
            fontmetrics = self.fontMetrics()
            self.setMinimumSize(fontmetrics.boundingRect("-0.0 dB").size()*1.1)
        self.update()

class MixerChannel(QWidget):
    hide = pyqtSignal(int, bool, name='hide')
    def __init__(self, number, parent=None, name="", smallFont=False):
        QWidget.__init__(self, parent)
        layout = QGridLayout(self)
        self.number = number
        self.name = name
        self.lbl = QLabel(self)
        self.lbl.setAlignment(Qt.AlignCenter)
        if (smallFont):
            font = self.lbl.font()
            font.setPointSize(int(font.pointSize()/1.5 + 0.5))
            self.lbl.setFont(font)
        layout.addWidget(self.lbl, 0, 0, 1, 2)
        self.hideChannel(False)

        self.setContextMenuPolicy(Qt.ActionsContextMenu)

        action = QAction("Make this channel small", self)
        action.setCheckable(True)
        action.triggered.connect(self.hideChannel)
        self.addAction(action)

    def hideChannel(self, hide):
        if hide:
            self.lbl.setText("%i" % (self.number+1));
        else:
            self.lbl.setText(self.name)
        self.hide.emit(self.number, hide)
        self.update()

# Matrix view widget
class MatrixControlView(QWidget):
    valueChanged = pyqtSignal([tuple])
    def __init__(self, servername, basepath, parent=None, sliderMaxValue=-1, mutespath=None, invertspath=None, smallFont=False, shortname=False, shortcolname="Ch", shortrowname="Ch", transpose=False):
        QWidget.__init__(self, parent)

        if not ffado.config.bypassdbus:
            self.bus = dbus.SessionBus()
            self.dev = self.bus.get_object(servername, basepath)
            self.interface = dbus.Interface(self.dev, dbus_interface="org.ffado.Control.Element.MatrixMixer")

        self.transpose = transpose
        if (transpose):
            self.shortcolname = shortrowname
            self.shortrowname = shortcolname
            if ffado.config.bypassdbus:
                self.cols = 2
                self.rows = 2
            else:
                self.cols = self.interface.getRowCount()
                self.rows = self.interface.getColCount()
        else:
            self.shortcolname = shortcolname
            self.shortrowname = shortrowname
            if ffado.config.bypassdbus:
                self.cols = 2
                self.rows = 2
            else:
                self.cols = self.interface.getColCount()
                self.rows = self.interface.getRowCount()

        log.debug("Mixer has %i rows and %i columns" % (self.rows, self.cols))

        self.mutes_dev = None
        self.mutes_interface = None
        if not ffado.config.bypassdbus and (mutespath != None):
            self.mutes_dev = self.bus.get_object(servername, mutespath)
            self.mutes_interface = dbus.Interface(self.mutes_dev, dbus_interface="org.ffado.Control.Element.MatrixMixer")

        self.inverts_dev = None
        self.inverts_interface = None
        if not ffado.config.bypassdbus and (invertspath != None):
            self.inverts_dev = self.bus.get_object(servername, invertspath)
            self.inverts_interface = dbus.Interface(self.inverts_dev, dbus_interface="org.ffado.Control.Element.MatrixMixer")

        layout = QGridLayout(self)
        layout.setSizeConstraint(QLayout.SetNoConstraint);
        self.setLayout(layout)

        self.rowHeaders = []
        self.columnHeaders = []
        self.items = []
        self.shortname = shortname

        # Add row/column headers, but only if there's more than one 
        # row/column
        if (self.cols > 1):
            for i in range(self.cols):
                ch = MixerChannel(i, self, self.getColName(i, self.shortname), smallFont)
                ch.hide.connect(self.hideColumn)
                layout.addWidget(ch, 0, i+1)
                self.columnHeaders.append( ch )
            layout.setRowStretch(0, 0)
            layout.setRowStretch(1, 10)
        if (self.rows > 1):
            for i in range(self.rows):
                ch = MixerChannel(i, self, self.getRowName(i, self.shortname), smallFont)
                ch.hide.connect(self.hideRow)
                layout.addWidget(ch, i+1, 0)
                self.rowHeaders.append( ch )

        # Add node-widgets
        for i in range(self.rows):
            self.items.append([])
            for j in range(self.cols):
                if (transpose):
                    mute_value = None
                    if (self.mutes_interface != None):
                        mute_value = self.mutes_interface.getValue(j,i)
                    inv_value = None
                    if (self.inverts_interface != None):
                        inv_value = self.inverts_interface.getValue(j,i)
                    if ffado.config.bypassdbus:
                        val = 0
                    else:
                        val = self.interface.getValue(j,i)
                    node = MixerNode(i, j, val, sliderMaxValue, mute_value, inv_value, self, self)
                else:
                    mute_value = None
                    if (self.mutes_interface != None):
                        mute_value = self.mutes_interface.getValue(i,j)
                    inv_value = None
                    if (self.inverts_interface != None):
                        inv_value = self.inverts_interface.getValue(i,j)
                    if ffado.config.bypassdbus:
                        val = 0
                    else:
                        val = self.interface.getValue(i,j)
                    node = MixerNode(j, i, val, sliderMaxValue, mute_value, inv_value, self, self)
                if (smallFont):
                    font = node.font()
                    font.setPointSize(font.pointSize()/1.5)
                    node.setFont(font)
                self.nodeConnect(node)
                layout.addWidget(node, i+1, j+1)
                self.items[i].append(node)

        self.hiddenRows = []
        self.hiddenCols = []

    def nodeConnect(self, node):
        node.nodeValueChanged.connect(self.valueChangedFn)

    def nodeDisconnect(self, node):
        node.nodeValueChanged.disconnect(self.valueChangedFn)

    def checkVisibilities(self):
        for x in range(len(self.items)):
            for y in range(len(self.items[x])):
                self.items[x][y].setSmall(
                        (x in self.hiddenRows)
                        | (y in self.hiddenCols)
                        )

    def hideColumn(self, column, hide):
        if hide:
            self.hiddenCols.append(column)
        else:
            self.hiddenCols.remove(column)
        self.checkVisibilities()

    def hideRow(self, row, hide):
        if hide:
            self.hiddenRows.append(row)
        else:
            self.hiddenRows.remove(row)
        self.checkVisibilities()

    # Columns and rows
    def getColName(self, i, shortname):
        if ffado.config.bypassdbus:
            return 'col ' + str(i)
        if (self.transpose):
            name = self.interface.getRowName(i)
        else:
            name = self.interface.getColName(i)
        self.shortname = shortname
        if (shortname or (name == '')):
            number = " %d" % (i+1)
            name = self.shortcolname + number
        return name

    def getRowName(self, j, shortname):
        if ffado.config.bypassdbus:
            return 'row ' + str(j)
        if (self.transpose):
            name = self.interface.getColName(j)
        else:
            name = self.interface.getRowName(j)
        self.shortname = shortname
        if (shortname or (name == '')):
            number = " %d" % (j+1)
            name = self.shortrowname + number
        return name

    def valueChangedFn(self, n):
        #log.debug("MatrixNode.valueChangedFn( %s )" % str(n))
        if not ffado.config.bypassdbus:
            self.interface.setValue(n[1], n[0], n[2])
        self.valueChanged.emit(n)
        
    # Update when routing is modified
    def updateRouting(self):
        if (self.cols > 1):
            for i in range(self.cols):
                last_name = self.columnHeaders[i].lbl.text()
                col_name = self.getColName(i, self.shortname)
                if last_name != col_name:
                    #log.debug("MatrixControlView.updateRouting( %s )" % str(col_name))
                    self.columnHeaders[i].name = col_name
                    self.columnHeaders[i].lbl.setText(col_name)
      
        if (self.rows > 1):
            for j in range(self.rows):
                last_name = self.rowHeaders[j].lbl.text()
                row_name = self.getRowName(j, self.shortname)
                if last_name != row_name:
                    #log.debug("MatrixControlView.updateRouting( %s )" % str(row_name))
                    self.rowHeaders[j].name = row_name
                    self.rowHeaders[j].lbl.setText(row_name)

    def updateValues(self, n):
        nbitems = len(n) // 3
        for i in range(nbitems):
            n_0 = n[3*i]    
            n_1 = n[3*i+1]   
            n_2 = n[3*i+2] 
            self.nodeDisconnect(self.items[n_0][n_1])
            self.items[n_0][n_1].setValue(n_2)
            self.nodeConnect(self.items[n_0][n_1])

    def refreshValues(self):
        if ffado.config.bypassdbus:
            return
        for x in range(len(self.items)):
            for y in range(len(self.items[x])):
                val = self.interface.getValue(x,y)
                if (self.transpose):
                    self.items[y][x].setValue(val)
                    self.items[y][x].internalValueChanged(val)
                else:
                    self.items[x][y].setValue(val)
                    self.items[x][y].internalValueChanged(val)

    def saveSettings(self, indent):
        matrixSaveString = []
        matrixSaveString.append('%s  <row_number>\n' % indent)
        matrixSaveString.append('%s    %d\n' % (indent, self.rows))        
        matrixSaveString.append('%s  </row_number>\n' % indent)
        matrixSaveString.append('%s  <col_number>\n' % indent)
        matrixSaveString.append('%s    %d\n' % (indent, self.cols))        
        matrixSaveString.append('%s  </col_number>\n' % indent)
        matrixSaveString.append('%s  <coefficients>\n' % indent)
        for i in range(self.rows):
            line = '%s    ' % indent
            for j in range(self.cols):
                line += '%d ' % self.interface.getValue(i,j)
            line += '\n'
            matrixSaveString.append(line)        
        matrixSaveString.append('%s  </coefficients>\n' % indent)
        if (self.mutes_interface != None):
            matrixSaveString.append('%s  <mutes>\n' % indent)
            for i in range(self.rows):
                line = '%s    ' % indent
                for j in range(self.cols):
                    line += '%d ' % self.mutes_interface.getValue(i,j)
                line += '\n'
                matrixSaveString.append(line)        
            matrixSaveString.append('%s  </mutes>\n' % indent)

        if (self.inverts_interface != None):
            matrixSaveString.append('%s  <inverts>\n' % indent)
            for i in range(self.rows):
                line = '%s    ' % indent
                for j in range(self.cols):
                    line += '%d ' % self.inverts_interface.getValue(i,j)
                line += '\n'
                matrixSaveString.append(line)        
            matrixSaveString.append('%s  </inverts>\n' % indent)

        return matrixSaveString

    def readSettings(self, readMatrixString, transpose_coeff):
        if readMatrixString[0].find("<row_number>") == -1:
            log.debug("Number of matrix rows must be specified")
            return False
        if readMatrixString[2].find("</row_number>") == -1:
            log.debug("Non-conformal xml file")
            return False
        n_rows = int(readMatrixString[1])

        if readMatrixString[3].find("<col_number>") == -1:
            log.debug("Number of matrix columns must be specified")
            return False
        if readMatrixString[5].find("</col_number>") == -1:
            log.debug("Non-conformal xml file")
            return False
        n_cols = int(readMatrixString[4])

        if transpose_coeff:
            if n_rows > self.cols:
                n_rows = self.cols
            if n_cols > self.rows:
                n_cols = self.rows
        else:
            if n_rows > self.rows:
                n_rows = self.rows
            if n_cols > self.cols:
                n_cols = self.cols
        log.debug("Setting %d rows and %d columns coefficients" % (n_rows, n_cols))

        try:
            idxb = readMatrixString.index('<coefficients>')
            idxe = readMatrixString.index('</coefficients>')
        except Exception:
            log.debug("No mixer matrix coefficients specified")
            idxb = -1
            idxe = -1
        if idxb >= 0:
            if idxe < idxb + n_rows + 1:
                log.debug("Incoherent number of rows in coefficients")
                return False
            i = 0
            for s in readMatrixString[idxb+1:idxb + n_rows + 1]:
                coeffs = s.split()
                if len(coeffs) < n_cols:
                    log.debug("Incoherent number of columns in coefficients")
                    return False
                j = 0
                for c in coeffs[0:n_cols]:
                    if transpose_coeff:
                        self.interface.setValue(j, i, int(c))
                    else:
                        self.interface.setValue(i, j, int(c))
                    j += 1
                i += 1
                del coeffs

        try:
            idxb = readMatrixString.index('<mutes>')
            idxe = readMatrixString.index('</mutes>')
        except Exception:
            log.debug("No mixer mute coefficients specified")
            idxb = -1
            idxe = -1
        if idxb >= 0:
            if idxe < idxb + n_rows + 1:
                log.debug("Incoherent number of rows in mute")
                return false
            i = 0
            for s in readMatrixString[idxb+1:idxb + n_rows + 1]:
                coeffs = s.split()
                if len(coeffs) < n_cols:
                    log.debug("Incoherent number of columns in mute")
                    return false
                j = 0
                for c in coeffs[0:n_cols]:
                    if transpose_coeff:
                        self.mutes_interface.setValue(j, i, int(c))
                    else:
                        self.mutes_interface.setValue(i, j, int(c))
                    j += 1
                i += 1
                del coeffs

        try:
            idxb = readMatrixString.index('<inverts>')
            idxe = readMatrixString.index('</inverts>')
        except Exception:
            log.debug("No mixer inverts coefficients specified")
            idxb = -1
            idxe = -1
        if idxb >= 0:
            if idxe < idxb + n_rows + 1:
                log.debug("Incoherent number of rows in inverts")
                return false
            i = 0
            for s in readMatrixString[idxb+1:idxb + n_rows + 1]:
                coeffs = s.split()
                if len(coeffs) < n_cols:
                    log.debug("Incoherent number of columns in inverts")
                    return false
                j = 0
                for c in coeffs[0:n_cols]:
                    if transpose_coeff:
                        self.inverts_interface.setValue(j, i, int(c))
                    else:
                        self.inverts_interface.setValue(i, j, int(c))
                    j += 1
                i += 1
                del coeffs

        self.refreshValues()
        return True

class VolumeSlider(QSlider):
    sliderChanged = pyqtSignal(tuple)
    def __init__(self, In, Out, value, parent):
        QSlider.__init__(self, QtCore.Qt.Vertical, parent)

        self.setTickPosition(QSlider.TicksBothSides)
        v_min = 10.0*toDBvalue(0)
        v_max = 10.0*toDBvalue(65536)
        self.setTickInterval(int((v_max-v_min)/10))
        self.setMinimum(int(v_min))
        self.setMaximum(int(v_max))
        self.setSingleStep(1)
        self.sliderSetValue(value)
        self.In = In
        self.Out = Out
        self.valueChanged.connect(self.sliderValueChanged)

    def sliderSetValue(self, value):
        #log.debug("Volume slider value changed( %i )" % value)
        v = 10.0*toDBvalue(value)
        #log.debug("Volume slider value changed(dB: %g )" % (0.1*v))
        self.setValue(int(v))

    def sliderReadValue(self, value):
        return fromDBvalue(0.1*value)

    # Restore absolute value from DB
    # Emit signal for further use, especially for matrix view
    def sliderValueChanged(self, value):
        value = fromDBvalue(0.1*value)
        self.sliderChanged.emit((self.In, self.Out, value))
        self.update()


class VolumeSliderValueInfo(QLineEdit):
    def __init__(self, In, Out, value, parent):
        QLineEdit.__init__(self, parent)

        self.setReadOnly(True)
        self.setAlignment(Qt.AlignCenter)
        self.setAutoFillBackground(True)
        self.setFrame(False)

        self.sliderSetMinimalDim()

        self.bgcolors = BckgrdColorForNumber()

        self.sliderSetValue(value)

    def sliderSetMinimalDim(self):
        fontmetrics = self.fontMetrics()
        self.setMinimumSize(fontmetrics.boundingRect("-00.0 dB").size()*1.1)
        
    def sliderSetValue(self, value):
        color = self.bgcolors.getColor(value)
        palette = self.palette()
        palette.setColor(QPalette.Active, QPalette.Base, color)
        palette.setColor(QPalette.Active, QPalette.Text, self.bgcolors.getFrgdColor(color))
        self.setPalette(palette)

        v = round(toDBvalue(value),1)
        if (v > -40):
            text = "%.1f dB" % v
        else:
            symb_inf = u"\u221E"
            text = "-" + symb_inf + " dB"

        self.setText(text)
        
class BalanceSlider(QSlider):
    sliderChanged = pyqtSignal(tuple)
    def __init__(self, In, Out, value, parent):
        QSlider.__init__(self, QtCore.Qt.Horizontal, parent)

        v_min = -50
        v_max = 50
        self.setTickPosition(QSlider.TicksBothSides)
        self.setTickInterval((v_max-v_min)/2)
        self.setMinimum(v_min)
        self.setMaximum(v_max)
        self.setSingleStep(1)
        self.In = In
        self.Out = Out
        self.sliderSetValue(value)
        self.valueChanged.connect(self.sliderValueChanged)

    def sliderSetValue(self, value):
        #log.debug("Balance fader value set( %d, %d, %f )" % (self.In, self.Out, value))
        v = int(round(50.0*value, 2))
        self.setValue(v)

    def sliderReadValue(self):
        return float(round(self.value()/50.0, 2))

    def sliderValueChanged(self, value):
        value = float(round(self.value()/50.0, 2))
        #log.debug("Balance fader value changed( %d, %d, %f )" % (self.In, self.Out, value))
        self.sliderChanged.emit((self.In, self.Out, value))

# Slider view widget
class SliderControlView(QWidget):
    valueChanged = pyqtSignal(tuple)
    def __init__(self, parent, servername, basepath, rule="Columns_are_inputs", shortname=False, shortinname="Ch", shortoutname="Ch", stereochannels = []):
        QWidget.__init__(self, parent)

        if not ffado.config.bypassdbus:
            self.bus = dbus.SessionBus()
            self.dev = self.bus.get_object(servername, basepath)
            self.interface = dbus.Interface(self.dev, dbus_interface="org.ffado.Control.Element.MatrixMixer")

        self.rule = rule
        self.shortname = shortname
        self.shortinname = shortinname
        self.shortoutname = shortoutname

        self.stereochannels = stereochannels

        self.out = []
        self.nbIn = self.getNbIn()
        self.nbOut = self.getNbOut()
        self.outmatrix = []

        k = 0
        for i in range(self.nbOut):
            widget = QWidget(parent)
            v_layout = QVBoxLayout(widget)
            v_layout.setAlignment(Qt.AlignCenter)
            widget.setLayout(v_layout)
            self.out.append(widget)

            self.out[i].is_stereo = False
            self.out[i].out_1 = k
            self.outmatrix.append(i)
            self.out[i].outname = "Out %d" % (k+1)
            if k in self.stereochannels:
                self.out[i].is_stereo = True
                self.out[i].outname += "+%d" % (k+2)
                self.outmatrix.append(i)

            self.out[i].lbl = []

            # Mixer/Out info label
            if (self.nbOut > 1):
                lbl = QLabel(widget)
                lbl.setText(self.getOutName(i, self.shortname))
                lbl.setAlignment(Qt.AlignCenter)
                v_layout.addWidget(lbl)
                self.out[i].lbl.append(lbl)

            h_layout_wid = QWidget(widget)
            h_layout = QHBoxLayout(h_layout_wid)
            h_layout.setAlignment(Qt.AlignCenter)
            h_layout_wid.setLayout(h_layout)
            v_layout.addWidget(h_layout_wid)
            self.out[i].volume = []
            self.out[i].svl = []
            self.out[i].balance = []

            for j in range(self.nbIn):
                h_v_layout_wid = QWidget(h_layout_wid)
                h_v_layout = QVBoxLayout(h_v_layout_wid)
                h_v_layout.setAlignment(Qt.AlignCenter)
                h_v_layout_wid.setLayout(h_v_layout)
                h_layout.addWidget(h_v_layout_wid)

                # Mixer/In info label
                if (self.nbIn > 1):
                    lbl = QLabel(h_v_layout_wid)
                    lbl.setText(self.getInName(j, self.shortname))
                    lbl.setAlignment(Qt.AlignCenter)
                    h_v_layout.addWidget(lbl)
                    self.out[i].lbl.append(lbl)

                h_v_h_layout_wid = QWidget(h_v_layout_wid)
                h_v_h_layout = QHBoxLayout(h_v_h_layout_wid)
                h_v_h_layout.setAlignment(Qt.AlignCenter)
                h_v_h_layout_wid.setLayout(h_v_h_layout)
                h_v_layout.addWidget(h_v_h_layout_wid)

                volume = VolumeSlider(j, i, self.getVolumeValue(j,i), h_v_h_layout_wid)
                h_v_h_layout.addWidget(volume)
                self.out[i].volume.append(volume)
                self.volumeConnect(volume)

                # Volume slider info
                svl = VolumeSliderValueInfo(j, i, self.getVolumeValue(j,i), h_v_layout_wid)
                h_v_layout.addWidget(svl)
                self.out[i].svl.append(svl)

                # Balance fader
                if self.out[i].is_stereo:
                    balance = BalanceSlider(j, i, self.getBalanceValue(j,i), h_v_layout_wid)
                    h_v_layout.addWidget(balance)
                    self.out[i].balance.append(balance)
                    self.balanceConnect(balance)
            k += 1
            if self.out[i].is_stereo:
                k += 1

    def volumeConnect(self, volume):
        volume.sliderChanged.connect(self.valueChangedVolume)

    def volumeDisconnect(self, volume):
        volume.sliderChanged.disconnect(self.valueChangedVolume)

    def balanceConnect(self, balance):
        balance.sliderChanged.connect(self.valueChangedBalance)

    def balanceDisconnect(self, balance):
        balance.sliderChanged.disconnect(self.valueChangedBalance)

    def getNbIn(self):
        if ffado.config.bypassdbus:
            return 2
        if (self.rule == "Columns_are_inputs"):
            return self.interface.getColCount()
        else:
            return self.interface.getRowCount()
        
    def getNbOut(self):
        if ffado.config.bypassdbus:
            return 2
        if (self.rule == "Columns_are_inputs"):
            nbout = self.interface.getRowCount()
        else:
            nbout = self.interface.getColCount()
        return nbout-len(self.stereochannels)
        
    def getVolumeValue(self, In, i):
        if ffado.config.bypassdbus:
            return 1
        Out = self.out[i].out_1
        if (self.rule == "Columns_are_inputs"):
            vl = self.interface.getValue(Out, In)           
        else:
            vl = self.interface.getValue(In, Out)
        if (self.out[i].is_stereo):
            if (self.rule == "Columns_are_inputs"):
                vr = self.interface.getValue(Out+1, In)           
            else:
                vr = self.interface.getValue(In, Out+1)
            return getStereoVolume(vl, vr)
        else:
            return vl;     

    def getBalanceValue(self, In, i):
        if ffado.config.bypassdbus:
            return 0.5
        Out = self.out[i].out_1
        if (self.rule == "Columns_are_inputs"):
            vl = self.interface.getValue(Out, In)           
            vr = self.interface.getValue(Out+1, In)           
        else:
            vl = self.interface.getValue(In, Out)
            vr = self.interface.getValue(In, Out+1)
        return getStereoBalance(vl, vr)

    def setValue(self, In, Out, val):
        if ffado.config.bypassdbus:
            return
        if (self.rule == "Columns_are_inputs"):
            return self.interface.setValue(Out, In, val)           
        else:
            return self.interface.setValue(In, Out, val)            

    def updateValues(self, n):
        nbitems = len(n) // 3
        for j in range(nbitems):
            n_0 = n[3*j]    
            n_1 = n[3*j+1]   
            n_2 = n[3*j+2] 
            i = self.outmatrix[n_1]
            if (self.out[i].is_stereo):
                v = self.getVolumeValue(n_0, i)
                self.volumeDisconnect(self.out[i].volume[n_0])
                self.balanceDisconnect(self.out[i].balance[n_0])
                self.out[i].volume[n_0].sliderSetValue(v)
                self.out[i].svl[n_0].sliderSetValue(v)
                b = self.getBalanceValue(n_0, i)       
                # log.debug("update Value (%d %d %d %f)" % (n_0, i, v, b))
                self.out[i].balance[n_0].sliderSetValue(b)
                self.volumeConnect(self.out[i].volume[n_0])
                self.balanceConnect(self.out[i].balance[n_0])
            else:
                v = n_2
                # log.debug("update Value (%d %d %d)" % (n_0, i, v))
                self.volumeDisconnect(self.out[i].volume[n_0])
                self.out[i].volume[n_0].sliderSetValue(v)
                self.out[i].svl[n_0].sliderSetValue(v)
                self.volumeConnect(self.out[i].volume[n_0])
        
    def valueChangedVolume(self, n):
        #log.debug("VolumeSlider.valueChanged( %s )" % str(n))
        v = n[2]
        n1 = self.out[n[1]].out_1
        if (self.out[n[1]].is_stereo):
            b = self.out[n[1]].balance[n[0]].value()/50.0
            vl = int(getVolumeLeft(v, b))
            self.setValue(n[0], n1, vl)
            n2 = n1+1
            vr = int(getVolumeRight(v, b))
            self.setValue(n[0], n2, vr)
            n_t = (n[0], n1, vl, n[0], n2, vr)
            self.valueChanged.emit(n_t)
        else:
            self.setValue(n[0], n1, v)
            n_t = (n[0], n1, v)
            self.valueChanged.emit(n_t)
        self.out[n[1]].svl[n[0]].sliderSetValue(v)

    def valueChangedBalance(self, n):
        #log.debug("BalanceSlider.valueChanged( %s )" % str(n))
        n1 = self.out[n[1]].out_1
        v = fromDBvalue(0.1*self.out[n[1]].volume[n[0]].value())
        b = n[2]
        vl = int(getVolumeLeft(v, b))
        self.setValue(n[0], n1, vl)
        n2 = n1+1
        vr = int(getVolumeRight(v, b))
        self.setValue(n[0], n2, vr)
        n_t = (n[0], n1, vl, n[0], n2, vr)
        self.valueChanged.emit(n_t)

    def getOutName(self, i, shortname):
        self.shortname = shortname
        k = self.out[i].out_1
        if (shortname):
            if (self.out[i].is_stereo):
                number = " %d+%d" % (k+1, k+2)
            else:
                number = " %d" % (k+1)
            name = self.shortoutname + number
            return name
        else:
            if ffado.config.bypassdbus:
                return 'OutName ' + str(i)
            if (self.rule == "Columns_are_inputs"):                
                if (self.out[i].is_stereo):
                    name = self.interface.getRowName(k).replace('\n','')+" + "+self.interface.getRowName(k+1).replace('\n','')
                else:
                    name = self.interface.getRowName(k).replace('\n','')
                return name
            else:
                if (self.out[i].is_stereo):
                    name = self.interface.getColName(k).replace('\n','')+" + "+self.interface.getColName(k+1).replace('\n','')
                else:
                    name = self.interface.getColName(k).replace('\n','')
                return name

    def getInName(self, j, shortname):
        self.shortname = shortname
        if (shortname):
            number = " %d" % (j+1)
            name = self.shortinname + number
            return name
        else:
            if ffado.config.bypassdbus:
                return 'InName ' + str(j)
            if (self.rule == "Columns_are_inputs"):
                return self.interface.getColName(j)            
            else:
                return self.interface.getRowName(j)            

    # Update when routing is modified
    def updateRouting(self):
        for i in range(self.nbOut):
            if (self.nbOut > 1):
                last_name = self.out[i].lbl[0].text()
                out_name = self.getOutName(i, self.shortname)
                if last_name != out_name:
                    #log.debug("SliderControlView.updateRouting( %s )" % str(out_name))
                    self.out[i].lbl[0].setText(out_name)
            if (self.nbIn > 1):
                for j in range(self.nbIn):
                    last_name = self.out[i].lbl[j+1].text()
                    in_name = self.getInName(j, self.shortname)
                    if last_name != in_name:
                        #log.debug("SliderControlView.updateRouting( %s )" % str(in_name))
                        self.out[i].lbl[j+1].setText(in_name)

    def refreshValues(self):
        for n_out in range(self.nbOut):
            for n_in in range(self.nbIn):
                i = self.outmatrix[n_out]
                v = self.getVolumeValue(n_in, i)
                if (self.out[i].is_stereo):
                    self.volumeDisconnect(self.out[i].volume[n_in])
                    self.out[i].volume[n_in].sliderSetValue(v)
                    self.out[i].svl[n_in].sliderSetValue(v)
                    b = self.getBalanceValue(n_in, i)       
                    # log.debug("update Value (%d %d %d %f)" % (n_0, i, v, b))
                    self.out[i].balance[n_in].sliderSetValue(b)
                    self.volumeConnect(self.out[i].volume[n_in])
                else:
                    # log.debug("update Value (%d %d %d)" % (n_0, i, v))
                    self.out[i].volume[n_in].sliderSetValue(v)
                    self.out[i].svl[n_in].sliderSetValue(v)

    def saveSettings(self, indent):
        if ffado.config.bypassdbus:
            rows = 2
            cols = 2
        else:
            rows = self.interface.getRowCount()
            cols = self.interface.getColCount()
        matrixSaveString = []
        matrixSaveString.append('%s  <row_number>\n' % indent)
        matrixSaveString.append('%s    %d\n' % (indent, rows))        
        matrixSaveString.append('%s  </row_number>\n' % indent)
        matrixSaveString.append('%s  <col_number>\n' % indent)
        matrixSaveString.append('%s    %d\n' % (indent, cols))        
        matrixSaveString.append('%s  </col_number>\n' % indent)
        matrixSaveString.append('%s  <coefficients>\n' % indent)
        for i in range(rows):
            line = '%s    ' % indent
            for j in range(cols):
                line += '%d ' % self.interface.getValue(i,j)
            line += '\n'
            matrixSaveString.append(line)        
        matrixSaveString.append('%s  </coefficients>\n' % indent)

        return matrixSaveString

    def readSettings(self, readMatrixString, transpose_coeff):
        if ffado.config.bypassdbus:
            rows = 2
            cols = 2
        else:
            rows = self.interface.getRowCount()
            cols = self.interface.getColCount()
        if readMatrixString[0].find("<row_number>") == -1:
            log.debug("Number of matrix rows must be specified")
            return False
        if readMatrixString[2].find("</row_number>") == -1:
            log.debug("Non-conformal xml file")
            return False
        n_rows = int(readMatrixString[1])

        if readMatrixString[3].find("<col_number>") == -1:
            log.debug("Number of matrix columns must be specified")
            return False
        if readMatrixString[5].find("</col_number>") == -1:
            log.debug("Non-conformal xml file")
            return False
        n_cols = int(readMatrixString[4])

        if transpose_coeff:
            if n_rows > cols:
                n_rows = cols
            if n_cols > rows:
                n_cols = rows
        else:
            if n_rows > rows:
                n_rows = rows
            if n_cols > cols:
                n_cols = cols
        log.debug("Setting %d rows and %d columns coefficients" % (n_rows, n_cols))

        try:
            idxb = readMatrixString.index('<coefficients>')
            idxe = readMatrixString.index('</coefficients>')
        except Exception:
            log.debug("No mixer matrix coefficients specified")
            idxb = -1
            idxe = -1
        if idxb >= 0:
            if idxe < idxb + n_rows + 1:
                log.debug("Incoherent number of rows in coefficients")
                return False
            i = 0
            for s in readMatrixString[idxb+1:idxb + n_rows + 1]:
                coeffs = s.split()
                if len(coeffs) < n_cols:
                    log.debug("Incoherent number of columns in coefficients")
                    return False
                j = 0
                for c in coeffs[0:n_cols]:
                    if transpose_coeff:
                        self.interface.setValue(j, i, int(c))
                    else:
                        self.interface.setValue(i, j, int(c))
                    j += 1
                i += 1
                del coeffs

        self.refreshValues()
        return True

from functools import partial

class MatrixMixer(QWidget):
    def __init__(self, servername, basepath, parent=None, rule="Columns_are_inputs", sliderMaxValue=-1, mutespath=None, invertspath=None, smallFont=False, taborientation=QTabWidget.West, tabshape=QTabWidget.Triangular):
        QWidget.__init__(self, parent)
        self.servername = servername
        self.basepath = basepath
        self.sliderMaxValue = sliderMaxValue
        self.mutespath = mutespath
        self.invertspath = invertspath
        self.smallFont = smallFont

        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)

        # Mixer view Tool bar
        mxv_set = QToolBar("View settings", self)

        # Here is a hack; the first action button appears to behaves strangely,
        # possibly a PyQt bug (or an unsufficient fair implementation of it)
        # Feel free to remove the next three lines at a time in the future
        hack = QAction(" ", mxv_set)
        hack.setDisabled(True)
        mxv_set.addAction(hack)

        transpose_matrix = QAction("Transpose", mxv_set)
        self.transpose = False
        transpose_matrix.setShortcut('Ctrl+T')
        transpose_matrix.setToolTip("Invert rows and columns in Matrix view")
        mxv_set.addAction(transpose_matrix)
        transpose_matrix.triggered.connect(self.transposeMatrixView)
        mxv_set.addSeparator()

        self.hide_matrix = QAction("Hide matrix", mxv_set)
        self.hide_matrix_bool = False
        mxv_set.addAction(self.hide_matrix)
        self.hide_matrix.triggered.connect(self.hideMatrixView)
        mxv_set.addSeparator()

        self.hide_per_output = QAction("Hide per Output", mxv_set)
        self.hide_per_output_bool = False
        mxv_set.addAction(self.hide_per_output)
        self.hide_per_output.triggered.connect(self.hidePerOutputView)
        mxv_set.addSeparator()

        self.use_short_names = QAction("Short names", mxv_set)
        self.short_names_bool = False
        mxv_set.addAction(self.use_short_names)
        self.use_short_names.setToolTip("Use short or full names for input and output channels")
        self.use_short_names.triggered.connect(self.shortChannelNames)
        mxv_set.addSeparator()

        font_switch_lbl = QLabel(mxv_set)
        font_switch_lbl.setText("Font size ")
        mxv_set.addWidget(font_switch_lbl)
        font_switch = QComboBox(mxv_set)
        font_switch.setToolTip("Labels font size")
        font = font_switch.font()
        for i in range(10):
            font_switch.addItem(" %d " % (font.pointSize()+4-i))
        font_switch.setCurrentIndex(font_switch.findText(" %d " % font.pointSize()))
        mxv_set.addWidget(font_switch)
        mxv_set.addSeparator()
        font_switch.activated.connect(self.changeFontSize)

        self.layout.addWidget(mxv_set)
        self.mxv_set = mxv_set

        # First tab is for matrix view
        # Next are for "per Out" view
        self.tabs = QTabWidget(self)
        self.tabs.setTabPosition(taborientation)
        self.tabs.setTabShape(tabshape)
        self.layout.addWidget(self.tabs)

        # Inputs/Outputs versus rows/columns rule
        self.rule = rule

        # Matrix view tab
        if (rule == "Columns_are_inputs"):
            self.matrix = MatrixControlView(servername, basepath, self, sliderMaxValue, mutespath, invertspath, smallFont, self.short_names_bool, "In", "Out", self.transpose)
        else:
            self.matrix = MatrixControlView(servername, basepath, self, sliderMaxValue, mutespath, invertspath, smallFont, self.short_names_bool, "Out", "In", self.transpose)
        self.matrix.valueChanged.connect(self.matrixControlChanged)

        self.scrollarea_matrix = QScrollArea(self.tabs)
        self.scrollarea_matrix.setWidgetResizable(True)
        self.scrollarea_matrix.setWidget(self.matrix)
        self.tabs.addTab(self.scrollarea_matrix, " Matrix ")

        # Add stereo/mono output choice in tool bar
        if (rule == "Columns_are_inputs"):
            if (self.transpose):
                nb_out_mono = self.matrix.cols
            else:
                nb_out_mono = self.matrix.rows
        else:
            if (self.transpose):
                nb_out_mono = self.matrix.rows
            else:
                nb_out_mono = self.matrix.cols

        stereo_switch_lbl = QLabel(mxv_set)
        stereo_switch_lbl.setText("Stereo: ")
        mxv_set.addWidget(stereo_switch_lbl)

        self.stereo_channels = []

        self.stereo_switch = []
        for i in range(int(nb_out_mono/2)):
            stereo_switch = QPushButton("%d+%d" % (2*i+1, 2*i+2), mxv_set)
            stereo_switch.setToolTip("Set these output channels as stereo")
            stereo_switch.setCheckable(True)
            stereo_switch.clicked.connect(partial(self.switchStereoChannel, i))
            stereo_switch.setMinimumSize(stereo_switch_lbl.fontMetrics().boundingRect("%d+%d" % (nb_out_mono, nb_out_mono)).size()*1.05)
            stereo_switch.setSizePolicy(QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum))
            stereo_switch.is_stereo = False
            mxv_set.addWidget(stereo_switch)
            self.stereo_switch.append(stereo_switch)
        mxv_set.addSeparator()

        # Per out view tabs
        self.perOut = SliderControlView(self, servername, basepath, rule, self.short_names_bool, "In", "Out", self.stereo_channels)
        self.perOut.valueChanged.connect(self.sliderControlChanged)
        for i in range(self.perOut.nbOut):
            self.perOut.out[i].scrollarea = QScrollArea(self.tabs)
            self.perOut.out[i].scrollarea.setWidgetResizable(True)
            self.perOut.out[i].scrollarea.setWidget(self.perOut.out[i])
            self.tabs.addTab(self.perOut.out[i].scrollarea, " %s " % self.perOut.out[i].outname)

    def transposeMatrixView(self):
        self.transpose = not(self.transpose)
        self.tabs.removeTab(0)
        self.scrollarea_matrix.destroy()
        if (self.rule == "Columns_are_inputs"):
            self.matrix = MatrixControlView(self.servername, self.basepath, self, self.sliderMaxValue, self.mutespath, self.invertspath, self.smallFont, self.short_names_bool, "In", "Out", self.transpose)
        else:
            self.matrix = MatrixControlView(self.servername, self.basepath, self, self.sliderMaxValue, self.mutespath, self.invertspath, self.smallFont, self.short_names_bool, "Out", "In", self.transpose)
        self.matrix.valueChanged.connect(self.matrixControlChanged)

        self.scrollarea_matrix = QScrollArea(self.tabs)
        self.scrollarea_matrix.setWidgetResizable(True)
        self.scrollarea_matrix.setWidget(self.matrix)
        self.tabs.insertTab(0, self.scrollarea_matrix, "Matrix")
        self.tabs.setCurrentIndex(0)
        
    def hideMatrixView(self):
        self.hide_matrix_bool = not(self.hide_matrix_bool)
        if (self.hide_matrix_bool):
            self.tabs.removeTab(0)
            self.hide_matrix.setText("Show Matrix")
        else:
            self.tabs.insertTab(0, self.scrollarea_matrix, "Matrix")
            self.tabs.setCurrentIndex(0)
            self.hide_matrix.setText("Hide Matrix")
            
    def hidePerOutputView(self):
        self.hide_per_output_bool = not(self.hide_per_output_bool)
        if (self.hide_per_output_bool):
            index_0 = 1
            if (self.hide_matrix_bool):
                index_0 = 0
            for i in range(self.perOut.nbOut):
                self.tabs.removeTab(index_0)
            self.hide_per_output.setText("Show per Output")
        else:
            for i in range(self.perOut.nbOut):
                self.tabs.insertTab(i+1, self.perOut.out[i].scrollarea, " %s " % self.perOut.out[i].outname)
            self.hide_per_output.setText("Hide per Output")

    # Font size for channel names
    def changeFontSize(self, size):
        font = self.mxv_set.font()
        font.setPointSize(int(size))
        self.mxv_set.setFont(font)

        font = self.tabs.font()
        font.setPointSize(int(size))
        self.tabs.setFont(font)

        font = self.matrix.font()
        font.setPointSize(int(size))
        self.matrix.setFont(font)

        font = self.perOut.font()
        font.setPointSize(int(size))
        self.perOut.setFont(font)

        for i in range(self.perOut.nbOut):
            for j in range(self.perOut.nbIn):
                self.perOut.out[i].svl[j].sliderSetMinimalDim()

    # Allows long name for Mixer/Out and /In to be hidden 
    def shortChannelNames(self):
        checked = not(self.short_names_bool)
        if (self.matrix.cols > 1):
            for i in range(self.matrix.cols):
                self.matrix.columnHeaders[i].name = self.matrix.getColName(i, checked)
                self.matrix.columnHeaders[i].lbl.setText(self.matrix.columnHeaders[i].name)

        if (self.matrix.rows > 1):
            for j in range(self.matrix.rows):
                self.matrix.rowHeaders[j].name = self.matrix.getRowName(j, checked)
                self.matrix.rowHeaders[j].lbl.setText(self.matrix.rowHeaders[j].name)       

        for i in range(self.perOut.nbOut):
            if (self.perOut.nbOut > 1):
                self.perOut.out[i].lbl[0].setText(self.perOut.getOutName(i, checked))
            if (self.perOut.nbIn > 1):
                for j in range(self.perOut.nbIn):
                    self.perOut.out[i].lbl[j+1].setText(self.perOut.getInName(j, checked))

        # Care for hidden columns
        if (self.matrix.cols > 1):
            for i in self.matrix.hiddenCols:
                self.matrix.columnHeaders[i].lbl.setText("%d" % (i+1))
        # Care for hidden rows
        if (self.matrix.rows > 1):
            for j in self.matrix.hiddenRows:
                self.matrix.rowHeaders[j].lbl.setText("%d" % (j+1))

        self.short_names_bool = checked
        if (self.short_names_bool):
            self.use_short_names.setText("Long names")
        else:
            self.use_short_names.setText("Short names")

    # Sliders value change
    #   Care that some recursive process is involved and only stop when exactly same values are involved
    # Matrix view
    def matrixControlChanged(self, n):
        # Update value needed for "per Out" view
        #log.debug("Update per Output( %s )" % str(n))
        nbitems = len(n) // 3
        if (self.rule == "Columns_are_inputs"):
           n_t = n
        else:
            n_t = ()
            for i in range(nbitems):
                n_t += (n[3*i+1], n[3*i], n[3*i+2])

        self.perOut.updateValues(n_t)

    # "per Out" view
    def sliderControlChanged(self, n):
        # Update value needed for matrix view
        #log.debug("Update Matrix( %s )" % str(n))
        nbitems = len(n) // 3
        if (((self.rule == "Columns_are_inputs") and not self.transpose) or ((self.rule != "Columns_are_inputs") and self.transpose)):
            n_t = ()
            for i in range(nbitems):
                n_t += (n[3*i+1], n[3*i], n[3*i+2])
        else:
            n_t = n

        self.matrix.updateValues(n_t)

    def refreshValues(self):
        # Refreshing matrix coefficient should be sufficient,
        #  propagating the changes to perOut view
        self.matrix.refreshValues()

    def switchStereoChannel(self, channel, is_stereo):
        #log.debug(" switching channels %d+%d to stereo/mono" % (2*channel, 2*channel+1))
        self.stereo_switch[channel].is_stereo = self.stereo_switch[channel].isChecked();
        if (self.stereo_switch[channel].is_stereo):
            self.stereo_channels.append(2*channel)
        else:
            self.stereo_channels.remove(2*channel)

        # tab 0 is for matrix except if it is hidden
        index_0 = 1
        if (self.hide_matrix_bool):
            index_0 = 0
        for i in range(self.perOut.nbOut):
            self.tabs.removeTab(index_0)
        self.perOut.destroy()
        self.perOut = SliderControlView(self, self.servername, self.basepath, self.rule, self.short_names_bool, "In", "Out", self.stereo_channels)
        self.perOut.valueChanged.connect(self.sliderControlChanged)
        current = 0
        for i in range(self.perOut.nbOut):
            self.perOut.out[i].scrollarea = QScrollArea(self.tabs)
            self.perOut.out[i].scrollarea.setWidgetResizable(True)
            self.perOut.out[i].scrollarea.setWidget(self.perOut.out[i])
            self.tabs.addTab(self.perOut.out[i].scrollarea, " %s " % self.perOut.out[i].outname)
            if self.perOut.out[i].out_1 == 2*channel:
                current = i

        self.tabs.setCurrentWidget(self.perOut.out[current].scrollarea)

    # Update when routing is modified
    def updateRouting(self):
        self.matrix.updateRouting()
        self.perOut.updateRouting()
        
    def saveSettings(self, indent):
        mixerString = []
        mixerString.append("%s<matrices>\n" % indent)
        mixerString.extend(self.matrix.saveSettings(indent))
        mixerString.append("%s</matrices>\n" % indent)
        mixerString.append("%s<stereo_outputs>\n" % indent)
        mixerString.append("%s  <number>\n" % indent)
        n = len(self.stereo_channels)
        mixerString.append("%s    %d\n" % (indent, n))
        mixerString.append("%s  </number>\n" % indent)
        if n > 0:
            mixerString.append("%s  <channels>\n" % indent)
            for i in self.stereo_channels:
                mixerString.append("%s    %d %d\n" % (indent, i+1, i+2))
            mixerString.append("%s  </channels>\n" % indent)
        mixerString.append("%s</stereo_outputs>\n" % indent)
        return mixerString

    def readSettings(self, readMixerString, transpose_coeff):
        try:
            idxb = readMixerString.index('<matrices>')
            idxe = readMixerString.index('</matrices>')
        except Exception:
            log.debug("No matrices found")
            idxb = -1
            idxe = -1
        if idxb >= 0:
            if idxe > idxb+1:
                readString = []
                for s in readMixerString[idxb+1:idxe]:
                    readString.append(s)
                if self.matrix.readSettings(readString, transpose_coeff):
                    log.debug("Mixer matrices settings modified")
                del readString
        try:
            idx = readMixerString.index('<stereo_outputs>')
        except Exception:
            log.debug("No stereo outputs channels information found")
            idx = -1
        if idx >= 0:
            if readMixerString[idx+1].find('<number>') == -1:
                log.debug("Number of stereo output channels must be specified")
                return False
            n = int(readMixerString[idx+2])
            if n > self.perOut.nbOut // 2:
                log.debug("Incoherent number of stereo channels")
                return False
            if n > 0:
                if readMixerString[idx+3].find('</number>') == -1:
                    log.debug("No </number> tag found")
                    return False
                if readMixerString[idx+4].find('<channels>') == -1:
                    log.debug("No <channels> tag found")
                    return False
                for s in readMixerString[idx+5:idx+5+n]:
                    i = (int(s.split()[0]) - 1)/2
                    self.stereo_switch[i].setChecked(True);
                    self.switchStereoChannel(i, True)
        return True
                    
#
# vim: et ts=4 sw=4 fileencoding=utf8
