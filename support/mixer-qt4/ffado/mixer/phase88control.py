#
# Copyright (C) 2014-2015 by Andras Muranyi
# Copyright (c) 2013 by Takashi Sakamoto (Yamaha GO Control)
# Copyright (C) 2005-2008 by Pieter Palmers
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

# from PyQt4.QtGui import QWidget
from ffado.import_pyqt import *

from math import log10
from ffado.config import *

import logging
log = logging.getLogger('phase88')

class Phase88Control(QWidget):
    def __init__(self,parent = None):
        QWidget.__init__(self,parent)
        
    def initValues(self):

        uicLoad("ffado/mixer/phase88", self)

        self.VolumeControls={
            self.sldInputMasterL:   ['/Mixer/Feature_Volume_1', 1, self.sldInputMasterR, 2, self.linkButtonMaster, self.decibelMasterL, 0], 
            self.sldInputMasterR:   ['/Mixer/Feature_Volume_1', 2, self.sldInputMasterL, 1, self.linkButtonMaster, self.decibelMasterR, 0], 
            self.sldInput1:         ['/Mixer/Feature_Volume_2', 1, self.sldInput2, 2, self.linkButton12, self.decibel1, 0],
            self.sldInput2:         ['/Mixer/Feature_Volume_2', 2, self.sldInput1, 1, self.linkButton12, self.decibel2, 0],
            self.sldInput3:         ['/Mixer/Feature_Volume_3', 1, self.sldInput4, 2, self.linkButton34, self.decibel3, 0],
            self.sldInput4:         ['/Mixer/Feature_Volume_3', 2, self.sldInput3, 1, self.linkButton34, self.decibel4, 0],
            self.sldInput5:         ['/Mixer/Feature_Volume_4', 1, self.sldInput6, 2, self.linkButton56, self.decibel5, 0],
            self.sldInput6:         ['/Mixer/Feature_Volume_4', 2, self.sldInput5, 1, self.linkButton56, self.decibel6, 0],
            self.sldInput7:         ['/Mixer/Feature_Volume_5', 1, self.sldInput8, 2, self.linkButton78, self.decibel7, 0],
            self.sldInput8:         ['/Mixer/Feature_Volume_5', 2, self.sldInput7, 1, self.linkButton78, self.decibel8, 0],
            self.sldInputSPDIFL:    ['/Mixer/Feature_Volume_6', 1, self.sldInputSPDIFR, 2, self.linkButtonSPDIF, self.decibelSPDIFL, 0],
            self.sldInputSPDIFR:    ['/Mixer/Feature_Volume_6', 2, self.sldInputSPDIFL, 1, self.linkButtonSPDIF, self.decibelSPDIFR, 0],
            self.sldInputWavePlayL: ['/Mixer/Feature_Volume_7', 1, self.sldInputWavePlayR, 2, self.linkButtonWavePlay, self.decibelWavePlayL, 0],
            self.sldInputWavePlayR: ['/Mixer/Feature_Volume_7', 2, self.sldInputWavePlayL, 1, self.linkButtonWavePlay, self.decibelWavePlayR, 0]
            }

        self.SelectorControls={
            self.comboOutAssign:    '/Mixer/Selector_6', 
            self.comboInAssign:     '/Mixer/Selector_7', 
            self.comboExtSync:      '/Mixer/Selector_8', 
            self.comboSyncSource:   '/Mixer/Selector_9', 
            self.comboFrontBack:    '/Mixer/Selector_10'
        }

        self.MuteControls={
            self.muteButtonMasterL:   [self.sldInputMasterL, self.muteButtonMasterR],
            self.muteButtonMasterR:   [self.sldInputMasterR, self.muteButtonMasterL],
            self.muteButton1:         [self.sldInput1, self.muteButton2],
            self.muteButton2:         [self.sldInput2, self.muteButton1],
            self.muteButton3:         [self.sldInput3, self.muteButton4],
            self.muteButton4:         [self.sldInput4, self.muteButton3],
            self.muteButton5:         [self.sldInput5, self.muteButton6],
            self.muteButton6:         [self.sldInput6, self.muteButton5],
            self.muteButton7:         [self.sldInput7, self.muteButton8],
            self.muteButton8:         [self.sldInput8, self.muteButton7],
            self.muteButtonSPDIFL:    [self.sldInputSPDIFL, self.muteButtonSPDIFR],
            self.muteButtonSPDIFR:    [self.sldInputSPDIFR, self.muteButtonSPDIFL],
            self.muteButtonWavePlayL: [self.sldInputWavePlayL, self.muteButtonWavePlayR],
            self.muteButtonWavePlayR: [self.sldInputWavePlayR, self.muteButtonWavePlayL]
        }
        
        # gain control
        for ctl, params in self.VolumeControls.items():
                path    = params[0]
                idx     = params[1]
                dbmeter = params[5]
                
                #db = self.hw.getContignuous(path, idx)
                #vol = self.db2vol(db)
                vol = self.hw.getContignuous(path, idx)
                print("%s ch %d volume is %d" % (path, idx, vol))
                ctl.setValue(vol)
                dbmeter.setText(self.vol2dbstr(vol))
                self.VolumeControls[ctl][6] = vol
                
                pair    = params[2]
                pidx    = params[3]
                link    = params[4]
                
#               pdb = self.hw.getContignuous(path, pidx)
#               pvol = self.db2vol(db)
                pvol = self.hw.getContignuous(path, pidx)

                if pvol == vol:
                        link.setChecked(True)
                
                ctl.valueChanged.connect(self.updateVolume)

        # selector controls
        for ctl, param in self.SelectorControls.items():
                state = self.hw.getDiscrete(param)
                ctl.setCurrentIndex(state)
                
                ctl.activated.connect(self.updateSelector)

        # mute controls
        for ctl, param in self.MuteControls.items():
                ctl.toggled.connect(self.muteVolume)


    # helper functions 
    def muteVolume(self, state):
        sender    = self.sender()
        volctl    = self.MuteControls[sender][0]
        path      = self.VolumeControls[volctl][0]
        idx       = self.VolumeControls[volctl][1]
        pair      = self.VolumeControls[volctl][2]
        pidx      = self.VolumeControls[volctl][3]
        link      = self.VolumeControls[volctl][4]
        savedvol  = self.VolumeControls[volctl][6]
        psavedvol = self.VolumeControls[pair][6]

        if state == True :
            self.hw.setContignuous(path, -25600, idx) # The PHASE88 supports volume between 0 and -25600
            if link.isChecked():
                pair.setDisabled(state)
                self.MuteControls[sender][1].setChecked(state)
#                self.hw.setContignuous(path, 0, pidx)
        else:
            self.hw.setContignuous(path, savedvol, idx)
            if link.isChecked():
                pair.setDisabled(state)
                self.MuteControls[sender][1].setChecked(state)
#                self.hw.setContignuous(path, psavedvol, pidx)

    def updateVolume(self, vol):
        sender  = self.sender()
        path    = self.VolumeControls[sender][0]
        idx     = self.VolumeControls[sender][1]
        pair    = self.VolumeControls[sender][2]
        pidx    = self.VolumeControls[sender][3]
        link    = self.VolumeControls[sender][4]
        dbmeter = self.VolumeControls[sender][5]

        #db = self.vol2dbstr(vol)
        #self.hw.setContignuous(path, db, idx)
        self.hw.setContignuous(path, vol, idx)
        dbmeter.setText(self.vol2dbstr(vol))
        self.VolumeControls[sender][6] = vol

        if link.isChecked():
            pair.setValue(vol)

    def updateSelector(self, state):
        sender  = self.sender()
        path    = self.SelectorControls[sender]
        self.hw.setDiscrete(path, state)
#       if path == '/Mixer/Selector_7'
#            ctrl  = self.VolumeControls['line78']
#            vol = self.hw.getContignuous(ctrl[0])          # Recall volume for selected channel *******************************************************
#            name  = 'line78'
#            log.debug("%s volume is %d" % (name , vol))
#            ctrl[1].setValue(-vol)

    def vol2dbstr(self, vol):
        vol = vol + 25600
        if vol == 0 :
            return "- "+u"\u221E"+" dB"
        return str("{0:.2f}".format(log10( float(abs(vol) + 0.001) / 25600 ) * 20))+"dB"

# vim: et
