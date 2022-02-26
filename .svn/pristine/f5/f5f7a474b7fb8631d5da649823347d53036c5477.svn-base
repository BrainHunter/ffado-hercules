#
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

import logging
log = logging.getLogger('saffirebase')

# the class that holds all actual control code
class SaffireMixerBase:
    def __init__(self):
        pass

    def updateMatrixVolume(self,a0):
        sender = self.sender()
        vol = a0
        log.debug("set %s %d %d to %d" % (
                    self.VolumeControls[sender][0],
                    self.VolumeControls[sender][1],
                    self.VolumeControls[sender][2],
                    vol))
        self.hw.setMatrixMixerValue(self.VolumeControls[sender][0], 
                                    self.VolumeControls[sender][1],
                                    self.VolumeControls[sender][2],
                                    vol)
    def updateLowResVolume(self,a0):
        sender = self.sender()
        vol = 127 - a0
        log.debug("set %s to %d" % (
                    self.VolumeControlsLowRes[sender][0],
                    vol))
        self.hw.setDiscrete(self.VolumeControlsLowRes[sender][0], vol)

    def updateSelector(self,a0):
        sender = self.sender()
        if a0:
            state = 1
        else:
            state = 0
        log.debug("set %s to %d" % (
                    self.SelectorControls[sender][0],
                    state))
        self.hw.setDiscrete(self.SelectorControls[sender][0], state)

        # if there are linked selector controls, update them
        if len(self.SelectorControls[sender]) >= 2:
            linked = self.SelectorControls[sender][1]
            for ctl in linked:
                if ctl.isChecked():
                    state = 1
                else:
                    state = 0
                self.hw.setDiscrete(self.SelectorControls[ctl][0], state)

    def triggerButton(self):
        sender = self.sender()
        log.debug("trigger %s" % (
                    self.TriggerButtonControls[sender][0]))
        self.hw.setDiscrete(self.TriggerButtonControls[sender][0], 1)

    def saveText(self):
        sender = self.sender()
        textbox = self.saveTextControls[sender][0]
        log.debug("save %s" % (
                    textbox.text().ascii()))
        self.hw.setText(self.TextControls[textbox][0], textbox.text().ascii())

    def initCombo(self, combo):
        path = self.ComboControls[combo][0]
        combo.clear()
        nb_items = self.hw.enumCount(path)
        for i in range( nb_items ):
            combo.insertItem( nb_items, self.hw.enumGetLabel(path, i) )
        combo.setCurrentIndex( self.hw.enumSelected(path) )

    def selectCombo(self, mode):
        sender = self.sender()
        path = self.ComboControls[sender][0]
        self.hw.enumSelect(path, mode)
        sender.setCurrentIndex( self.hw.enumSelected(path) )

    def updateValues(self):
        for ctrl, info in self.VolumeControls.items():
            vol = self.hw.getMatrixMixerValue(self.VolumeControls[ctrl][0],
                                              self.VolumeControls[ctrl][1],
                                              self.VolumeControls[ctrl][2])
            log.debug("%s volume is %d" % (ctrl.objectName() , vol))
            ctrl.setValue(vol)
        for ctrl, info in self.VolumeControlsLowRes.items():
            vol = self.hw.getDiscrete(self.VolumeControlsLowRes[ctrl][0])

            log.debug("%s volume is %d" % (ctrl.objectName() , 127-vol))
            ctrl.setValue(127 - vol)

        for ctrl, info in self.SelectorControls.items():
            state = self.hw.getDiscrete(self.SelectorControls[ctrl][0])
            log.debug("%s state is %d" % (ctrl.objectName() , state))
            if state:
                ctrl.setChecked(True)
            else:
                ctrl.setChecked(False)

        for ctrl, info in self.TriggerButtonControls.items():
            pass

        for ctrl, info in self.TextControls.items():
            text = self.hw.getText(self.TextControls[ctrl][0])
            log.debug("%s text is %s" % (ctrl.objectName() , text))
            ctrl.setText(text)

        for ctrl, info in self.ComboControls.items():
            self.initCombo(ctrl)

    def polledUpdateVolumeLowRes(self, srcpath, ctrl, divider=1):
        vol = self.hw.getDiscrete(srcpath) / divider
        #log.debug("polledUpdateVolumeLowRes: vol = %s" % vol)
        ctrl.setValue(vol)

    def initValues(self):
        self.updateValues()
        for ctrl, info in self.VolumeControls.items():
            ctrl.valueChanged.connect(self.updateMatrixVolume)

        for ctrl, info in self.VolumeControlsLowRes.items():
            ctrl.valueChanged.connect(self.updateLowResVolume)

        for ctrl, info in self.SelectorControls.items():
            ctrl.stateChanged.connect(self.updateSelector)

        for ctrl, info in self.TriggerButtonControls.items():
            ctrl.clicked.connect(self.triggerButton)

        for ctrl, info in self.saveTextControls.items():
            ctrl.clicked.connect(self.saveText)

        for ctrl, info in self.ComboControls.items():
            ctrl.activated.connect(self.selectCombo)

# vim: et
