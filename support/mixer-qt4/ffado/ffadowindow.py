# Copyright (C) 2005-2008 by Pieter Palmers
#               2007-2009 by Arnold Krille
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

import ctypes
import datetime
import os

from ffado.config import *

import subprocess

# from PyQt4.QtCore import QObject, QTimer, Qt
# from PyQt4.QtGui import *
from ffado.import_pyqt import *

from ffado.dbus_util import *

from ffado.panelmanager import PanelManager

from ffado.logginghandler import *

"""Just a small helper to ask the retry-question without a blocking messagebox"""
class StartDialog(QWidget):
    def __init__(self, parent):
        QWidget.__init__(self, parent)
        self.setObjectName("Restart Dialog")
        self.label = QLabel("<qt>Somehow the connection to the dbus-service of FFADO couldn't be established.<p>\nShall we take another try?</qt>",self)
        self.button = QPushButton("Retry", self)
        self.layout = QGridLayout(self)
        self.layout.setContentsMargins( 50, 10, 50, 10 )
        self.layout.addWidget(self.label, 0, 0, Qt.AlignHCenter|Qt.AlignBottom)
        self.layout.addWidget(self.button, 1, 0, Qt.AlignHCenter|Qt.AlignTop)

class FFADOWindow(QMainWindow):
    def __init__(self, parent):
        QMainWindow.__init__(self, parent)

        self.textlogger = QTextLogger(self)
        dock = QDockWidget("Log Messages",self)
        dock.setWidget(self.textlogger.textedit)
        logging.getLogger('').addHandler(self.textlogger)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)

        self.statuslogger = QStatusLogger(self, self.statusBar(), 20)
        logging.getLogger('').addHandler(self.statuslogger)

        self.settings = QtCore.QSettings(self)

        self.manager = PanelManager(self)
        self.manager.connectionLost.connect(self.connectToDBUS)

        filemenu = self.menuBar().addMenu("&File")
        self.openaction = QAction(QIcon.fromTheme("document-open"),"&Open", self)
        self.openaction.setShortcut(self.tr("Ctrl+O"))
        self.openaction.setEnabled(False)
        self.openaction.triggered.connect(self.manager.readSettings)
        filemenu.addAction(self.openaction)
        self.saveaction = QAction(QIcon.fromTheme("document-save-as"),"&Save as...", self)
        self.saveaction.setShortcut(self.tr("Ctrl+S"))
        self.saveaction.setEnabled(False)
        self.saveaction.triggered.connect(self.manager.saveSettings)
        filemenu.addAction(self.saveaction)
        self.quitaction = QAction(QIcon.fromTheme("application-exit"),"&Quit", self)
        self.quitaction.setShortcut(self.tr("Ctrl+q"))
        self.quitaction.triggered.connect(self.close)
        filemenu.addAction(self.quitaction)

        self.editmenu = self.menuBar().addMenu("&View")

        self.thememenu = self.editmenu.addMenu("Theme")
        themes = QStyleFactory.keys()
        self.menuTheme = {}
        for theme in themes:
            self.menuTheme[theme] = QAction(QIcon.fromTheme("preferences-desktop-theme"), theme, self )
            self.menuTheme[theme].setCheckable(True)

            if (ffado_python3 and (self.style().objectName().lower() == theme.lower()) or
                    not(ffado_python3) and (self.style().objectName().toLower() == theme.toLower() if ffado_pyqt_version == 4 else
                                            self.style().objectName().lower() == theme.lower())):
                self.menuTheme[theme].setDisabled(True)
                self.menuTheme[theme].setChecked(True)
            self.menuTheme[theme].triggered.connect(self.switchTheme )
            self.thememenu.addAction( self.menuTheme[theme] )

        conftheme = self.settings.value("window/theme", "ukui-dark")
        contains = False
        for theme in self.menuTheme:
            if theme.__str__() == conftheme:
                contains = True
                break
        if contains:
            for theme in self.menuTheme:
                if theme.__str__() != conftheme:
                    self.menuTheme[theme].setChecked(False)
                    self.menuTheme[theme].setDisabled(False)
            for theme in self.menuTheme:
                if theme.__str__() == conftheme:
                    self.menuTheme[theme].setDisabled(True)
                    QApplication.setStyle(QStyleFactory.create(theme))
                    self.settings.setValue("window/theme", theme.__str__())

        self.updateaction = QAction(QIcon.fromTheme("view-refresh"),"&Update Mixer Panels", self)
        self.updateaction.setEnabled(False)
        self.updateaction.triggered.connect(self.manager.updatePanels)
        self.editmenu.addAction(self.updateaction)
        self.refreshaction = QAction(QIcon.fromTheme("view-refresh"),"&Refresh Current Panels", self)
        self.refreshaction.triggered.connect(self.manager.refreshPanels)
        self.editmenu.addAction(self.refreshaction)
        self.editmenu.addSeparator()
        self.devices = {}

        helpmenu = self.menuBar().addMenu( "&Help" )
        self.aboutaction = QAction(QIcon.fromTheme("help-about"), "About &FFADO", self )
        self.aboutaction.triggered.connect(self.aboutFFADO)
        helpmenu.addAction( self.aboutaction )
        self.aboutqtaction = QAction(QIcon.fromTheme("help-about"),  "About &Qt", self )
        self.aboutqtaction.triggered.connect(QApplication.instance().aboutQt)
        helpmenu.addAction( self.aboutqtaction )

        log.info( "Starting up" )

        QTimer.singleShot( 1, self.tryStartDBUSServer )

    def __del__(self):
        log.info("__del__")
        del self.manager
        log.info("__del__ finished")

    def switchTheme(self, checked) :
        for theme in self.menuTheme :
            if not self.menuTheme[theme].isEnabled() :
                self.menuTheme[theme].setChecked(False)
                self.menuTheme[theme].setDisabled(False)
        for theme in self.menuTheme :
            if self.menuTheme[theme].isChecked() :
                self.menuTheme[theme].setDisabled(True)
                QApplication.setStyle(QStyleFactory.create(theme))
                self.settings.setValue("window/theme", theme.__str__())

    def closeEvent(self, event):
        log.info("closeEvent()")
        event.accept()

    def connectToDBUS(self):
        log.info("connectToDBUS")
        try:
            self.setupDeviceManager()
        except dbus.DBusException as ex:
            log.error("Could not communicate with the FFADO DBus service...")
            if not hasattr(self,"retry"):
                self.retry = StartDialog(self)
                self.retry.button.clicked.connect(self.tryStartDBUSServer)
            if hasattr(self, "retry"):
                self.manager.setParent(None)
                self.setCentralWidget(self.retry)
                self.retry.setEnabled(True)

    def tryStartDBUSServer(self):
        try:
            self.setupDeviceManager()
        except dbus.DBusException as ex:
            if hasattr(self, "retry"):
                self.retry.setEnabled(False)
            subprocess.Popen(['ffado-dbus-server', '-v3'], close_fds=True).pid
            QTimer.singleShot(5000, self.connectToDBUS)

    def setupDeviceManager(self):
        devmgr = DeviceManagerInterface(FFADO_DBUS_SERVER, FFADO_DBUS_BASEPATH)
        self.manager.setManager(devmgr)
        if hasattr(self, "retry"):
            self.retry.setParent(None)
        self.setCentralWidget(self.manager)
        self.updateaction.setEnabled(True)

    def aboutFFADO(self):
        QMessageBox.about( self, "About FFADO", """
<h1><a href="http://ffado.org">ffado.org</a></h1>

<p>{ffado_version}</p>

<p><a href="http://ffado.org">FFADO</a> is the new approach to have FireWire audio on Linux.</p>

        <p>&copy; 2006-2021 by the FFADO developers<br />ffado is licensed under the GPLv3, for the full license text see <a href="http://www.gnu.org/licenses/">www.gnu.org/licenses</a> or the LICENSE.* files shipped with ffado.</p>

<p>FFADO developers are:<ul>
<li>Pieter Palmers
<li>Daniel Wagner
<li>Jonathan Woithe
<li>Arnold Krille
<li>Philippe Carriere
<li>Takashi Sakamoto
</ul>
with contributions from:<ul>
<li>Adrian Knoth
<li>Stefan Richter
<li>Jano Svitok
<li>Pander Musubi
</ul>
        """.format(ffado_version=get_ffado_version(), thisyear=datetime.datetime.now().year))

def get_ffado_version():
    try:
        # call the C function ffado_get_version() to figure out the version
        lib = ctypes.cdll.LoadLibrary('libffado.so')
        func = ctypes.CFUNCTYPE(ctypes.c_char_p)
        ffado_get_version = func(('ffado_get_version', lib))
        return ffado_get_version()
    except:
        return "libffado"

def get_lock(process_name):
    import socket
    import sys

    global lock_socket
    lock_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        lock_socket.bind('\0' + process_name)
        # Lock acquired
    except socket.error:
        print( 'ffado-mixer instance is already running' )
        sys.exit()


def ffadomain(args):
    #set up logging
    import logging
    logging.basicConfig( datefmt="%H:%M:%S", format="%(asctime)s %(name)-16s %(levelname)-8s %(message)s" )

    if DEBUG:
        debug_level = logging.DEBUG
    else:
        debug_level = logging.INFO

    get_lock('ffado-mixer')

    # Very simple command line option parser
    if (len(args) > 1) and (args[1] == "-b" or args[1] == "--bypassdbus"):
        ffado.config.bypassdbus = True

    # main loggers:
    logging.getLogger('main').setLevel(debug_level)
    logging.getLogger('dbus').setLevel(debug_level)
    logging.getLogger('registration').setLevel(debug_level)
    logging.getLogger('panelmanager').setLevel(debug_level)
    logging.getLogger('configparser').setLevel(logging.INFO)

    # widgets:
    logging.getLogger('matrixmixer').setLevel(debug_level)
    logging.getLogger('crossbarrouter').setLevel(debug_level)

    # mixers:
    logging.getLogger('audiofire').setLevel(debug_level)
    logging.getLogger('bridgeco').setLevel(debug_level)
    logging.getLogger('edirolfa101').setLevel(debug_level)
    logging.getLogger('edirolfa66').setLevel(debug_level)
    logging.getLogger('motu').setLevel(debug_level)
    logging.getLogger('rme').setLevel(debug_level)
    logging.getLogger('phase24').setLevel(debug_level)
    logging.getLogger('phase88').setLevel(debug_level)
    logging.getLogger('quatafire').setLevel(debug_level)
    logging.getLogger('saffirebase').setLevel(debug_level)
    logging.getLogger('saffire').setLevel(debug_level)
    logging.getLogger('saffirepro').setLevel(debug_level)

    logging.getLogger('global').setLevel(debug_level)

    log = logging.getLogger('main')
    log.debug("Using %s with Qt: %s PyQt: %s" % (get_ffado_version(), QtCore.QT_VERSION_STR, QtCore.PYQT_VERSION_STR))

    app = QApplication(args)
    app.setWindowIcon( QIcon( SHAREDIR + "/icons/hi64-apps-ffado.png" ) )

    app.setOrganizationName("FFADO")
    app.setOrganizationDomain("ffado.org")
    app.setApplicationName("ffado-mixer")

    mainwindow = FFADOWindow(None)


    # rock & roll
    mainwindow.show()
    return app.exec_()

if __name__ == "__main__":
    import sys
    sys.exit(ffadomain(sys.argv))

#
# vim: ts=4 sw=4 et
