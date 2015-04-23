#!/usr/bin/env python

import rospy

from python_qt_binding.QtCore import Slot, QCoreApplication, QBasicTimer, QSettings
from python_qt_binding.QtGui import QWidget

from std_msgs.msg import Int8

class OCSPlugin(QWidget):

    def __init__(self, widget_class, name, window, *args):
        super(OCSPlugin, self).__init__(None)

        if len(args) > 0:
            self._widget = widget_class(self, args[0])
        else:
            self._widget = widget_class(self)

        self._name = name
        self._window = window

        self._window_control_sub = rospy.Subscriber("/flor/ocs/window_control", Int8, self.processWindowControl)
        self._window_control_pub = rospy.Publisher("/flor/ocs/window_control", Int8, queue_size=10)
        
        # window visibility configuration variables
        self._last_window_control_data = -self._window
        self._set_window_visibility = True

        # this is only used to make sure we close window if ros::shutdown has already been called
        self._timer = QBasicTimer()
        self._timer.start(33, self)
        
        settings = QSettings("OCS", self._name)
        if settings.value("mainWindowGeometry") != None:
            self.restoreGeometry(settings.value("mainWindowGeometry"))
    	self.geometry_ = self.geometry()

    def processWindowControl(self, visible):
        # set window visibility changed flag
        self._set_window_visibility = True
        self._last_window_control_data = visible.data

    def timerEvent(self, event):
        if rospy.is_shutdown():
            QCoreApplication.instance().quit();
            
        # needed to move qt calls out of the ros callback, otherwise qt crashes because of inter-thread communication
        if self._set_window_visibility:
            self._set_window_visibility = False
            if not self.isVisible() and self._last_window_control_data == self._window:
                self.show()
                self.setGeometry(self._geometry)
            elif self.isVisible() and (not self._last_window_control_data or self._last_window_control_data == -self._window):
                self._geometry = self.geometry()
                self.hide()
                
    def closeEvent(self, event):
        settings = QSettings("OCS", self._name)
        settings.setValue("mainWindowGeometry", self.saveGeometry())
        msg = Int8()
        msg.data = -self._window
        self._window_control_pub.publish(msg)
        event.ignore()

    def resizeEvent(self, event):
        settings = QSettings("OCS", self._name)
        settings.setValue("mainWindowGeometry", self.saveGeometry())

    def moveEvent(self, event):
        settings = QSettings("OCS", self._name)
        settings.setValue("mainWindowGeometry", self.saveGeometry())

