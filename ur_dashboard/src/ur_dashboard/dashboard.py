# Much of this code is borrowed from kobuki_dashboard

from functools import partial

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Empty, Int32MultiArray

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget
from rqt_robot_dashboard.widgets import MenuDashWidget, IconToolButton
from QtGui import QMessageBox, QAction, QTextEdit
from python_qt_binding.QtCore import QSize, Signal
from QtCore import pyqtSlot, pyqtSignal

class URDashboard(Dashboard):
    DIAG_TIMEOUT_TIME = 3.0

    def setup(self, context):
        self._last_diag_msg_time = 0.0

        self._iface_widget = InterfaceWidget('/config_fwd_ctrl/open_interface',
                                             '/config_fwd_ctrl/close_interface')
        self._pwr_widget = PowerWidget('/config_fwd_ctrl/power_on_robot',
                                       '/config_fwd_ctrl/power_off_robot')
        self._estop_widget = EStopWidget('/config_fwd_ctrl/power_on_robot')
        self._sstop_widget = SecuStopWidget('/config_fwd_ctrl/unlock_security_stop',
                                            '/config_fwd_ctrl/set_security_stop')
        self._mode_text = RobotModeTextWidget()

        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', DiagnosticArray, 
                                                   self.diagnostics_cb)
        self.timeout_timer = rospy.Timer(rospy.Duration(0.3), self.check_timeout_cb)

    def get_widgets(self):
        return [[MonitorDashWidget(self.context), 
                 ConsoleDashWidget(self.context)], 
                [self._iface_widget, self._pwr_widget], 
                [self._estop_widget, self._sstop_widget],
                [self._mode_text]]

    def diagnostics_cb(self, msg):
        self._last_diag_msg_time = rospy.get_time()
        
        for status in msg.status:
            if "UR arm controller" in status.name:
                robot_mode = status.values[0].value
                self._mode_text.update_state(robot_mode)

                if robot_mode == 'NOT_CONNECTED':
                    self._iface_widget.update_state(0)
                elif robot_mode != 'UNINITIALIZED':
                    self._iface_widget.update_state(1)

                if int(status.values[2].value):
                    self._pwr_widget.update_state(1)
                else:
                    self._pwr_widget.update_state(0)

                if robot_mode == 'RUNNING':
                    pass
                if robot_mode == 'READY':
                    pass

                if robot_mode == 'EMERGENCY_STOPPED':
                    self._estop_widget.update_state(0)
                elif robot_mode == 'INITIALIZING':
                    self._estop_widget.update_state(2)
                else:
                    self._estop_widget.update_state(1)

                if robot_mode == 'SECURITY_STOPPED':
                    self._sstop_widget.update_state(0)
                else:
                    self._sstop_widget.update_state(1)

    def check_timeout_cb(self, te):
        if rospy.get_time() - self._last_diag_msg_time > self.DIAG_TIMEOUT_TIME:
            # set every button to stale
            self._iface_widget.update_state(2)
            self._pwr_widget.update_state(2)
            self._estop_widget.update_state(3)
            self._sstop_widget.update_state(2)
            self._mode_text.update_state('STALE')

    def shutdown_dashboard(self):
        self._dashboard_agg_sub.unregister()
        self.timeout_timer.shutdown()

class InterfaceWidget(IconToolButton):
    def __init__(self, topic_open, topic_close):
        self._pub_open = rospy.Publisher(topic_open, Empty)
        self._pub_close = rospy.Publisher(topic_close, Empty)

        self._off_icon = ['bg-red.svg', 'ic-navigation.svg']
        self._on_icon = ['bg-green.svg', 'ic-navigation.svg']
        self._stale_icon = ['bg-grey.svg', 'ic-navigation.svg', 'ol-stale-badge.svg']

        icons = [self._off_icon, self._on_icon, self._stale_icon]
        super(InterfaceWidget, self).__init__("interface_button", icons=icons)
        self.setFixedSize(QSize(40,40))

        super(InterfaceWidget, self).update_state(2)
        self.setToolTip("Interface: Stale")

        self.clicked.connect(self.toggle)


    def update_state(self, state):
        if state is not super(InterfaceWidget, self).state:
            super(InterfaceWidget, self).update_state(state)
            if state is 1:
                self.setToolTip("Interface: Open")
            elif state is 0:
                self.setToolTip("Interface: Closed")
            else:
                self.setToolTip("Interface: Stale")


    def toggle(self):
        if super(InterfaceWidget, self).state is 1:
            self._pub_close.publish(Empty())
        else:
            self._pub_open.publish(Empty())

    def close(self):
        self._pub_open.unregister()
        self._pub_close.unregister()

class PowerWidget(IconToolButton):
    def __init__(self, topic_on, topic_off):
        self._pub_on = rospy.Publisher(topic_on, Empty)
        self._pub_off = rospy.Publisher(topic_off, Empty)

        self._off_icon = ['bg-red.svg', 'ic-breaker.svg']
        self._on_icon = ['bg-green.svg', 'ic-breaker.svg']
        self._stale_icon = ['bg-grey.svg', 'ic-breaker.svg', 'ol-stale-badge.svg']

        icons = [self._off_icon, self._on_icon, self._stale_icon]
        super(PowerWidget, self).__init__("power_button", icons=icons)
        self.setFixedSize(QSize(40,40))

        super(PowerWidget, self).update_state(2)
        self.setToolTip("Power: Stale")

        self.clicked.connect(self.toggle)


    def update_state(self, state):
        if state is not super(PowerWidget, self).state:
            super(PowerWidget, self).update_state(state)
            if state is 1:
                self.setToolTip("Power: On")
            elif state is 0:
                self.setToolTip("Power: Off")
            else:
                self.setToolTip("Power: Stale")

    def toggle(self):
        if super(PowerWidget, self).state is 1:
            self._pub_off.publish(Empty())
        else:
            self._pub_on.publish(Empty())

    def close(self):
        self._pub_on.unregister()
        self._pub_off.unregister()

class EStopWidget(IconToolButton):
    def __init__(self, topic_enable):
        self._pub_enable = rospy.Publisher(topic_enable, Empty)

        self._stopped_icon = ['bg-red.svg', 'ic-wireless-runstop-on.svg']
        self._running_icon = ['bg-green.svg', 'ic-wireless-runstop-off.svg']
        self._resetting_icon = ['bg-yellow.svg', 'ic-wireless-runstop-on.svg', 'ol-warn-badge.svg']
        self._stale_icon = ['bg-grey.svg', 'ic-wireless-runstop-on.svg', 'ol-stale-badge.svg']

        icons = [self._stopped_icon, self._running_icon, self._resetting_icon, self._stale_icon]
        super(EStopWidget, self).__init__("estop_button", icons=icons)
        self.setFixedSize(QSize(40,40))

        super(EStopWidget, self).update_state(3)
        self.setToolTip("E-Stop: Stale")

        self.clicked.connect(self.toggle)


    def update_state(self, state):
        if state is not super(EStopWidget, self).state:
            super(EStopWidget, self).update_state(state)
            if state is 0:
                self.setToolTip("E-Stop: Stopped")
            elif state is 1:
                self.setToolTip("E-Stop: Running")
            elif state is 2:
                self.setToolTip("E-Stop: Needs Reset")
            else:
                self.setToolTip("E-Stop: Stale")

    def toggle(self):
        if super(EStopWidget, self).state is 2:
            self._pub_enable.publish(Empty())

    def close(self):
        self._pub_enable.unregister()

class SecuStopWidget(IconToolButton):
    def __init__(self, topic_enable, topic_stop):
        self._pub_enable = rospy.Publisher(topic_enable, Empty)
        self._pub_stop = rospy.Publisher(topic_stop, Int32MultiArray)

        self._stopped_icon = ['bg-red.svg', 'ic-runstop-on.svg', 'ol-err-badge.svg']
        self._running_icon = ['bg-green.svg', 'ic-runstop-off.svg']
        self._stale_icon = ['bg-grey.svg', 'ic-runstop-on.svg', 'ol-stale-badge.svg']

        icons = [self._stopped_icon, self._running_icon, self._stale_icon]
        super(SecuStopWidget, self).__init__("secustop_button", icons=icons)
        self.setFixedSize(QSize(40,40))

        super(SecuStopWidget, self).update_state(2)
        self.setToolTip("Secu-Stop: Stale")

        self.clicked.connect(self.toggle)

    def update_state(self, state):
        if state is not super(SecuStopWidget, self).state:
            super(SecuStopWidget, self).update_state(state)
            if state is 0:
                self.setToolTip("Secu-Stop: Stopped")
            elif state is 1:
                self.setToolTip("Secu-Stop: Running")
            elif state is 2:
                self.setToolTip("Secu-Stop: Stale")

    def toggle(self):
        if super(SecuStopWidget, self).state is 0:
            self._pub_enable.publish(Empty())
        elif super(SecuStopWidget, self).state is 1:
            stop_msg = Int32MultiArray()
            stop_msg.data = [0, 110, 0]
            self._pub_stop.publish(stop_msg)

    def close(self):
        self._pub_enable.unregister()
        self._pub_stop.unregister()

class RobotModeTextWidget(QTextEdit):
    text_changed = Signal()
    def __init__(self):
        super(QTextEdit, self).__init__()
        self.setObjectName("robot_mode_text")
        self.setReadOnly(True)
        self.setText('STALE')
        self.setFixedSize(QSize(180,27))
        self.text_changed.connect(self._update_state)
        self.__state = 'STALE'

    def update_state(self, state):
        if type(state) is str:
            self.__state = state
            self.text_changed.emit()
        else:
            raise TypeError('state must be a string')

    def _update_state(self):
        self.setText(self.__state)

    def state(self):
        return self.__state
