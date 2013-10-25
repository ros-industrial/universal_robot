import os
import numpy as np
from functools import partial

import QtCore
import QtGui
from qt_gui.plugin import Plugin

import rospy
from rqt_robot_dashboard.util import IconHelper

from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState

from .joint_vel_ctrl import JointVelocityController

JOINT_NAMES = ['Shoulder Pan', 'Shoulder Lift', 'Elbow',
               'Wrist 1', 'Wrist 2', 'Wrist 3']
JOINT_SHORT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
ICON_NAMES = [['rotate_y_pos_on.png'], ['rotate_y_neg_on.png'],
              ['rotate_z_neg_on.png'], ['rotate_z_pos_on.png']]
BUTTON_ICONS = [(2,3), (0,1), (0,1), (0,1), (2,3), (0,1)]
VEL_MULTS = [-1.0, 1.0] 
MONITOR_RATE = 1000./250.

class URJointCtrlGui(Plugin):

    # signals
    state_changed = QtCore.Signal()
    start_moving = QtCore.Signal(int, float, float)
    stop_moving = QtCore.Signal()

    def __init__(self, context):
        super(URJointCtrlGui,self).__init__(context)
        self.context = context
        self.name = 'URJointCtrlGui'

        # outer frame widget
        self._frame_widget = QtGui.QWidget()
        frame = self._frame_widget
        frame.setObjectName(self.name)
        frame.setWindowTitle('UR Joint Control GUI')
        context.add_widget(frame)

        # interactive components
        self.buttons = []
        self.speed_boxes = []
        self.joint_modes = []
        self.q_rad_nums = []
        self.q_deg_nums = []
        self.q_sliders = []

        # internal state
        self._joint_modes = ['STALE']*6
        self._q_pos = [0.]*6
        self._q_order = None
        self._cur_speed = 0.0
        self._cur_vel_mult = 0.0
        self._cur_jnt_moving = None

        # setup IconHelper
        import rospkg
        rp = rospkg.RosPack()
        paths = [os.path.join(rp.get_path('ur_joint_ctrl_gui'), 'images')]
        self.icon_helper = IconHelper(paths, self.name)
        converted_icons = self.icon_helper.set_icon_lists(ICON_NAMES, None, False)
        self._icons = converted_icons[0]
        self._clicked_icons = converted_icons[1]

        # setup gui
        vlayout_outer = QtGui.QVBoxLayout(frame)
        vlayout_outer.setObjectName('vert_layout_outer')
        for jnt_idx in range(6):
            hlayout = QtGui.QHBoxLayout()
            hlayout.setObjectName('hlayout_%d' % jnt_idx)

            ############################################################################

            vlayout1 = QtGui.QVBoxLayout()
            vlayout1.setObjectName('vlayout1_%d' % jnt_idx)

            joint_label = QtGui.QLabel(JOINT_NAMES[jnt_idx])
            joint_label.setObjectName('joint_label_%d' % jnt_idx)
            joint_label.setAlignment(QtCore.Qt.AlignCenter)
            vlayout1.addWidget(joint_label)

            hlayout11 = QtGui.QHBoxLayout()
            hlayout11.setObjectName('hlayout11_%d' % jnt_idx)
            cur_buttons = []
            for bside_idx, bside in enumerate(['l', 'r']):
                button = QtGui.QPushButton(frame)
                button.setObjectName('button_%s_%d' % (bside, jnt_idx))
                size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, 
                                                QtGui.QSizePolicy.Expanding)
                size_policy.setHorizontalStretch(0)
                size_policy.setVerticalStretch(0)
                size_policy.setHeightForWidth(button.sizePolicy().hasHeightForWidth())
                button.setSizePolicy(size_policy)
                button.setBaseSize(QtCore.QSize(80, 80))
                button.setIcon(self._icons[BUTTON_ICONS[jnt_idx][bside_idx]])
                button.setIconSize(QtCore.QSize(40,40))
                hlayout11.addWidget(button)
                cur_buttons.append(button)

            vlayout1.addLayout(hlayout11)

            hlayout.addLayout(vlayout1)

            ############################################################################

            vlayout2 = QtGui.QVBoxLayout()
            vlayout2.setObjectName('vlayout2_%d' % jnt_idx)

            speed_label = QtGui.QLabel('Speed')
            speed_label.setObjectName('speed_label_%d' % jnt_idx)
            speed_label.setAlignment(QtCore.Qt.AlignCenter)
            vlayout2.addWidget(speed_label)

            speed_box = QtGui.QDoubleSpinBox(frame)
            speed_box.setObjectName('speed_box_%d' % jnt_idx)
            speed_box.setMaximum(1.0)
            speed_box.setMinimum(0.0)
            speed_box.setSingleStep(0.05)
            speed_box.setProperty('value', 0.1)
            vlayout2.addWidget(speed_box)

            hlayout.addLayout(vlayout2)

            ############################################################################

            vlayout3 = QtGui.QVBoxLayout()
            vlayout3.setObjectName('vlayout3_%d' % jnt_idx)

            state_label = QtGui.QLabel('State')
            state_label.setObjectName('state_label_%d' % jnt_idx)
            state_label.setAlignment(QtCore.Qt.AlignCenter)
            vlayout3.addWidget(state_label)

            joint_mode = QtGui.QTextBrowser(frame)
            joint_mode.setObjectName('q_state_%d' % jnt_idx)
            size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
            size_policy.setHorizontalStretch(0)
            size_policy.setVerticalStretch(0)
            size_policy.setHeightForWidth(joint_mode.sizePolicy().hasHeightForWidth())
            joint_mode.setSizePolicy(size_policy)
            joint_mode.setMinimumSize(QtCore.QSize(100, 31))
            joint_mode.setMaximumSize(QtCore.QSize(180, 31))
            joint_mode.setBaseSize(QtCore.QSize(100, 31))
            joint_mode.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            joint_mode.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            joint_mode.setLineWrapMode(QtGui.QTextEdit.NoWrap)
            joint_mode.setText('STALE')
            #joint_mode.setText('MOTOR_INITIALISATION')
            vlayout3.addWidget(joint_mode)

            hlayout.addLayout(vlayout3)

            ############################################################################

            vlayout4 = QtGui.QVBoxLayout()
            vlayout4.setObjectName('vlayout4_%d' % jnt_idx)

            hlayout41 = QtGui.QHBoxLayout()
            hlayout41.setObjectName('hlayout41_%d' % jnt_idx)

            q_rad_label = QtGui.QLabel('q (rad)')
            q_rad_label.setObjectName('q_rad_label_%d' % jnt_idx)
            q_rad_label.setAlignment(QtCore.Qt.AlignCenter)
            hlayout41.addWidget(q_rad_label)

            q_rad_num = QtGui.QTextBrowser(frame)
            q_rad_num.setObjectName('q_rad_num_%d' % jnt_idx)
            size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
            size_policy.setHorizontalStretch(0)
            size_policy.setVerticalStretch(0)
            size_policy.setHeightForWidth(q_rad_num.sizePolicy().hasHeightForWidth())
            q_rad_num.setSizePolicy(size_policy)
            q_rad_num.setMinimumSize(QtCore.QSize(60, 31))
            q_rad_num.setMaximumSize(QtCore.QSize(100, 31))
            q_rad_num.setBaseSize(QtCore.QSize(80, 31))
            q_rad_num.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            q_rad_num.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            q_rad_num.setLineWrapMode(QtGui.QTextEdit.NoWrap)
            #q_rad_num.setText('0.12345')
            hlayout41.addWidget(q_rad_num)

            q_deg_label = QtGui.QLabel('q (deg)')
            q_deg_label.setObjectName('q_deg_label_%d' % jnt_idx)
            q_deg_label.setAlignment(QtCore.Qt.AlignCenter)
            hlayout41.addWidget(q_deg_label)

            q_deg_num = QtGui.QTextBrowser(frame)
            q_deg_num.setObjectName('q_deg_num_%d' % jnt_idx)
            size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
            size_policy.setHorizontalStretch(0)
            size_policy.setVerticalStretch(0)
            size_policy.setHeightForWidth(q_deg_num.sizePolicy().hasHeightForWidth())
            q_deg_num.setSizePolicy(size_policy)
            q_deg_num.setMinimumSize(QtCore.QSize(60, 31))
            q_deg_num.setMaximumSize(QtCore.QSize(100, 31))
            q_deg_num.setBaseSize(QtCore.QSize(80, 31))
            q_deg_num.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            q_deg_num.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            q_deg_num.setLineWrapMode(QtGui.QTextEdit.NoWrap)
            #q_deg_num.setText('-180.5')
            hlayout41.addWidget(q_deg_num)

            vlayout4.addLayout(hlayout41)

            hlayout42 = QtGui.QHBoxLayout()
            hlayout42.setObjectName('hlayout42_%d' % jnt_idx)
            q_slider = QtGui.QSlider(frame)
            q_slider.setObjectName('q_slider_%d' % jnt_idx)
            q_slider.setTickPosition(QtGui.QSlider.TicksBelow)
            q_slider.setMinimum(-360)
            q_slider.setMaximum(360)
            q_slider.setTickInterval(90)
            q_slider.setValue(0)
            q_slider.setEnabled(False)
            q_slider.setOrientation(QtCore.Qt.Horizontal)
            hlayout42.addWidget(q_slider)

            vlayout4.addLayout(hlayout42)

            hlayout.addLayout(vlayout4)

            vlayout_outer.addLayout(hlayout)

            # collect the updatable components
            self.buttons.append(cur_buttons)
            self.speed_boxes.append(speed_box)
            self.joint_modes.append(joint_mode)
            self.q_rad_nums.append(q_rad_num)
            self.q_deg_nums.append(q_deg_num)
            self.q_sliders.append(q_slider)

        self.joint_vel_ctrl = JointVelocityController()

        # connect signals
        self.state_changed.connect(self._update_state)
        for i in range(6):
            for j in range(2):
                self.buttons[i][j].pressed.connect(partial(self._start_move, i, j))
                self.buttons[i][j].released.connect(self._stop_move)
        self.start_moving.connect(self.joint_vel_ctrl.start_moving)
        self.stop_moving.connect(self.joint_vel_ctrl.stop_moving)

        # setup update timer
        self.monitor_timer = QtCore.QTimer(context)
        QtCore.QObject.connect(self.monitor_timer, QtCore.SIGNAL("timeout()"), 
                               self.joint_vel_ctrl.update)
        self.monitor_timer.start(MONITOR_RATE)

        # create ros subscribers
        self._diag_agg_sub = rospy.Subscriber('diagnostics_agg', DiagnosticArray, 
                                              self._diagnostics_cb)
        self._joint_states_sub = rospy.Subscriber('joint_states', JointState, 
                                                  self._joint_states_cb)

    def _update_state(self):
        for i in range(6):
            self.joint_modes[i].setText(self._joint_modes[i])
            self.q_rad_nums[i].setText('%.4f' % self._q_pos[i])
            q_deg = np.rad2deg(self._q_pos[i])
            self.q_deg_nums[i].setText('%.2f' % q_deg)
            self.q_sliders[i].setValue(int(q_deg))

    def _start_move(self, jnt_idx, button_idx):
        self._cur_jnt_moving = jnt_idx
        self._cur_speed = self.speed_boxes[jnt_idx].value()
        self._cur_vel_mult = VEL_MULTS[button_idx] 
        self.start_moving.emit(jnt_idx, self._cur_speed, self._cur_vel_mult)

    def _stop_move(self):
        self._cur_jnt_moving = None
        self._cur_vel = 0.0
        self.stop_moving.emit()

    def _diagnostics_cb(self, msg):
        for status in msg.status:
            if "UR arm joint" in status.name:
                joint_mode = status.values[0].value
                for i, jnt_name in enumerate(JOINT_SHORT_NAMES):
                    if jnt_name in status.name:
                        self._joint_modes[i] = joint_mode
                        break
        self.state_changed.emit()

    def _joint_states_cb(self, msg):
        if self._q_order is None:
            self._q_order = []
            for jnt_name in JOINT_SHORT_NAMES:
                self._q_order.append(msg.name.index(jnt_name))

        for i, q_idx in enumerate(self._q_order):
            self._q_pos[i] = msg.position[q_idx]
        self.state_changed.emit()
