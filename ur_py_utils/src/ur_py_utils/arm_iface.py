import numpy as np

import rospy
from rospy import ROSException
from std_msgs.msg import Bool, Int32, Int32MultiArray, Empty, Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

class ArmInterface(object):
    CONTROL_RATE = 125
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    def __init__(self, js_prefix='', config_prefix='/config_fwd_ctrl'):
        self.js_prefix = js_prefix

        self.joint_state_inds = None

        self._pub_open_iface = rospy.Publisher(config_prefix + '/open_interface', Empty)
        self._pub_close_iface = rospy.Publisher(config_prefix + '/close_interface', Empty)
        self._pub_power_on = rospy.Publisher(config_prefix + '/power_on_robot', Empty)
        self._pub_power_off = rospy.Publisher(config_prefix + '/power_off_robot', Empty)
        self._pub_unlock_sstop = rospy.Publisher(config_prefix + '/unlock_security_stop', Empty)
        self._pub_set_sstop = rospy.Publisher(config_prefix + '/set_security_stop', Int32MultiArray)

        self._pub_vel = rospy.Publisher('/velocity_fwd_ctrl/command', Float64MultiArray)
        self._pub_pva = rospy.Publisher('/pos_vel_acc_fwd_ctrl/command', JointTrajectoryPoint)
    
    def is_emergency_stopped(self, timeout=0.3):
        try:
            return rospy.wait_for_message('/is_emergency_stopped', Bool, timeout).data
        except ROSException as e:
            rospy.logwarn('is_emergency_stopped timed out after %1.1f s' % timeout)
            return None

    def is_extra_button_pressed(self, timeout=0.3):
        try:
            return rospy.wait_for_message('/is_extra_button_pressed', Bool, timeout).data
        except ROSException as e:
            rospy.logwarn('is_extra_button_pressed timed out after %1.1f s' % timeout)
            return None

    def is_power_button_pressed(self, timeout=0.3):
        try:
            return rospy.wait_for_message('/is_power_button_pressed', Bool, timeout).data
        except ROSException as e:
            rospy.logwarn('is_power_button_pressed timed out after %1.1f s' % timeout)
            return None

    def is_power_on_robot(self, timeout=0.3):
        try:
            return rospy.wait_for_message('/is_power_on_robot', Bool, timeout).data
        except ROSException as e:
            rospy.logwarn('is_power_on_robot timed out after %1.1f s' % timeout)
            return None

    def is_safety_sig_should_stop(self, timeout=0.3):
        try:
            return rospy.wait_for_message('/is_safety_signal_such_that_we_should_stop', 
                                          Bool, timeout).data
        except ROSException as e:
            rospy.logwarn('is_safety_sig_should_stop timed out after %1.1f s' % timeout)
            return None

    def is_security_stopped(self, timeout=0.3):
        try:
            return rospy.wait_for_message('/is_security_stopped', Bool, timeout).data
        except ROSException as e:
            rospy.logwarn('is_security_stopped timed out after %1.1f s' % timeout)
            return None

    def joint_mode_ids(self, timeout=0.3):
        try:
            return rospy.wait_for_message('/joint_mode_ids', Int32MultiArray, timeout).data
        except ROSException as e:
            rospy.logwarn('joint_mode_ids timed out after %1.1f s' % timeout)
            return None

    def robot_mode_id(self, timeout=0.3):
        try:
            return rospy.wait_for_message('/robot_mode_id', Int32, timeout).data
        except ROSException as e:
            rospy.logwarn('robot_mode_id timed out after %1.1f s' % timeout)
            return None

    def power_on_robot(self):
        self._pub_power_on.publish(Empty())

    def power_off_robot(self):
        self._pub_power_off.publish(Empty())

    def open_interface(self):
        self._pub_open_iface.publish(Empty())

    def close_interface(self):
        self._pub_close_iface.publish(Empty())

    def unlock_security_stop(self):
        self._pub_unlock_sstop.publish(Empty())

    def set_security_stop(self):
        stop_msg = Int32MultiArray()
        stop_msg.data = [0, 110, 0]
        self._pub_set_sstop.publish(stop_msg)

    def cmd_vel(self, qd):
        cmd = Float64MultiArray()
        cmd.data = np.array(qd).tolist()
        self._pub_vel.publish(cmd)

    def cmd_pos_vel_acc(self, q, qd, qdd):
        cmd = JointTrajectoryPoint()
        cmd.positions = np.array(q).tolist()
        cmd.velocities = np.array(qd).tolist()
        cmd.accelerations = np.array(qdd).tolist()
        self._pub_pva.publish(cmd)

    def get_joint_state(self, timeout=1.1):
        try:
            js = rospy.wait_for_message(self.js_prefix + '/joint_states', JointState, timeout)
        except ROSException as e:
            rospy.logwarn('get_joint_state timed out after %1.1f s' % timeout)
            return None
        if self.joint_state_inds is None:
            self.joint_state_inds = [js.name.index(joint_name) for 
                                     joint_name in ArmInterface.JOINT_NAMES]
        return js

    def get_q(self, timeout=1.1):
        js = self.get_joint_state(timeout)
        if js is None:
            return None
        return js.position

    def get_qd(self, timeout=1.1):
        js = self.get_joint_state(timeout)
        if js is None:
            return None
        return js.velocity

    def get_effort(self, timeout=1.1):
        js = self.get_joint_state(timeout)
        if js is None:
            return None
        return js.effort

    def shutdown(self):
        self._pub_open_iface.unregister()
        self._pub_close_iface.unregister()
        self._pub_power_on.unregister()
        self._pub_power_off.unregister()
        self._pub_unlock_sstop.unregister()
        self._pub_set_sstop.unregister()
        self._pub_vel.unregister()
        self._pub_pva.unregister()

