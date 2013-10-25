
import numpy as np

import rospy
from ur_py_utils.arm_iface import ArmInterface

class JointVelocityController(object):
    def __init__(self):
        self.moving_joint = None
        self.moving_dir = None
        self.delta_x = 0.01
        self.vel_f = 0.2
        self.t_f = 0.5 
        self.acc = self.vel_f / self.t_f
        self.pos_f = 0.5*self.acc*(self.t_f**2)
        self.t = 0.0
        js_prefix = rospy.get_param("~js_prefix", "")
        self.arm_iface = ArmInterface(js_prefix=js_prefix)
        self.q_init = None

    def update(self):
        if self.moving_joint is not None:
            t, t_f, vel_f, acc, pos_f = self.t, self.t_f, self.vel_f, self.acc, self.pos_f
            q_cmd = self.q_init.copy()
            qd_cmd = np.zeros(6)
            qdd_cmd = np.zeros(6)
            if t < t_f:
                q_cmd[self.moving_joint] += self.moving_dir*(0.5*acc*(t**2))
                qd_cmd[self.moving_joint] += self.moving_dir*(acc*t)
                qdd_cmd[self.moving_joint] += self.moving_dir*(acc)
            else:
                q_cmd[self.moving_joint] += self.moving_dir*(vel_f*(t-t_f) + pos_f)
                qd_cmd[self.moving_joint] = self.moving_dir*vel_f
            self.arm_iface.cmd_pos_vel_acc(q_cmd, qd_cmd, qdd_cmd)
            self.t += 1.0/self.arm_iface.CONTROL_RATE

    def start_moving(self, joint_ind, direction, vel):
        self.arm_iface.unlock_security_stop()
        self.moving_joint = joint_ind
        self.moving_dir = direction
        self.t = 0.0
        self.vel_f = vel
        self.acc = self.vel_f / self.t_f
        self.pos_f = 0.5*self.acc*(self.t_f**2)
        self.q_init = np.array(self.arm_iface.get_q())

    def stop_moving(self):
        self.moving_joint = None
