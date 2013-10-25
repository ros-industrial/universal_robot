#! /usr/bin/python

from threading import Lock
import numpy as np
#from openravepy import *

import roslib
roslib.load_manifest("ur_cart_move")
roslib.load_manifest("ur_controller_manager")
roslib.load_manifest("hrl_geom")

import rospy
import roslaunch.substitution_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench, Twist

from hrl_geom.pose_converter import PoseConv
from hrl_geom.transformations import rotation_from_matrix as mat_to_ang_axis_point
from hrl_geom.transformations import rotation_matrix as ang_axis_point_to_mat
from hrl_geom.transformations import euler_matrix
from ur_controller_manager.msg import URJointCommand, URModeStates, URJointStates, URModeCommand

roslib.load_manifest("pykdl_utils")
from pykdl_utils.kdl_kinematics import create_kdl_kin
from ur_kin_py import inverse, forward

from ur_analytical_ik import inverse_kin, UR10_A, UR10_D, UR10_L

class RAVEKinematics(object):
    def __init__(self, robot_file='$(find ur10_description)/ur10_robot.dae', load_ik=True):
        robot_file = roslaunch.substitution_args.resolve_args(robot_file)
        #self.env = Environment()
        #self.env.Load(robot_file)
        #self.robot = self.env.GetRobots()[0] # get the first robot
        #self.manip = self.robot.SetActiveManipulator('arm')
        #self.manip = self.robot.GetManipulators()[0]
    #    self.ikmodel3d = databases.inversekinematics.InverseKinematicsModel(
    #                         self.robot,iktype=IkParameterization.Type.Translation3D)
        #self.ikmodel6d = databases.inversekinematics.InverseKinematicsModel(
        #                     self.robot,iktype=IkParameterization.Type.Transform6D)
        robot_urdf = roslaunch.substitution_args.resolve_args(
                '$(find ur10_description)/ur10_robot.urdf')
        self.kdl_kin = create_kdl_kin('/base_link', '/ee_link', robot_urdf)
        #if True:
        #    self.ik_options = (IkFilterOptions.IgnoreCustomFilters |
        #                       IkFilterOptions.IgnoreJointLimits)
        #else:
        #    self.ik_options = IkFilterOptions.CheckEnvCollisions

        #if False and load_ik:
        #    self.load_ik_model()

    #def load_ik_model(self):
    #    if self.ikmodel6d.load():
    #        print 'Model loaded'
    #    else:
    #        print 'No model, generating...',
    #        self.ikmodel6d.autogenerate()
    #        print 'model generated!'
    #    if self.ikmodel3d.load():
    #        print 'Model loaded'
    #    else:
    #        print 'No model, generating...',
    #        self.ikmodel3d.autogenerate()
    #        print 'model generated!'

    def forward(self, q):
        #self.robot.SetDOFValues(q)
        return np.mat(forward(np.array(q)))

    def inverse_all(self, x, q_guess=None, q_min=6*[-2.*np.pi], q_max=6*[2.*np.pi]):
        if q_guess is None:
            q_guess = np.zeros(6)
        q_min = np.array(q_min)
        q_max = np.array(q_max)
        sols = inverse(np.array(x), q_guess[5])
        closest_sols = []
        for sol in sols:
            test_sol = np.ones(6)*9999.
            for i in range(6):
                for add_ang in [-2.*np.pi, 0]:
                    test_ang = sol[i] + add_ang
                    if (test_ang >= q_min[i] and test_ang <= q_max[i] and
                        abs(test_ang) < 2.*np.pi and 
                        abs(test_ang - q_guess[i]) < abs(test_sol[i] - q_guess[i])):
                        test_sol[i] = test_ang
            if np.all(test_sol != 9999.):
                # sanity check for inverse_kin stuff
                if np.allclose(np.linalg.inv(self.forward(test_sol)) * x, np.eye(4)):
                    closest_sols.append(test_sol)
        return closest_sols

    def inverse(self, x, q_guess=None, q_min=6*[-2.*np.pi], q_max=6*[2.*np.pi], weights=6*[1.]):
        closest_sols = self.inverse_all(x, q_guess=q_guess, 
                                        q_min=6*[-2.*np.pi], q_max=6*[2.*np.pi])
        best_sol_ind = np.argmin(np.sum((weights*(closest_sols - np.array(q_guess)))**2,1))
        best_sol = closest_sols[best_sol_ind]
        if False:
            print 'q_guess', q_guess
            print 'q_resid', q_resid
            print 'sols', sols 
            print 'closest_sols', closest_sols 
            print 'best_sol', best_sol 
        return best_sol

    def inverse_rand_search(self, x, q_guess=None, pos_tol=0.01, rot_tol=20.0/180.0*np.pi, 
                            restarts=10, 
                            q_min=6*[-2.*np.pi], q_max=6*[2.*np.pi], weights=6*[1.], options=None):
        num_restart = 0
        while not rospy.is_shutdown():
            if num_restart == 0:
                x_try = x
            else:
                x_diff = PoseConv.to_homo_mat((2*(np.random.rand(3)-0.5)*pos_tol).tolist(),
                                              (2*(np.random.rand(3)-0.5)*rot_tol).tolist())
                x_try = x.copy()
                x_try[:3,:3] = x_diff[:3,:3] * x_try[:3,:3]
                x_try[:3,3] += x_diff[:3,3]
            sol = self.inverse(x_try, q_guess, restarts=1, 
                               q_min=q_min, q_max=q_max, weights=weights, options=options)
            if sol is not None:
                return sol
            if num_restart == restarts:
                return None
            num_restart += 1
        return None

    #def jacobian(self, q):
    #    #self.robot.SetDOFValues(q)
    #    return np.bmat([[self.manip.CalculateJacobian()],
    #                   [self.manip.CalculateAngularVelocityJacobian()]])

    def inverse_velocity(self, x_delta, q_cur):
        pos, euler = PoseConv.to_pos_euler(x_delta)
        x_twist = np.mat(pos + euler).T
        q_sol, residues, rank, s = np.linalg.lstsq(self.jacobian(q_cur), x_twist)
        if np.shape(residues)[1] == 0:
            residues = np.mat([[0.]])
        return q_sol.A.T[0], residues[0,0]

    def wrap_angles(self, q):
        q_resid = 2.0*np.pi*(np.array(q) > np.pi) + -2.0*np.pi*(np.array(q) < -np.pi)
        q_wrapped = q - q_resid
        return q_wrapped, q_resid

    #def is_self_colliding(self, q):
    #    self.robot.SetDOFValues(q)
    #    return self.robot.CheckSelfCollision()

class ArmInterface(object):
    CONTROL_RATE = 125
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    def __init__(self, timeout=3., topic_prefix=""):
        if topic_prefix == '/':
            topic_prefix = ''
        if topic_prefix == '':
            print_prefix = '/'
        else:
            print_prefix = topic_prefix
        self.joint_state_inds = None
        self.joint_states = None 
        self.ur_joint_states = None 
        self.ur_mode_states = None 
        self.message_history = []

        rospy.Subscriber(topic_prefix+'/joint_states', JointState, self._joint_states_cb)
        rospy.Subscriber(topic_prefix+'/ur_joint_states', URJointStates, self._ur_joint_states_cb)
        rospy.Subscriber(topic_prefix+'/ur_mode_states', URModeStates, self._ur_mode_states_cb)
        self.joint_cmd_pub = rospy.Publisher(topic_prefix+"/ur_joint_command", URJointCommand)
        self.mode_cmd_pub = rospy.Publisher(topic_prefix+"/ur_mode_command", URModeCommand)
        self.tcp_pub = rospy.Publisher(topic_prefix+"/ur_set_tcp", Twist)
        self.tcp_payload_pub = rospy.Publisher(topic_prefix+"/ur_set_tcp_payload", Float64)
        self.tcp_wrench_pub = rospy.Publisher(topic_prefix+"/ur_set_tcp_wrench", Wrench)

        print "%s UR arm: Connecting interface" % print_prefix
        if not self.wait_for_states(timeout) and timeout > 0.:
            print "%s UR arm: Unable to connect" % print_prefix
        else:
            print "%s UR arm: Succesfully connected" % print_prefix

    def wait_for_states(self, timeout=10.):
        start_time = rospy.get_time()
        timed_out = True
        while not rospy.is_shutdown() and rospy.get_time() - start_time < timeout:
            if (self.joint_states is not None and 
                self.ur_joint_states is not None and
                self.ur_mode_states is not None):
                timed_out = False
                break
        return not timed_out

    def _joint_states_cb(self, joint_states):
        self.joint_states = joint_states
        if self.joint_state_inds is None:
            self.joint_state_inds = [joint_states.name.index(joint_name) for 
                                     joint_name in ArmInterface.JOINT_NAMES]

    def _ur_joint_states_cb(self, ur_joint_states):
        self.ur_joint_states = ur_joint_states

    def _ur_mode_states_cb(self, ur_mode_states):
        self.ur_mode_states = ur_mode_states
        self.message_history.extend(ur_mode_states.messages)

    def get_q(self):
        #return np.array(self.joint_states.q_act)
        return np.array(self.joint_states.position)[self.joint_state_inds]

    def get_qd(self):
        #return np.array(self.joint_states.qd_act)
        return np.array(self.joint_states.velocity)[self.joint_state_inds]

    def get_effort(self):
        #return np.array(self.joint_states.qd_act)
        return np.array(self.joint_states.effort)[self.joint_state_inds]

    def get_q_des(self):
        return np.array(self.ur_joint_states.q_des)

    def get_qd_des(self):
        return np.array(self.ur_joint_states.qd_des)

    def get_qdd_des(self):
        return np.array(self.ur_joint_states.qdd_des)

    # pose of the payload in the end link
    # payload in kg
    def set_payload(self, pose=None, payload=None):
        if pose is not None:
            self.tcp_pub.publish(PoseConv.to_twist_msg(pose))
        if payload is not None:
            self.tcp_payload_pub.publish(Float64(payload))

    # wrench felt by the end effector in the base link
    def set_felt_wrench(self, wrench):
        w = Wrench()
        w.force.x, w.force.y, w.force.z  = wrench[0], wrench[1], wrench[2]
        w.torque.x, w.torque.y, w.torque.z  = wrench[3], wrench[4], wrench[5]
        self.tcp_wrench_pub.publish(w)

    def cmd_empty(self):
        cmd = URJointCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.mode = URJointCommand.CMD_EMPTY
        self.joint_cmd_pub.publish(cmd)

    def cmd_vel(self, qd):
        cmd = URJointCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.mode = URJointCommand.CMD_VELOCITY
        cmd.qd_des = qd
        self.joint_cmd_pub.publish(cmd)

    def cmd_pos_vel_acc(self, q, qd, qdd):
        cmd = URJointCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.mode = URJointCommand.CMD_POS_VEL_ACC
        cmd.q_des = np.array(q).tolist()
        cmd.qd_des = np.array(qd).tolist()
        cmd.qdd_des = np.array(qdd).tolist()
        self.joint_cmd_pub.publish(cmd)

    def set_ready_mode(self):
        cmd = URModeCommand()
        cmd.robot_ready_mode = True
        self.mode_cmd_pub.publish(cmd)

    def set_running_mode(self):
        cmd = URModeCommand()
        cmd.robot_running_mode = True
        self.mode_cmd_pub.publish(cmd)

    def set_freedrive_mode(self):
        cmd = URModeCommand()
        cmd.robot_freedrive_mode = True
        self.mode_cmd_pub.publish(cmd)

    def unlock_security_stop(self):
        cmd = URModeCommand()
        cmd.unlock_security_stop = True
        self.mode_cmd_pub.publish(cmd)

    def security_stop(self, joint_code=0, error_state=0, error_argument=0):
        cmd = URModeCommand()
        cmd.security_stop = True
        cmd.joint_code = joint_code
        cmd.error_state = error_state
        cmd.error_argument = error_argument
        self.mode_cmd_pub.publish(cmd)

    def is_security_stopped(self):
        return self.ur_mode_states.is_security_stopped

    def is_emergency_stopped(self):
        return self.ur_mode_states.is_emergency_stopped

    def is_running_mode(self):
        return self.ur_mode_states.robot_mode_id == 0

def min_jerk_traj(d, n, deriv=0):
    if deriv == 0:
        return [10.0*(t/d)**3-15.0*(t/d)**4+6*(t/d)**5 for t in np.linspace(0.,d,n)]
    elif deriv == 1:
        return [30.0/d*(t/d)**2-60.0/d*(t/d)**3+30.0/d*(t/d)**4 for t in np.linspace(0.,d,n)]
    elif deriv == 2:
        return [60.0/d**2*(t/d)-180.0/d**2*(t/d)**2+120.0/d**2*(t/d)**3 for t in np.linspace(0.,d,n)]
    else:
        return None

class ArmBehaviors(object):
    def __init__(self, arm, kin):
        self.kin = kin
        self.arm = arm

    def interpolated_ik(self, q_init, x_goal, duration, num_ik):
        x_init = self.kin.forward(q_init)
        pos_delta = x_goal[:3,3] - x_init[:3,3]
        rot_delta = x_init[:3,:3].T * x_goal[:3,:3]
        rot_delta_homo = np.eye(4)
        rot_delta_homo[:3,:3] = rot_delta
        ang_delta, axis_delta, point_delta = mat_to_ang_axis_point(rot_delta_homo)

        q_pts = []
        s_traj = min_jerk_traj(duration, num_ik)
        q_prev = q_init
        for s in s_traj:
            x_cur = x_init.copy()
            x_cur[:3,3] += s * pos_delta
            x_cur[:3,:3] *= ang_axis_point_to_mat(s * ang_delta, axis_delta, point_delta)[:3,:3]
            q_pt = self.kin.inverse(x_cur, q_prev)
            if q_pt is None:
                print x_init
                print "IK failed", q_pts, x_cur
                return None
            q_pts.append(q_pt)
            q_prev = q_pt
        q_pts = np.array(q_pts)
        return q_pts

    def interpolated_ik_delta(self, pos_delta=3*[0.], rot_delta=3*[0.], duration=5., num_ik=10):
        q_cur = self.arm.get_q()
        x_cur = self.kin.forward(q_cur)
        rot_trans = np.mat(euler_matrix(rot_delta[0], rot_delta[1], rot_delta[2])[:3,:3])
        x_goal = np.mat(np.eye(4))
        x_goal[:3,:3] = x_cur[:3,:3] * rot_trans
        x_goal[:3,3] = x_cur[:3,3] + np.mat(pos_delta).T
        q_pts = self.interpolated_ik(q_cur, x_goal, duration, num_ik)
        return q_pts

    def interpolated_q(self, q_init, q_final, duration=5., num_ik=10):
        s_traj = min_jerk_traj(duration, num_ik)
        return np.array([q_init + (q_final - q_init)*s for s in s_traj])

    def interpolated_q_delta(self, q_delta, duration=5., num_ik=10):
        q_init = self.arm.get_q()
        q_final = q_init + q_delta
        return self.interpolated_q(q_init, q_final, duration, num_ik)

    def exec_parab_blend(self, q_pts, duration=5., blend_delta=0.2):
        t_traj = np.linspace(0.5*blend_delta, duration+0.5*blend_delta, len(q_pts))
        seg = 0
        r = rospy.Rate(self.arm.CONTROL_RATE)
        start_time = rospy.get_time()
        for t in np.linspace(0., duration+blend_delta, duration*self.arm.CONTROL_RATE):
            if t < blend_delta:
                # first segment
                qd_avg_next = (q_pts[1] - q_pts[0]) / (t_traj[1] - t_traj[0])
                qdd_avg = qd_avg_next / blend_delta
                q = 0.5 * qdd_avg * t**2 + q_pts[0]
                qd = qdd_avg * t
                qdd = qdd_avg
            elif t >= duration:
                # last segment
                qd_avg_prev = (q_pts[-2] - q_pts[-1]) / (t_traj[-2] - t_traj[-1])
                qdd_avg = - qd_avg_prev / blend_delta
                t_f = duration + 0.5*blend_delta
                q = 0.5 * qdd_avg * (t-t_f)**2 + qd_avg_prev * (t-t_f) + q_pts[-2]
                qd = qdd_avg * (t-t_f) + qd_avg_prev
                qdd = qdd_avg
            else:
                # middle segments
                while t > t_traj[seg] + 0.5 * blend_delta:
                    seg += 1
                qd_avg_prev = (q_pts[seg] - q_pts[seg-1]) / (t_traj[seg] - t_traj[seg-1])
                t_i = t_traj[seg] - 0.5 * blend_delta
                if t >= t_i:
                    qd_avg_next = (q_pts[seg+1] - q_pts[seg]) / (t_traj[seg+1] - t_traj[seg])
                    qdd_avg = (qd_avg_next - qd_avg_prev) / blend_delta
                    q_i = q_pts[seg] - qd_avg_prev * 0.5 * blend_delta
                    q = 0.5 * qdd_avg * (t - t_i)**2 + qd_avg_prev * (t - t_i) + q_i
                    qd = qdd_avg * (t - t_i) + qd_avg_prev
                    qdd = qdd_avg
                else:
                    q = qd_avg_prev * (t - t_traj[seg-1]) + q_pts[seg-1]
                    qd = qd_avg_prev
                    qdd = np.zeros(6)
            #print np.array([t]), q, qd, qdd
            #print q
            self.arm.cmd_pos_vel_acc(q, qd, qdd)
            if not self.arm.is_running_mode():
                print 'ROBOT STOPPED'
                return False
            #print np.array([np.sum(np.fabs(np.array(qd)))])
            r.sleep()
        #print duration*self.arm.CONTROL_RATE / (rospy.get_time() - start_time) 
        return True

    def parab_blend(self, q_pts, duration=5., blend_delta=0.2):
        duration -= blend_delta # hack to make duration accurate
        t_traj = np.linspace(0.5*blend_delta, duration+0.5*blend_delta, len(q_pts))
        traj = []
        seg = 0
        for t in np.linspace(0., duration+blend_delta, duration*self.arm.CONTROL_RATE):
            if t < blend_delta:
                # first segment
                qd_avg_next = (q_pts[1] - q_pts[0]) / (t_traj[1] - t_traj[0])
                qdd_avg = qd_avg_next / blend_delta
                q = 0.5 * qdd_avg * t**2 + q_pts[0]
                qd = qdd_avg * t
                qdd = qdd_avg
            elif t >= duration:
                # last segment
                qd_avg_prev = (q_pts[-2] - q_pts[-1]) / (t_traj[-2] - t_traj[-1])
                qdd_avg = - qd_avg_prev / blend_delta
                t_f = duration + 0.5*blend_delta
                q = 0.5 * qdd_avg * (t-t_f)**2 + qd_avg_prev * (t-t_f) + q_pts[-2]
                qd = qdd_avg * (t-t_f) + qd_avg_prev
                qdd = qdd_avg
            else:
                # middle segments
                while t > t_traj[seg] + 0.5 * blend_delta:
                    seg += 1
                qd_avg_prev = (q_pts[seg] - q_pts[seg-1]) / (t_traj[seg] - t_traj[seg-1])
                t_i = t_traj[seg] - 0.5 * blend_delta
                if t >= t_i:
                    qd_avg_next = (q_pts[seg+1] - q_pts[seg]) / (t_traj[seg+1] - t_traj[seg])
                    qdd_avg = (qd_avg_next - qd_avg_prev) / blend_delta
                    q_i = q_pts[seg] - qd_avg_prev * 0.5 * blend_delta
                    q = 0.5 * qdd_avg * (t - t_i)**2 + qd_avg_prev * (t - t_i) + q_i
                    qd = qdd_avg * (t - t_i) + qd_avg_prev
                    qdd = qdd_avg
                else:
                    q = qd_avg_prev * (t - t_traj[seg-1]) + q_pts[seg-1]
                    qd = qd_avg_prev
                    qdd = np.zeros(6)
            traj.append((q, qd, qdd))
        return traj

    def move_to_q(self, q_final, velocity=0.1, blend_delta=0.2):
        q_init = self.arm.get_q()
        max_delta_q = np.max(np.fabs(q_final - q_init))
        duration = max_delta_q / velocity
        num_ik = 5 + int(duration * 2.)
        q_pts = self.interpolated_q(q_init, q_final, duration, num_ik)
        return self.exec_parab_blend(q_pts, duration, blend_delta)

    def move_to_x(self, x_final, velocity=0.1, blend_delta=0.2):
        q_init = self.arm.get_q()
        q_final = self.kin.inverse(x_final, q_init)
        return self.move_to_q(q_final, velocity, blend_delta)

def load_ur_robot(timeout=5., desc_filename='$(find ur10_description)/ur10_robot.dae',
                  topic_prefix=""):
    arm = ArmInterface(timeout=0., topic_prefix=topic_prefix)
    kin = RAVEKinematics(desc_filename)
    if not arm.wait_for_states(timeout=timeout):
        print 'arm not connected!'
    arm_behav = ArmBehaviors(arm, kin)
    return arm, kin, arm_behav

def main():
    import sys
    np.set_printoptions(precision=4, suppress=True)

    rospy.init_node("ur_cart_move")
    robot_descr = roslaunch.substitution_args.resolve_args('$(find ur10_description)/ur10_robot.dae')
    arm = ArmInterface(timeout=0.)
    kin = RAVEKinematics(robot_descr)
    if not arm.wait_for_states(timeout=5.):
        print 'arm not connected!'
        return
    print arm.get_q()
    arm_behav = ArmBehaviors(arm, kin)

    if True:
        pos_delta = np.array([float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])])
        rot_delta = np.array([float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6])])
        duration = float(sys.argv[7])
        num_ik = 5 + int(duration * 2.)

        if False:
            while not rospy.is_shutdown():
                q_pts = arm_behav.interpolated_ik_delta(pos_delta=pos_delta, rot_delta=rot_delta, duration=duration, num_ik=num_ik)
                print q_pts
                arm_behav.exec_parab_blend(q_pts, duration=duration, blend_delta=0.2)
                print arm.get_q()
                q_pts = arm_behav.interpolated_ik_delta(pos_delta=-pos_delta, rot_delta=-rot_delta, duration=duration, num_ik=num_ik)
                print q_pts
                arm_behav.exec_parab_blend(q_pts, duration=duration, blend_delta=0.2)
                print arm.get_q()
        if True:
            q_pts = arm_behav.interpolated_ik_delta(pos_delta=pos_delta, rot_delta=rot_delta, duration=duration, num_ik=num_ik)
            print q_pts
            arm_behav.exec_parab_blend(q_pts, duration=duration, blend_delta=0.2)
            print arm.get_q()

    if False:
        q_delta = np.array([float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]), 
                            float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6])])
        duration = float(sys.argv[7])
        num_ik = 5 + int(duration * 2.)
        while not rospy.is_shutdown():
            q_pts = arm_behav.interpolated_q_delta(q_delta, duration, num_ik)
            print q_pts
            arm_behav.exec_parab_blend(q_pts, duration=duration, blend_delta=0.2)
            print arm.get_q()
            q_pts = arm_behav.interpolated_q_delta(-q_delta, duration, num_ik)
            print q_pts
            arm_behav.exec_parab_blend(q_pts, duration=duration, blend_delta=0.2)
            print arm.get_q()

if __name__ == "__main__":
    main()
