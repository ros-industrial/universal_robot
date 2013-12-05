#!/usr/bin/python

import copy

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Float64MultiArray
import rospy
import actionlib

from ur_py_utils.arm_iface import ArmInterface

def main():
    rospy.init_node("test_traj_ctrl")
    act_cli = actionlib.SimpleActionClient(
            '/vel_trajectory_ctrl/follow_joint_trajectory', 
            FollowJointTrajectoryAction)
    if not act_cli.wait_for_server(rospy.Duration.from_sec(1.)):
        rospy.logerr("Can't find action server")
        return
    arm = ArmInterface()
    q_cur = arm.get_q().tolist()
    q_cur[2] += 0.2
    q_cur[5] += 0.2
    jtp1 = JointTrajectoryPoint()
    jtp1.positions = q_cur
    jtp1.velocities = 6*[0.0]
    jtp1.velocities[2] = 0.1
    jtp1.velocities[5] = 0.1
    jtp1.accelerations = 6*[0.0]
    jtp1.time_from_start = rospy.Duration.from_sec(5.)
    jtp2 = copy.deepcopy(jtp1)
    jtp2.positions[2] -= 0.2
    jtp2.positions[5] -= 0.2
    jtp2.velocities = 6*[0.0]
    jtp2.time_from_start = rospy.Duration.from_sec(10.)
    fjt = FollowJointTrajectoryGoal()
    fjt.trajectory.header.stamp = rospy.Time.now()
    fjt.trajectory.joint_names = arm.JOINT_NAMES
    fjt.trajectory.points = [jtp1, jtp2]

    act_cli.send_goal(fjt)
    rospy.loginfo("Starting trajectory")
    act_cli.wait_for_result()
    rospy.loginfo("Trajectory complete")

if __name__ == "__main__":
    main()
