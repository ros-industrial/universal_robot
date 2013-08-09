#!/usr/bin/python

from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import rospy

def main():
    rospy.init_node("test_fwd_ctrl")
    pva_pub = rospy.Publisher("/pos_vel_acc_fwd_ctrl/command", JointTrajectoryPoint)
    vel_pub = rospy.Publisher("/velocity_fwd_ctrl/command", Float64MultiArray)
    r = rospy.Rate(1)
    jtp = JointTrajectoryPoint()
    vel = Float64MultiArray()
    p, v, a = 0.0, 0.0, 0.0
    while not rospy.is_shutdown():
        jtp.positions = [p]*6
        jtp.velocities = [v]*6
        jtp.accelerations = [a]*6
        vel.data = [v]*6
        pva_pub.publish(jtp)
        vel_pub.publish(vel)
        p += 0.1
        v += 1.0
        a += 10.0
        r.sleep()

if __name__ == "__main__":
    main()
