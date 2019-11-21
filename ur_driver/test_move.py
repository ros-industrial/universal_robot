#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

qpos_up = [0, -1.5707, 0, -1.5707, 0, 0]
q1 = [0, -1.5707, 0.35, -1.5707, 0.35, 0]
q2 = [0, -1.5707, -0.35, -1.5707, -0.35, 0]

def get_trajectory_goal(joint_names, q1, q2, q3):
  joint_num = len(jnt_names)
  g = FollowJointTrajectoryGoal()
  g.trajectory = JointTrajectory()
  g.trajectory.joint_names = joint_names
  g.trajectory.points = [
    JointTrajectoryPoint(positions=q1, velocities=[0]*joint_num, time_from_start=rospy.Duration(0.0)),
    JointTrajectoryPoint(positions=q2, velocities=[0]*joint_num, time_from_start=rospy.Duration(2.0)),
    JointTrajectoryPoint(positions=q3, velocities=[0]*joint_num, time_from_start=rospy.Duration(4.0)),
    JointTrajectoryPoint(positions=q1, velocities=[0]*joint_num, time_from_start=rospy.Duration(6.0))]
  return g
     
def user_info():
  rospy.logwarn("""The script shows an example of creating and sending 
  a goal using FollowJointTrajectory action interface.

  IMPORTANT: The script assumes that the robot is in upright position 
  with joint values '[0, -1.5707, 0, -1.5707, 0, 0]'
  
  Executing the script will move the robot's joint 3 and 5 by 20° in either direction. 
  Make sure there is enough free space for this movement, 
  especially if an end-effector is mounted on the robot.
  """)
    
def main():
  user_info()
  try:
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    
    traj_ac = actionlib.SimpleActionClient('/follow_joint_trajectory', FollowJointTrajectoryAction)
    print("Waiting for server...")
    traj_ac.wait_for_server()
    print("Connected to server")
    
    while not rospy.is_shutdown():
      goal = get_trajectory_goal(joint_names, qpos_up, q1, q2)
      rospy.logwarn("The robot is going to move now!")
      traj_ac.send_goal(goal)
      try:
        traj_ac.wait_for_result()
      except KeyboardInterrupt:
        rospy.loginfo("Interrupt requested")
        traj_ac.cancel_goal()
      raise
      
  except KeyboardInterrupt:
    rospy.signal_shutdown("KeyboardInterrupt")
    raise

if __name__ == '__main__': 
  main()
