#!/usr/bin/env python
from __future__ import print_function
import time
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.utils import filter_by_type, filter_by_state
from controller_manager_msgs.utils import ControllerLister, ControllerManagerLister
from sensor_msgs.msg import JointState

t_stamps = [2.0, 4.0, 6.0]

DEFAULT_CONTROLLER_NS = ""

def get_trajectory_goal(_jnt_names, q1, q2, q3, t=t_stamps):
  joint_num = len(_jnt_names)
  g = FollowJointTrajectoryGoal()
  g.trajectory = JointTrajectory()
  g.trajectory.joint_names = _jnt_names
  g.trajectory.points = [
    JointTrajectoryPoint(positions=q1, velocities=[0]*joint_num, time_from_start=rospy.Duration(t[0])),
    JointTrajectoryPoint(positions=q2, velocities=[0]*joint_num, time_from_start=rospy.Duration(t[1])),
    JointTrajectoryPoint(positions=q3, velocities=[0]*joint_num, time_from_start=rospy.Duration(t[2]))]
  return g
  
def handle_send_goal(_traj_ac, g):
  _traj_ac.send_goal(g)
  try:
    _traj_ac.wait_for_result()
  except KeyboardInterrupt:
    _traj_ac.cancel_goal()
    raise

#Performs a sequence of joint movements once
def move1(_traj_ac, _jnt_names):
  q1, q2, q3 = get_traj_pts()
  g = get_trajectory_goal(_jnt_names, q1, q2, q3)
  handle_send_goal(_traj_ac, g)
    
#Sends a trajectory goal with a 'disordered' joint values
def move_disordered(_traj_ac, _jnt_names):
  order = [4, 2, 3, 1, 5, 0]
  q1, q2, q3 = get_traj_pts()
  q1 = [q1[i] for i in order]
  q2 = [q2[i] for i in order]
  q3 = [q3[i] for i in order]
  _jnt_names = [_jnt_names[i] for i in order]
  g = get_trajectory_goal(_jnt_names, q1, q2, q3)
  handle_send_goal(_traj_ac, g)
    
#Repeats the sequence of movements in a loop
def move_repeated(_traj_ac, _jnt_names):
  t = t_stamps
  q1, q2, q3 = get_traj_pts()
  for i in range(5):
    t = t + [t[-1]+2, t[-1]+4, t[-1]+6]
    g = get_trajectory_goal(_jnt_names, q1, q2, q3, t)
    handle_send_goal(_traj_ac, g)

#Resends the same goal after 3 seconds delay
#The already running goal is cancelled and the arm moves back to the start pose before executing the trajectory again
def move_interrupt(_traj_ac, _jnt_names):
  q1, q2, q3 = get_traj_pts()
  g = get_trajectory_goal(_jnt_names, q1, q2, q3)
  _traj_ac.send_goal(g)
  time.sleep(3.0)
  print("Interrupting")
  handle_send_goal(_traj_ac, g)

def get_joint_state():
  return rospy.wait_for_message('/joint_states', JointState)

def get_current_joint_vals():
  return list(get_joint_state().position)

def get_joint_names():
  return get_joint_state().name
  
#Adds/subtracts 0.3 radians to the last 2 joint values
def get_traj_pts():
  pos = get_current_joint_vals()
  q1 = pos[:4] + [x + 0.3 for x in pos[4:]]
  q2 = pos[:4] + [x - 0.3 for x in pos[4:]]
  q3 = pos
  return q1, q2, q3

def get_controller():
  _list_cm = ControllerManagerLister()
  controller_list = ControllerLister(_list_cm()[0])
  _list_controllers = controller_list()
    
  jtc_list = filter_by_type(_list_controllers,
                            'JointTrajectoryController',
                            match_substring=True)
  running_jtc_list = filter_by_state(jtc_list, 'running')
  return running_jtc_list[0]

def user_menu():
  choice=True
  print("""
  1.Perform a pre-defined sequence of joint movements once
  2.Perform a pre-defined sequence of joint movements in a loop
  3.Set joint values in JointTrajectory in a disordered way
    and perform a pre-defined sequence of joint movements once
  4.Send a pre-defined joint trajectory twice 
    cancels the first goal and starts the new one
  5.Exit/Quit
  """)
  choice=raw_input("What would you like to do? ") 
  return choice
    
def main():
  _use_ros_control = False
  try:
    rospy.init_node("test_move", anonymous=True, disable_signals=True)
    print("""
    Usage: You can use this node alongwith or without ros_control
           In case of without, make sure to pass the namespace of FollowJointTrajectoryAction as the first argument.
           Else, an empty namespace is assumed.
    """)
    
    if _use_ros_control == True:
      controller_ns = get_controller().name 
    elif(len(sys.argv) > 0):
      controller_ns = sys.argv[1]
    else:
      controller_ns = DEFAULT_CONTROLLER_NS
      
    JOINT_NAMES = get_joint_names()
    traj_ac = actionlib.SimpleActionClient(controller_ns + '/follow_joint_trajectory', FollowJointTrajectoryAction)
    print("Waiting for server...")
    traj_ac.wait_for_server()
    print("Connected to server")
    
    while not rospy.is_shutdown():
      choice = user_menu()
      if choice=="1":  
        move1(traj_ac, JOINT_NAMES)
      elif choice=="2":
        move_repeated(traj_ac, JOINT_NAMES)
      elif choice=="3": 
        move_disordered(traj_ac, JOINT_NAMES)
      elif choice=="4":
        move_interrupt(traj_ac, JOINT_NAMES)
      elif choice=="5":
        print("\n Exiting the program.") 
        exit()
      else:
        print("\n Invalid choice, try again!") 

  except KeyboardInterrupt:
    rospy.signal_shutdown("KeyboardInterrupt")
    raise

if __name__ == '__main__': main()
