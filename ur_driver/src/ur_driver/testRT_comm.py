#!/usr/bin/env python
import roslib; roslib.load_manifest('ur_driver')
import time, sys, threading, math
import copy
import datetime
import socket, select
import struct
import traceback, code
import SocketServer

import rospy

from sensor_msgs.msg import JointState
from ur_driver.deserializeRT import RobotStateRT
from ur_msgs.msg import *


joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
joint_offsets = {}


def __on_packet(buf):
    stateRT = RobotStateRT.unpack(buf)
    
    msg = RobotStateRTMsg()
    msg.time = stateRT.time
    msg.q_target = stateRT.q_target
    msg.qd_target = stateRT.qd_target
    msg.qdd_target = stateRT.qdd_target
    msg.i_target = stateRT.i_target
    msg.m_target = stateRT.m_target
    msg.q_actual = stateRT.q_actual
    msg.qd_actual = stateRT.qd_actual
    msg.i_actual = stateRT.i_actual
    msg.tool_acc_values = stateRT.tool_acc_values
    msg.tcp_force = stateRT.tcp_force
    msg.tool_vector = stateRT.tool_vector
    msg.tcp_speed = stateRT.tcp_speed
    msg.digital_input_bits = stateRT.digital_input_bits
    msg.motor_temperatures = stateRT.motor_temperatures
    msg.controller_timer = stateRT.controller_timer
    msg.test_value = stateRT.test_value
    msg.robot_mode = stateRT.robot_mode
    msg.joint_modes = stateRT.joint_modes
    pub_robot_stateRT.publish(msg)       
    

    msg = JointState()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "From real-time state data"
    msg.name = joint_names
    msg.position = [0.0] * 6
    for i, q in enumerate(stateRT.q_actual):
        msg.position[i] = q + joint_offsets.get(joint_names[i], 0.0)
    msg.velocity = stateRT.qd_actual
    msg.effort = [0]*6
    pub_joint_statesRT.publish(msg)




def main():
    rospy.init_node('testRT_comm', disable_signals=True)
    
    global pub_joint_statesRT
    pub_joint_statesRT = rospy.Publisher('joint_statesRT', JointState, queue_size=1)
    global pub_robot_stateRT
    pub_robot_stateRT = rospy.Publisher('robot_stateRT', RobotStateRTMsg, queue_size=1)
    
    
    
    robot_hostname = '192.168.0.42'
    rt_port = 30003
    
    rt_socket = socket.create_connection((robot_hostname, rt_port))
    buf = ""
    
    while not rospy.is_shutdown():
        more = rt_socket.recv(4096)
        if more:
            buf = buf + more

            # Attempts to extract a packet
            packet_length = struct.unpack_from("!i", buf)[0]
            print("PacketLength: ", packet_length, "; BufferSize: ", len(buf))
            if len(buf) >= packet_length:
                packet, buf = buf[:packet_length], buf[packet_length:]
                __on_packet(packet)
        else:
            print("There is no more...")
    
    rt_socket.close()




if __name__ == '__main__': main()
