#!/usr/bin/env python

import sys
import rospy
from ur_msgs.srv import *
from ur_msgs.msg import *

FUN_SET_DIGITAL_OUT = 1
FUN_SET_FLAG = 2
FUN_SET_ANALOG_OUT = 3
FUN_SET_TOOL_VOLTAGE = 4

#Flag_States = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
Digital_Out_States = [0,0,0,0,0,0,0,0,0,0]  #8(controller)+2(tool)
Digital_In_States = [0,0,0,0,0,0,0,0,0,0]   #8(controller)+2(tool)
Analog_Out_States = [0,0]  #2(controller)
Analog_In_States = [0,0]   #2(controller)+0(tool)

ANALOG_TOLERANCE_VALUE = 0.01

def set_io_val(fun, pin, val):
    try:
        set_io(fun, pin, val)
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

def set_tool_voltage(volts):
    try:
        set_io(FUN_SET_TOOL_VOLTAGE, volts, 0)
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

def set_digital_out(pin, val):
    try:
        set_io(FUN_SET_DIGITAL_OUT, pin, val)
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

def set_analog_out(pin, val):
    try:
        set_io(FUN_SET_ANALOG_OUT, pin, val)
    except rospy.ServiceException as e:
        print "Service call failed: %s"%e

def set_flag(pin, val):
    rospy.logerr("SETTING FLAGS IS NOT SUPPORTED!")
    #try:
        #set_io(FUN_SET_FLAG, pin, val)
    #except rospy.ServiceException, e:
        #print "Service call failed: %s"%e

def callback(data):
    rospy.logerr("Flag_States are currently not supported")
    #for i in range(0,32):
        #del Flag_States[i]
        #Flag_States.insert(i, data.flag_states[i].state)
    for i in range(0,10):
        del Digital_Out_States[i]
        Digital_Out_States.insert(i, data.digital_out_states[i].state)
    for i in range(0,10):
        del Digital_In_States[i]
        Digital_In_States.insert(i, data.digital_in_states[i].state)
    for i in range(0,2):
        del Analog_Out_States[i]
        Analog_Out_States.insert(i, data.analog_out_states[i].state)
    rospy.logerr("ToolInput analog_in[2] and analog_in[3] currently not supported")
    for i in range(0,2):
        del Analog_In_States[i]
        Analog_In_States.insert(i, data.analog_in_states[i].state)

def get_states():
    rospy.init_node('UR_State_Getter')
    rospy.Subscriber("io_states", IOStates, callback)
    
def set_states():
    rospy.wait_for_service('set_io')
    global set_io
    set_io = rospy.ServiceProxy('set_io', SetIO)

