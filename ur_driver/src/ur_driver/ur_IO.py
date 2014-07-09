#!/usr/bin/env python

import sys
import rospy
from ur_driver.srv import *
from ur_driver.msg import *

FUN_SET_DIGITAL_OUT = 1
FUN_SET_FLAG = 2
FUN_SET_ANALOG_OUT = 3
FUN_SET_TOOL_VOLTAGE = 4

Flag_States = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
Digital_Out_States = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
Digital_In_States = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
Analog_Out_States = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
Analog_In_States = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

ANALOG_TOLERANCE_VALUE = 0.01
DELAY_TIME = 0.3
i = 0
set_io = rospy.ServiceProxy('set_io', SetIO)

def set_io_val(fun, pin, val):
    rospy.wait_for_service('set_io')
    try:
        set_io(fun, pin, val)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def set_tool_voltage(volts):
    set_io_val(FUN_SET_TOOL_VOLTAGE, volts, 0)

def set_digital_out(pin, val):
    set_io_val(FUN_SET_DIGITAL_OUT, pin, val)
    if Digital_Out_States[pin] != val:
        rospy.logwarn("DIGITAL OUT IS NOT A HAPPY CAMPER, its not setting a pin on the first try, it may help to increase IO_SLEEP_TIME in driver.py")
        set_digital_out(pin, val)

def set_analog_out(pin, val):
    set_io_val(FUN_SET_ANALOG_OUT, pin, val)
    if abs(Analog_Out_States[pin] - val) > ANALOG_TOLERANCE_VALUE:
        rospy.logwarn("ANALOG OUT IS NOT A HAPPY CAMPER, its not setting a pin on the first try, it may help to increase IO_SLEEP_TIME in driver.py")
        set_digital_out(pin, val)

def set_flag(pin, val):
    set_io_val(FUN_SET_FLAG, pin, val)
    if Flag_States[pin] != val:
        rospy.logwarn("SETTING A FLAG IS NOT A HAPPY CAMPER, its not setting a pin on the first try, it may help to increase IO_SLEEP_TIME in driver.py")
        set_digital_out(pin, val)

def callback(data):
    for i in range(0,32):
        del Flag_States[i]
        Flag_States.insert(i, data.flag_states[i].state)
    for i in range(0,10):
        del Digital_Out_States[i]
        Digital_Out_States.insert(i, data.digital_out_states[i].state)
    for i in range(0,10):
        del Digital_In_States[i]
        Digital_In_States.insert(i, data.digital_in_states[i].state)
    for i in range(0,2):
        del Analog_Out_States[i]
        Analog_Out_States.insert(i, data.analog_out_states[i].state)
    for i in range(0,4):
        del Analog_In_States[i]
        Analog_In_States.insert(i, data.analog_in_states[i].state)

def get_states():
    rospy.init_node('UR_State_Getter')
    rospy.Subscriber("io_states", IOStates, callback)

