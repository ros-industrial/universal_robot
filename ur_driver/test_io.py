#!/usr/bin/env python
from __future__ import print_function
import time
from ur_driver.io_interface import *

if __name__ == "__main__":
    print("testing io-interface")
    get_states()
    print("listener has been activated")
    set_states()
    print("service-server has been started")
    while(True):
        set_tool_voltage(12)
        set_digital_out(0, True)
        set_analog_out(0, 0.75)
        #print "Flags are currently not supported"
        ##set_flag(0, True)
        ##print(Flag_States[0])
        print(Analog_Out_States[0])
        print(Digital_Out_States[0])
        time.sleep(1)
        set_tool_voltage(24)
        set_digital_out(0, False)
        set_analog_out(0, 0.25)
        #print "Flags are currently not supported"
        ##set_flag(0, False)
        ##print(Flag_States[0])
        print(Analog_Out_States[0])
        print(Digital_Out_States[0])
        time.sleep(1)


