#!/usr/bin/env python
import ur_IO
import time

if __name__ == "__main__":
    print "toggling things"
    ur_IO.get_states()
    print "listener has been activated"
    while(True):
        ur_IO.set_tool_voltage(12)
        ur_IO.set_digital_out(0, True)
        ur_IO.set_analog_out(0, 0.75)
        ur_IO.set_flag(0, True)
        print(ur_IO.Flag_States[0])
        print(ur_IO.Analog_Out_States[0])
        print(ur_IO.Digital_Out_States[0])
        time.sleep(1)
        ur_IO.set_tool_voltage(24)
        ur_IO.set_digital_out(0, False)
        ur_IO.set_analog_out(0, 0.25)
        ur_IO.set_flag(0, False)
        print(ur_IO.Flag_States[0])
        print(ur_IO.Analog_Out_States[0])
        print(ur_IO.Digital_Out_States[0])
        time.sleep(1)


