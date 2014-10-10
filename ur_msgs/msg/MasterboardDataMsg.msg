# This data structure contains the MasterboardData structure
# used by the Universal Robots controller
#
# MasterboardData is part of the data structure being send on the 
# secondary client communications interface
# 
# This data structure is send at 10 Hz on TCP port 30002
# 
# Dokumentation can be found on the Universal Robots Support Wiki
# (http://wiki03.lynero.net/Technical/DataStreamFromURController?rev=8)

int16 digital_input_bits
int16 digital_output_bits
int8 analog_input_range0
int8 analog_input_range1
float64 analog_input0
float64 analog_input1
int8 analog_output_domain0
int8 analog_output_domain1
float64 analog_output0
float64 analog_output1
float32 masterboard_temperature
float32 robot_voltage_48V
float32 robot_current
float32 master_io_current
uint8 master_safety_state
uint8 master_onoff_state
