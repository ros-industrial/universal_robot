# This data structure contains the ToolData structure
# used by the Universal Robots controller

int8 ANALOG_INPUT_RANGE_CURRENT = 0
int8 ANALOG_INPUT_RANGE_VOLTAGE = 1

int8 analog_input_range2 # one of ANALOG_INPUT_RANGE_*
int8 analog_input_range3 # one of ANALOG_INPUT_RANGE_*
float64 analog_input2
float64 analog_input3
float32 tool_voltage_48v
uint8 tool_output_voltage
float32 tool_current
float32 tool_temperature

uint8 TOOL_BOOTLOADER_MODE = 249
uint8 TOOL_RUNNING_MODE = 253
uint8 TOOL_IDLE_MODE = 255

uint8 tool_mode # one of TOOL_*
