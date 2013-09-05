import struct

class PackageType(object):
    ROBOT_MODE_DATA = 0
    JOINT_DATA = 1
    TOOL_DATA = 2
    MASTERBOARD_DATA = 3
    CARTESIAN_INFO = 4
    KINEMATICS_INFO = 5
    CONFIGURATION_DATA = 6
    FORCE_MODE_DATA = 7
    ADDITIONAL_INFO = 8

class RobotMode(object):
    RUNNING = 0
    FREEDRIVE = 1
    READY = 2
    INITIALIZING = 3
    SECURITY_STOPPED = 4
    EMERGENCY_STOPPED = 5
    FATAL_ERROR = 6
    NO_POWER = 7
    NOT_CONNECTED = 8
    SHUTDOWN = 9
    SAFEGUARD_STOP = 10

class JointMode(object):
    PART_D_CALIBRATION = 237
    BACKDRIVE = 238
    POWER_OFF = 239
    EMERGENCY_STOPPED = 240
    CALVAL_INITIALIZATION = 241
    ERROR = 242
    FREEDRIVE = 243
    SIMULATED = 244
    NOT_RESPONDING = 245
    MOTOR_INITIALISATION = 246
    ADC_CALIBRATION = 247
    DEAD_COMMUTATION = 248
    BOOTLOADER = 249
    CALIBRATION = 250
    STOPPED = 251
    FAULT = 252
    RUNNING = 253
    INITIALIZATION = 254
    IDLE = 255

class ToolMode(object):
    BOOTLOADER = 249
    RUNNING = 253
    IDLE = 255
    
class MasterSafetyState(object):
    UNDEFINED = 0
    BOOTLOADER = 1
    FAULT = 2
    BOOTING = 3
    INITIALIZING = 4
    ROBOT_EMERGENCY_STOP = 5
    EXTERNAL_EMERGENCY_STOP = 6
    SAFEGUARD_STOP = 7
    OK = 8

class MasterOnOffState(object):
    OFF = 0
    TURNING_ON = 1
    ON = 2
    TURNING_OFF = 3

class RobotModeData(object):
    __slots__ = ['timestamp', 'robot_connected', 'real_robot_enabled',
                 'power_on_robot', 'emergency_stopped',
                 'security_stopped', 'program_running', 'program_paused',
                 'robot_mode', 'speed_fraction']
    @staticmethod
    def unpack(buf):
        rmd = RobotModeData()
        (_, _,
         rmd.timestamp, rmd.robot_connected, rmd.real_robot_enabled,
         rmd.power_on_robot, rmd.emergency_stopped, rmd.security_stopped,
         rmd.program_running, rmd.program_paused, rmd.robot_mode,
         rmd.speed_fraction) = struct.unpack_from("!IBQ???????Bd", buf)
        return rmd

# Don't use T_micro (obsolete). For retrocompatibility purposes. 
class JointData(object):
    __slots__ = ['q_actual', 'q_target', 'qd_actual',
                 'I_actual', 'V_actual', 'T_motor', 'T_micro', 'joint_mode']
    @staticmethod
    def unpack(buf):
        all_joints = []
        offset = 5
        for i in range(6):
            jd = JointData()
            (jd.q_actual, jd.q_target, jd.qd_actual, jd.I_actual, jd.V_actual,
             jd.T_motor, jd.T_micro, 
             jd.joint_mode) = struct.unpack_from("!dddffffB", buf, offset)
            offset += 41
            all_joints.append(jd)
        return all_joints

class ToolData(object):
    __slots__ = ['analog_input_range2', 'analog_input_range3',
                 'analog_input2', 'analog_input3',
                 'tool_voltage_48V', 'tool_output_voltage', 'tool_current',
                 'tool_temperature', 'tool_mode']
    @staticmethod
    def unpack(buf):
        td = ToolData()
        (_, _,
         td.analog_input_range2, td.analog_input_range3,
         td.analog_input2, td.analog_input3,
         td.tool_voltage_48V, td.tool_output_voltage, td.tool_current,
         td.tool_temperature, td.tool_mode) = struct.unpack_from("!IBbbddfBffB", buf)
        return td

class MasterboardData(object):
    __slots__ = ['digital_input_bits', 'digital_output_bits',
                 'analog_input_range0', 'analog_input_range1',
                 'analog_input0', 'analog_input1',
                 'analog_output_domain0', 'analog_output_domain1',
                 'analog_output0', 'analog_output1',
                 'masterboard_temperature',
                 'robot_voltage_48V', 'robot_current',
                 'master_io_current', 'master_safety_state',
                 'master_onoff_state']
    @staticmethod
    def unpack(buf):
        md = MasterboardData()
        (_, _,
         md.digital_input_bits, md.digital_output_bits,
         md.analog_input_range0, md.analog_input_range1,
         md.analog_input0, md.analog_input1,
         md.analog_output_domain0, md.analog_output_domain1,
         md.analog_output0, md.analog_output1,
         md.masterboard_temperature,
         md.robot_voltage_48V, md.robot_current,
         md.master_io_current, md.master_safety_state,
         md.master_onoff_state) = struct.unpack_from("!IBhhbbddbbddffffBB", buf)
        return md

class CartesianInfo(object):
    __slots__ = ['x', 'y', 'z', 'rx', 'ry', 'rz']
    @staticmethod
    def unpack(buf):
        ci = CartesianInfo()
        (_, _,
         ci.x, ci.y, ci.z, ci.rx, ci.ry, ci.rz) = struct.unpack_from("!IB6d", buf)
        return ci

class KinematicsInfo(object):
    @staticmethod
    def unpack(buf):
        return KinematicsInfo()

class JointLimitData(object):
    __slots__ = ['min_limit', 'max_limit', 'max_speed', 'max_acceleration']

class ConfigurationData(object):
    __slots__ = ['joint_limit_data',
                 'v_joint_default', 'a_joint_default', 
                 'v_tool_default', 'a_tool_default', 'eq_radius',
                 'dh_a', 'dh_d', 'dh_alpha', 'dh_theta',
                 'masterboard_version', 'controller_box_type',
                 'robot_type', 'robot_subtype']
    @staticmethod
    def unpack(buf):
        cd = ConfigurationData()
        cd.joint_limit_data = []
        for i in range(6):
            jld = JointLimitData()
            (jld.min_limit, jld.max_limit) = struct.unpack_from("!dd", buf, 5+16*i)
            (jld.max_speed, jld.max_acceleration) = struct.unpack_from("!dd", buf, 5+16*6+16*i)
            cd.joint_limit_data.append(jld)
        (cd.v_joint_default, cd.a_joint_default, cd.v_tool_default, cd.a_tool_default,
         cd.eq_radius) = struct.unpack_from("!ddddd", buf, 5+32*6)
        (cd.masterboard_version, cd.controller_box_type, cd.robot_type,
         cd.robot_subtype) = struct.unpack_from("!iiii", buf, 5+32*6+5*8+6*32)
        return cd

class ForceModeData(object):
    __slots__ = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'robot_dexterity']
    @staticmethod
    def unpack(buf):
        fmd = ForceModeData()
        (_, _, fmd.x, fmd.y, fmd.z, fmd.rx, fmd.ry, fmd.rz,
         fmd.robot_dexterity) = struct.unpack_from("!IBddddddd", buf)
        return fmd

class AdditionalInfo(object):
    __slots__ = ['ctrl_bits', 'teach_button']
    @staticmethod
    def unpack(buf):
        ai = AdditionalInfo()
        (_, _, ai.ctrl_bits, ai.teach_button) = struct.unpack_from("!IBIB", buf)
        return ai

class RobotState(object):
    __slots__ = ['robot_mode_data', 'joint_data', 'tool_data',
                 'masterboard_data', 'cartesian_info',
                 'kinematics_info', 'configuration_data',
                 'force_mode_data', 'additional_info']

    def __init__(self):
        pass

    @staticmethod
    def unpack(buf):
        length, mtype = struct.unpack_from("!IB", buf)
        if length != len(buf):
            raise Exception("Could not unpack packet: length field is incorrect")
        if mtype != 16:
            if mtype == 20:
                print "Likely a syntax error:"
                print buf[:2048]
            raise Exception("Fatal error when unpacking RobotState packet")

        rs = RobotState()
        offset = 5
        while offset < len(buf):
            length, ptype = struct.unpack_from("!IB", buf, offset)
            assert offset + length <= len(buf)
            package_buf = buffer(buf, offset, length)
            offset += length

            if ptype == PackageType.ROBOT_MODE_DATA:
                rs.robot_mode_data = RobotModeData.unpack(package_buf)
            elif ptype == PackageType.JOINT_DATA:
                rs.joint_data = JointData.unpack(package_buf)
            elif ptype == PackageType.TOOL_DATA:
                rs.tool_data = ToolData.unpack(package_buf)
            elif ptype == PackageType.MASTERBOARD_DATA:
                rs.masterboard_data = MasterboardData.unpack(package_buf)
            elif ptype == PackageType.CARTESIAN_INFO:
                rs.cartesian_info = CartesianInfo.unpack(package_buf)
            elif ptype == PackageType.KINEMATICS_INFO:
                rs.kinematics_info = KinematicsInfo.unpack(package_buf)
            elif ptype == PackageType.CONFIGURATION_DATA:
                rs.configuration_data = ConfigurationData.unpack(package_buf)
            elif ptype == PackageType.FORCE_MODE_DATA:
                rs.force_mode_data = ForceModeData.unpack(package_buf)
            elif ptype == PackageType.ADDITIONAL_INFO:
                rs.additional_info = AdditionalInfo.unpack(package_buf)
            else:
                raise Exception("Unknown package type: %i" % ptype)
        return rs

def pstate(o, indent=''):
    for s in o.__slots__:
        child = getattr(o, s, None)
        if child is None:
            print "%s%s: None" % (indent, s)
        elif hasattr(child, '__slots__'):
            print "%s%s:" % (indent, s)
            pstate(child, indent + '    ')
        elif hasattr(child, '__iter__'):
            print "%s%s:" % (indent, s)
            for i, c in enumerate(child):
                print "%s  [%i]:" % (indent, i)
                pstate(c, indent + '    ')
        else:
            print "%s%s: %s" % (indent, s, child)
