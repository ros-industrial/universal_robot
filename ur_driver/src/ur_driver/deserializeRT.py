import struct
import copy

#this class handles different protocol versions
class RobotStateRT(object):
    @staticmethod
    def unpack(buf):
        rs = RobotStateRT()
        (plen, ptype) = struct.unpack_from("!IB", buf)
        if plen == 756:
            return RobotStateRT_V15.unpack(buf)        
        elif plen == 812:
            return RobotStateRT_V18.unpack(buf)
        elif plen == 1044:
            return RobotStateRT_V30.unpack(buf)
        else:
            print "RobotStateRT has wrong length: " + str(plen)
            return rs

#this parses RobotStateRT for versions = v1.5
#http://wiki03.lynero.net/Technical/RealTimeClientInterface?foswiki_redirect_cache=9b4574b30760f720c6f79c5f1f2203dd
class RobotStateRT_V15(object):
    __slots__ = ['time', 
                 'q_target', 'qd_target', 'qdd_target', 'i_target', 'm_target', 
                 'q_actual', 'qd_actual', 'i_actual', 'tool_acc_values', 
                 'unused', 
                 'tcp_force', 'tool_vector', 'tcp_speed', 
                 'digital_input_bits', 'motor_temperatures', 'controller_timer', 
                 'test_value']

    @staticmethod
    def unpack(buf):
        offset = 0
        message_size = struct.unpack_from("!i", buf, offset)[0]
        offset+=4
        if message_size != len(buf):
            print("MessageSize: ", message_size, "; BufferSize: ", len(buf))
            raise Exception("Could not unpack RobotStateRT packet: length field is incorrect")

        rs = RobotStateRT_V15()
        #time: 1x double (1x 8byte)
        rs.time = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #q_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.q_target = copy.deepcopy(all_values)
        
        #qd_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qd_target = copy.deepcopy(all_values)
        
        #qdd_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qdd_target = copy.deepcopy(all_values)
        
        #i_target: 6x double (6x 8byte) 
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_target = copy.deepcopy(all_values)
        
        #m_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.m_target = copy.deepcopy(all_values)
        
        #q_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.q_actual = copy.deepcopy(all_values)
        
        #qd_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qd_actual = copy.deepcopy(all_values)
        
        #i_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_actual = copy.deepcopy(all_values)

        ###
        
        #tool_acc_values: 3x double (3x 8byte)
        all_values = list(struct.unpack_from("!ddd",buf, offset))
        offset+=3*8
        rs.tool_acc_values = copy.deepcopy(all_values)
        
        #unused: 15x double (15x 8byte)
        offset+=120
        rs.unused = []
        
        #tcp_force: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_force = copy.deepcopy(all_values)
        
        #tool_vector: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tool_vector = copy.deepcopy(all_values)
        
        #tcp_speed: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_speed = copy.deepcopy(all_values)
        
        #digital_input_bits: 1x double (1x 8byte) ?
        rs.digital_input_bits = struct.unpack_from("!d",buf, offset)[0]
        offset+=8

        #motor_temperatures: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.motor_temperatures = copy.deepcopy(all_values)
        
        #controller_timer: 1x double (1x 8byte)
        rs.controller_timer = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #test_value: 1x double (1x 8byte)
        rs.test_value = struct.unpack_from("!d",buf, offset)[0]
        offset+=8

        return rs  


#this parses RobotStateRT for versions <= v1.8 (i.e. 1.6, 1.7, 1.8)
class RobotStateRT_V18(object):
    __slots__ = ['time', 
                 'q_target', 'qd_target', 'qdd_target', 'i_target', 'm_target', 
                 'q_actual', 'qd_actual', 'i_actual', 'tool_acc_values', 
                 'unused', 
                 'tcp_force', 'tool_vector', 'tcp_speed', 
                 'digital_input_bits', 'motor_temperatures', 'controller_timer', 
                 'test_value', 
                 'robot_mode', 'joint_modes']

    @staticmethod
    def unpack(buf):
        offset = 0
        message_size = struct.unpack_from("!i", buf, offset)[0]
        offset+=4
        if message_size != len(buf):
            print("MessageSize: ", message_size, "; BufferSize: ", len(buf))
            raise Exception("Could not unpack RobotStateRT packet: length field is incorrect")

        rs = RobotStateRT_V18()
        #time: 1x double (1x 8byte)
        rs.time = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #q_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.q_target = copy.deepcopy(all_values)
        
        #qd_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qd_target = copy.deepcopy(all_values)
        
        #qdd_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qdd_target = copy.deepcopy(all_values)
        
        #i_target: 6x double (6x 8byte) 
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_target = copy.deepcopy(all_values)
        
        #m_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.m_target = copy.deepcopy(all_values)
        
        #q_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.q_actual = copy.deepcopy(all_values)
        
        #qd_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qd_actual = copy.deepcopy(all_values)
        
        #i_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_actual = copy.deepcopy(all_values)
        
        #tool_acc_values: 3x double (3x 8byte)
        all_values = list(struct.unpack_from("!ddd",buf, offset))
        offset+=3*8
        rs.tool_acc_values = copy.deepcopy(all_values)
        
        #unused: 15x double (15x 8byte)
        offset+=120
        rs.unused = []
        
        #tcp_force: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_force = copy.deepcopy(all_values)
        
        #tool_vector: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tool_vector = copy.deepcopy(all_values)
        
        #tcp_speed: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_speed = copy.deepcopy(all_values)
        
        #digital_input_bits: 1x double (1x 8byte) ?
        rs.digital_input_bits = struct.unpack_from("!d",buf, offset)[0]
        offset+=8

        #motor_temperatures: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.motor_temperatures = copy.deepcopy(all_values)
        
        #controller_timer: 1x double (1x 8byte)
        rs.controller_timer = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #test_value: 1x double (1x 8byte)
        rs.test_value = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #robot_mode: 1x double (1x 8byte)
        rs.robot_mode = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #joint_mode: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.joint_modes = copy.deepcopy(all_values)

        return rs

#this parses RobotStateRT for versions >=3.0 (i.e. 3.0)
class RobotStateRT_V30(object):
    __slots__ = ['time', 
                 'q_target', 'qd_target', 'qdd_target', 'i_target', 'm_target', 
                 'q_actual', 'qd_actual', 'i_actual', 'i_control', 
                 'tool_vector_actual', 'tcp_speed_actual', 'tcp_force', 
                 'tool_vector_target', 'tcp_speed_target', 
                 'digital_input_bits', 'motor_temperatures', 'controller_timer', 
                 'test_value', 
                 'robot_mode', 'joint_modes', 'safety_mode', 
                 #6xd: unused
                 'tool_acc_values', 
                 #6xd: unused
                 'speed_scaling', 'linear_momentum_norm', 
                 #2xd: unused
                 'v_main', 'v_robot', 'i_robot', 'v_actual']

    @staticmethod
    def unpack(buf):
        offset = 0
        message_size = struct.unpack_from("!i", buf, offset)[0]
        offset+=4
        if message_size != len(buf):
            print("MessageSize: ", message_size, "; BufferSize: ", len(buf))
            raise Exception("Could not unpack RobotStateRT packet: length field is incorrect")

        rs = RobotStateRT_V30()
        #time: 1x double (1x 8byte)
        rs.time = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #q_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.q_target = copy.deepcopy(all_values)
        
        #qd_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qd_target = copy.deepcopy(all_values)
        
        #qdd_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qdd_target = copy.deepcopy(all_values)
        
        #i_target: 6x double (6x 8byte) 
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_target = copy.deepcopy(all_values)
        
        #m_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.m_target = copy.deepcopy(all_values)
        
        #q_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.q_actual = copy.deepcopy(all_values)
        
        #qd_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.qd_actual = copy.deepcopy(all_values)
        
        #i_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_actual = copy.deepcopy(all_values)
        
        #i_control: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.i_control = copy.deepcopy(all_values)
        
        #tool_vector_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tool_vector_actual = copy.deepcopy(all_values)

        #tcp_speed_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_speed_actual = copy.deepcopy(all_values)
        
        #tcp_force: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_force = copy.deepcopy(all_values)
        
        #tool_vector_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tool_vector_target = copy.deepcopy(all_values)
        
        #tcp_speed_target: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.tcp_speed_target = copy.deepcopy(all_values)
        
        #digital_input_bits: 1x double (1x 8byte) ?
        rs.digital_input_bits = struct.unpack_from("!d",buf, offset)[0]
        offset+=8

        #motor_temperatures: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.motor_temperatures = copy.deepcopy(all_values)
        
        #controller_timer: 1x double (1x 8byte)
        rs.controller_timer = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #test_value: 1x double (1x 8byte)
        rs.test_value = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #robot_mode: 1x double (1x 8byte)
        rs.robot_mode = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #joint_modes: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.joint_modes = copy.deepcopy(all_values)
        
        #safety_mode: 1x double (1x 8byte)
        rs.safety_mode = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #unused: 6x double (6x 8byte)
        offset+=48
        
        #tool_acc_values: 3x double (3x 8byte)
        all_values = list(struct.unpack_from("!ddd",buf, offset))
        offset+=3*8
        rs.tool_acc_values = copy.deepcopy(all_values)
        
        #unused: 6x double (6x 8byte)
        offset+=48
        
        #speed_scaling: 1x double (1x 8byte)
        rs.speed_scaling = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #linear_momentum_norm: 1x double (1x 8byte)
        rs.linear_momentum_norm = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #unused: 2x double (2x 8byte)
        offset+=16
        
        #v_main: 1x double (1x 8byte)
        rs.v_main = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #v_robot: 1x double (1x 8byte)
        rs.v_robot = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #i_robot: 1x double (1x 8byte)
        rs.i_robot = struct.unpack_from("!d",buf, offset)[0]
        offset+=8
        
        #v_actual: 6x double (6x 8byte)
        all_values = list(struct.unpack_from("!dddddd",buf, offset))
        offset+=6*8
        rs.v_actual = copy.deepcopy(all_values)
        
        return rs
