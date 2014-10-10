import struct
import copy

class RobotStateRT(object):
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

        rs = RobotStateRT()
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
