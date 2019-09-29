#!/usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('ur_driver')
import time, sys, threading, math
import copy
import datetime
import socket, select
import struct
import traceback, code
import optparse
import SocketServer

import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped

from dynamic_reconfigure.server import Server
from ur_driver.cfg import URDriverConfig

from ur_driver.deserialize import RobotState, RobotMode
from ur_driver.deserializeRT import RobotStateRT

from ur_msgs.srv import SetPayload, SetIO
from ur_msgs.msg import *

# renaming classes
DigitalIn = Digital
DigitalOut = Digital
Flag  = Digital

prevent_programming = False

# Joint offsets, pulled from calibration information stored in the URDF
#
# { "joint_name" : offset }
#
# q_actual = q_from_driver + offset
joint_offsets = {}

PORT=30002       # 10 Hz, RobotState 
RT_PORT=30003    #125 Hz, RobotStateRT
DEFAULT_REVERSE_PORT = 50001     #125 Hz, custom data (from prog)

MSG_OUT = 1
MSG_QUIT = 2
MSG_JOINT_STATES = 3
MSG_MOVEJ = 4
MSG_WAYPOINT_FINISHED = 5
MSG_STOPJ = 6
MSG_SERVOJ = 7
MSG_SET_PAYLOAD = 8
MSG_WRENCH = 9
MSG_SET_DIGITAL_OUT = 10
MSG_GET_IO = 11
MSG_SET_FLAG = 12
MSG_SET_TOOL_VOLTAGE = 13
MSG_SET_ANALOG_OUT = 14
MULT_payload = 1000.0
MULT_wrench = 10000.0
MULT_jointstate = 10000.0
MULT_time = 1000000.0
MULT_blend = 1000.0
MULT_analog = 1000000.0
MULT_analog_robotstate = 0.1

#Max Velocity accepted by ur_driver
MAX_VELOCITY = 10.0
#Using a very high value in order to not limit execution of trajectories being sent from MoveIt!

#Bounds for SetPayload service
MIN_PAYLOAD = 0.0
MAX_PAYLOAD = 1.0
#Using a very conservative value as it should be set throught the parameter server


IO_SLEEP_TIME = 0.05

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]
  

connected_robot = None
connected_robot_lock = threading.Lock()
connected_robot_cond = threading.Condition(connected_robot_lock)
last_joint_states = None
last_joint_states_lock = threading.Lock()
pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=1)
pub_wrench = rospy.Publisher('wrench', WrenchStamped, queue_size=1)
pub_io_states = rospy.Publisher('io_states', IOStates, queue_size=1)
#dump_state = open('dump_state', 'wb')

class EOF(Exception): pass

def dumpstacks():
    id2name = dict([(th.ident, th.name) for th in threading.enumerate()])
    code = []
    for threadId, stack in sys._current_frames().items():
        code.append("\n# Thread: %s(%d)" % (id2name.get(threadId,""), threadId))
        for filename, lineno, name, line in traceback.extract_stack(stack):
            code.append('File: "%s", line %d, in %s' % (filename, lineno, name))
            if line:
                code.append("  %s" % (line.strip()))
    print("\n".join(code))

def log(s):
    print("[%s] %s" % (datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'), s))


RESET_PROGRAM = '''def resetProg():
  sleep(0.0)
end
'''
#RESET_PROGRAM = ''
    
class URConnection(object):
    TIMEOUT = 1.0
    
    DISCONNECTED = 0
    CONNECTED = 1
    READY_TO_PROGRAM = 2
    EXECUTING = 3
    
    def __init__(self, hostname, port, program):
        self.__thread = None
        self.__sock = None
        self.robot_state = self.DISCONNECTED
        self.hostname = hostname
        self.port = port
        self.program = program
        self.last_state = None

    def connect(self):
        if self.__sock:
            self.disconnect()
        self.__buf = ""
        self.robot_state = self.CONNECTED
        self.__sock = socket.create_connection((self.hostname, self.port))
        self.__keep_running = True
        self.__thread = threading.Thread(name="URConnection", target=self.__run)
        self.__thread.daemon = True
        self.__thread.start()

    def send_program(self):
        global prevent_programming
        if prevent_programming:
            rospy.loginfo("Programming is currently prevented")
            return
        assert self.robot_state in [self.READY_TO_PROGRAM, self.EXECUTING]
        rospy.loginfo("Programming the robot at %s" % self.hostname)
        self.__sock.sendall(self.program)
        self.robot_state = self.EXECUTING

    def send_reset_program(self):
        self.__sock.sendall(RESET_PROGRAM)
        self.robot_state = self.READY_TO_PROGRAM
        
    def disconnect(self):
        if self.__thread:
            self.__keep_running = False
            self.__thread.join()
            self.__thread = None
        if self.__sock:
            self.__sock.close()
            self.__sock = None
        self.last_state = None
        self.robot_state = self.DISCONNECTED

    def ready_to_program(self):
        return self.robot_state in [self.READY_TO_PROGRAM, self.EXECUTING]

    def __trigger_disconnected(self):
        log("Robot disconnected")
        self.robot_state = self.DISCONNECTED
    def __trigger_ready_to_program(self):
        rospy.loginfo("Robot ready to program")
    def __trigger_halted(self):
        log("Halted")

    def __on_packet(self, buf):
        state = RobotState.unpack(buf)
        self.last_state = state
        #import deserialize; deserialize.pstate(self.last_state)

        #log("Packet.  Mode=%s" % state.robot_mode_data.robot_mode)

        if not state.robot_mode_data.real_robot_enabled:
            rospy.logfatal("Real robot is no longer enabled.  Driver is fuxored")
            time.sleep(2)
            sys.exit(1)
        
        ###
        # IO-Support is EXPERIMENTAL
        # 
        # Notes: 
        # - Where are the flags coming from? Do we need flags? No, as 'prog' does not use them and other scripts are not running!
        # - analog_input2 and analog_input3 are within ToolData
        # - What to do with the different analog_input/output_range/domain?
        # - Shall we have appropriate ur_msgs definitions in order to reflect MasterboardData, ToolData,...?
        ###
        
        # Use information from the robot state packet to publish IOStates        
        msg = IOStates()
        #gets digital in states
        for i in range(0, 10):
            msg.digital_in_states.append(DigitalIn(i, (state.masterboard_data.digital_input_bits & (1<<i))>>i))
        #gets digital out states
        for i in range(0, 10):
            msg.digital_out_states.append(DigitalOut(i, (state.masterboard_data.digital_output_bits & (1<<i))>>i))
        #gets analog_in[0] state
        inp = state.masterboard_data.analog_input0 / MULT_analog_robotstate
        msg.analog_in_states.append(Analog(0, inp))
        #gets analog_in[1] state
        inp = state.masterboard_data.analog_input1 / MULT_analog_robotstate
        msg.analog_in_states.append(Analog(1, inp))      
        #gets analog_out[0] state
        inp = state.masterboard_data.analog_output0 / MULT_analog_robotstate
        msg.analog_out_states.append(Analog(0, inp))     
        #gets analog_out[1] state
        inp = state.masterboard_data.analog_output1 / MULT_analog_robotstate
        msg.analog_out_states.append(Analog(1, inp))     
        #print "Publish IO-Data from robot state data"
        pub_io_states.publish(msg)
        

        # Updates the state machine that determines whether we can program the robot.
        can_execute = (state.robot_mode_data.robot_mode in [RobotMode.READY, RobotMode.RUNNING])
        if self.robot_state == self.CONNECTED:
            if can_execute:
                self.__trigger_ready_to_program()
                self.robot_state = self.READY_TO_PROGRAM
        elif self.robot_state == self.READY_TO_PROGRAM:
            if not can_execute:
                self.robot_state = self.CONNECTED
        elif self.robot_state == self.EXECUTING:
            if not can_execute:
                self.__trigger_halted()
                self.robot_state = self.CONNECTED

        # Report on any unknown packet types that were received
        if len(state.unknown_ptypes) > 0:
            state.unknown_ptypes.sort()
            s_unknown_ptypes = [str(ptype) for ptype in state.unknown_ptypes]
            self.throttle_warn_unknown(1.0, "Ignoring unknown pkt type(s): %s. "
                          "Please report." % ", ".join(s_unknown_ptypes))

    def throttle_warn_unknown(self, period, msg):
        self.__dict__.setdefault('_last_hit', 0.0)
        # this only works for a single caller
        if (self._last_hit + period) <= rospy.get_time():
            self._last_hit = rospy.get_time()
            rospy.logwarn(msg)

    def __run(self):
        while self.__keep_running:
            r, _, _ = select.select([self.__sock], [], [], self.TIMEOUT)
            if r:
                more = self.__sock.recv(4096)
                if more:
                    self.__buf = self.__buf + more

                    #unpack_from requires a buffer of at least 48 bytes
                    while len(self.__buf) >= 48:
                        # Attempts to extract a packet
                        packet_length, ptype = struct.unpack_from("!IB", self.__buf)
                        #print("PacketLength: ", packet_length, "; BufferSize: ", len(self.__buf))
                        if len(self.__buf) >= packet_length:
                            packet, self.__buf = self.__buf[:packet_length], self.__buf[packet_length:]
                            self.__on_packet(packet)
                        else:
                            break

                else:
                    self.__trigger_disconnected()
                    self.__keep_running = False
                    
            else:
                self.__trigger_disconnected()
                self.__keep_running = False


class URConnectionRT(object):
    TIMEOUT = 1.0
    
    DISCONNECTED = 0
    CONNECTED = 1
    
    def __init__(self, hostname, port):
        self.__thread = None
        self.__sock = None
        self.robot_state = self.DISCONNECTED
        self.hostname = hostname
        self.port = port
        self.last_stateRT = None

    def connect(self):
        if self.__sock:
            self.disconnect()
        self.__buf = ""
        self.robot_state = self.CONNECTED
        self.__sock = socket.create_connection((self.hostname, self.port))
        self.__keep_running = True
        self.__thread = threading.Thread(name="URConnectionRT", target=self.__run)
        self.__thread.daemon = True
        self.__thread.start()
        
    def disconnect(self):
        if self.__thread:
            self.__keep_running = False
            self.__thread.join()
            self.__thread = None
        if self.__sock:
            self.__sock.close()
            self.__sock = None
        self.last_state = None
        self.robot_state = self.DISCONNECTED

    def __trigger_disconnected(self):
        log("Robot disconnected")
        self.robot_state = self.DISCONNECTED

    def __on_packet(self, buf):
        global last_joint_states, last_joint_states_lock
        now = rospy.get_rostime()
        stateRT = RobotStateRT.unpack(buf)
        self.last_stateRT = stateRT
        
        msg = JointState()
        msg.header.stamp = now
        msg.header.frame_id = "From real-time state data"
        msg.name = joint_names
        msg.position = [0.0] * 6
        for i, q in enumerate(stateRT.q_actual):
            msg.position[i] = q + joint_offsets.get(joint_names[i], 0.0)
        msg.velocity = stateRT.qd_actual
        msg.effort = [0]*6
        pub_joint_states.publish(msg)
        with last_joint_states_lock:
            last_joint_states = msg
        
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = now
        wrench_msg.wrench.force.x = stateRT.tcp_force[0]
        wrench_msg.wrench.force.y = stateRT.tcp_force[1]
        wrench_msg.wrench.force.z = stateRT.tcp_force[2]
        wrench_msg.wrench.torque.x = stateRT.tcp_force[3]
        wrench_msg.wrench.torque.y = stateRT.tcp_force[4]
        wrench_msg.wrench.torque.z = stateRT.tcp_force[5]
        pub_wrench.publish(wrench_msg)
        

    def __run(self):
        while self.__keep_running:
            r, _, _ = select.select([self.__sock], [], [], self.TIMEOUT)
            if r:
                more = self.__sock.recv(4096)
                if more:
                    self.__buf = self.__buf + more
                    
                    #unpack_from requires a buffer of at least 48 bytes
                    while len(self.__buf) >= 48:
                        # Attempts to extract a packet
                        packet_length = struct.unpack_from("!i", self.__buf)[0]
                        #print("PacketLength: ", packet_length, "; BufferSize: ", len(self.__buf))
                        if len(self.__buf) >= packet_length:
                            packet, self.__buf = self.__buf[:packet_length], self.__buf[packet_length:]
                            self.__on_packet(packet)
                        else:
                            break
                else:
                    self.__trigger_disconnected()
                    self.__keep_running = False
                    
            else:
                self.__trigger_disconnected()
                self.__keep_running = False


def setConnectedRobot(r):
    global connected_robot, connected_robot_lock
    with connected_robot_lock:
        connected_robot = r
        connected_robot_cond.notify()

def getConnectedRobot(wait=False, timeout=-1):
    started = time.time()
    with connected_robot_lock:
        if wait:
            while not connected_robot:
                if timeout >= 0 and time.time() > started + timeout:
                    break
                connected_robot_cond.wait(0.2)
        return connected_robot

# Receives messages from the robot over the socket
class CommanderTCPHandler(SocketServer.BaseRequestHandler):

    def recv_more(self):
        global last_joint_states, last_joint_states_lock
        while True:
            r, _, _ = select.select([self.request], [], [], 0.2)
            if r:
                more = self.request.recv(4096)
                if not more:
                    raise EOF("EOF on recv")
                return more
            else:
                now = rospy.get_rostime()
                if last_joint_states and \
                        last_joint_states.header.stamp < now - rospy.Duration(1.0):
                    rospy.logerr("Stopped hearing from robot (last heard %.3f sec ago).  Disconnected" % \
                                     (now - last_joint_states.header.stamp).to_sec())
                    raise EOF()

    def handle(self):
        self.__socket_lock = threading.Lock()
        setConnectedRobot(self)
        print("Handling a request")
        try:
            buf = self.recv_more()
            if not buf: return

            while True:
                #print "Buf:", [ord(b) for b in buf]

                # Unpacks the message type
                mtype = struct.unpack_from("!i", buf, 0)[0]
                buf = buf[4:]
                #print "Message type:", mtype

                if mtype == MSG_OUT:
                    # Unpacks string message, terminated by tilde
                    i = buf.find("~")
                    while i < 0:
                        buf = buf + self.recv_more()
                        i = buf.find("~")
                        if len(buf) > 2000:
                            raise Exception("Probably forgot to terminate a string: %s..." % buf[:150])
                    s, buf = buf[:i], buf[i+1:]
                    log("Out: %s" % s)
                
                elif mtype == MSG_QUIT:
                    print("Quitting")
                    raise EOF("Received quit")
                elif mtype == MSG_WAYPOINT_FINISHED:
                    while len(buf) < 4:
                        buf = buf + self.recv_more()
                    waypoint_id = struct.unpack_from("!i", buf, 0)[0]
                    buf = buf[4:]
                    print("Waypoint finished (not handled)")
                else:
                    raise Exception("Unknown message type: %i" % mtype)

                if not buf:
                    buf = buf + self.recv_more()
        except EOF as ex:
            print("Connection closed (command):", ex)
            setConnectedRobot(None)

    def __send_message(self, data):
        """
        Send a message to the robot.

        The message is given as a list of integers that will be packed
        as 4 bytes each in network byte order (big endian).

        A lock is acquired before sending the message to prevent race conditions.

        :param data: list of int, the data to send
        """
        buf = struct.pack("!%ii" % len(data), *data)
        with self.__socket_lock:
            self.request.send(buf)

    def send_quit(self):
        self.__send_message([MSG_QUIT])

    def send_servoj(self, waypoint_id, q_actual, t):
        assert(len(q_actual) == 6)
        q_robot = [0.0] * 6
        for i, q in enumerate(q_actual):
            q_robot[i] = q - joint_offsets.get(joint_names[i], 0.0)
        params = [MSG_SERVOJ, waypoint_id] + \
                 [MULT_jointstate * qq for qq in q_robot] + \
                 [MULT_time * t]
        self.__send_message(params)

    #Experimental set_payload implementation
    def send_payload(self,payload):
        self.__send_message([MSG_SET_PAYLOAD, payload * MULT_payload])

    #Experimental set_digital_output implementation
    def set_digital_out(self, pinnum, value):
        self.__send_message([MSG_SET_DIGITAL_OUT, pinnum, value])
        time.sleep(IO_SLEEP_TIME)

    def set_analog_out(self, pinnum, value):
        self.__send_message([MSG_SET_ANALOG_OUT, pinnum, value * MULT_analog])
        time.sleep(IO_SLEEP_TIME)

    def set_tool_voltage(self, value):
        self.__send_message([MSG_SET_TOOL_VOLTAGE, value, 0])
        time.sleep(IO_SLEEP_TIME)

    def set_flag(self, pin, val):
        self.__send_message([MSG_SET_FLAG, pin, val])
        #set_flag will fail if called too closely together--added delay
        time.sleep(IO_SLEEP_TIME)

    def send_stopj(self):
        self.__send_message([MSG_STOPJ])

    def set_waypoint_finished_cb(self, cb):
        self.waypoint_finished_cb = cb

    # Returns the last JointState message sent out
    def get_joint_states(self):
        global last_joint_states, last_joint_states_lock
        return last_joint_states
    

class TCPServer(SocketServer.TCPServer):
    allow_reuse_address = True  # Allows the program to restart gracefully on crash
    timeout = 5


# Waits until all threads have completed.  Allows KeyboardInterrupt to occur
def joinAll(threads):
    while any(t.isAlive() for t in threads):
        for t in threads:
            t.join(0.2)

# Returns the duration between moving from point (index-1) to point
# index in the given JointTrajectory
def get_segment_duration(traj, index):
    if index == 0:
        return traj.points[0].time_from_start.to_sec()
    return (traj.points[index].time_from_start - traj.points[index-1].time_from_start).to_sec()

# Reorders the JointTrajectory traj according to the order in
# joint_names.  Destructive.
def reorder_traj_joints(traj, joint_names):
    order = [traj.joint_names.index(j) for j in joint_names]

    new_points = []
    for p in traj.points:
        new_points.append(JointTrajectoryPoint(
            positions = [p.positions[i] for i in order],
            velocities = [p.velocities[i] for i in order] if p.velocities else [],
            accelerations = [p.accelerations[i] for i in order] if p.accelerations else [],
            time_from_start = p.time_from_start))
    traj.joint_names = joint_names
    traj.points = new_points

def interp_cubic(p0, p1, t_abs):
    T = (p1.time_from_start - p0.time_from_start).to_sec()
    t = t_abs - p0.time_from_start.to_sec()
    q = [0] * 6
    qdot = [0] * 6
    qddot = [0] * 6
    for i in range(len(p0.positions)):
        a = p0.positions[i]
        b = p0.velocities[i]
        c = (-3*p0.positions[i] + 3*p1.positions[i] - 2*T*p0.velocities[i] - T*p1.velocities[i]) / T**2
        d = (2*p0.positions[i] - 2*p1.positions[i] + T*p0.velocities[i] + T*p1.velocities[i]) / T**3

        q[i] = a + b*t + c*t**2 + d*t**3
        qdot[i] = b + 2*c*t + 3*d*t**2
        qddot[i] = 2*c + 6*d*t
    return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=rospy.Duration(t_abs))

# Returns (q, qdot, qddot) for sampling the JointTrajectory at time t.
# The time t is the time since the trajectory was started.
def sample_traj(traj, t):
    # First point
    if t <= 0.0:
        return copy.deepcopy(traj.points[0])
    # Last point
    if t >= traj.points[-1].time_from_start.to_sec():
        return copy.deepcopy(traj.points[-1])
    
    # Finds the (middle) segment containing t
    i = 0
    while traj.points[i+1].time_from_start.to_sec() < t:
        i += 1
    return interp_cubic(traj.points[i], traj.points[i+1], t)

def traj_is_finite(traj):
    for pt in traj.points:
        for p in pt.positions:
            if math.isinf(p) or math.isnan(p):
                return False
        for v in pt.velocities:
            if math.isinf(v) or math.isnan(v):
                return False
    return True
        
def has_limited_velocities(traj):
    for p in traj.points:
        for v in p.velocities:
            if math.fabs(v) > max_velocity:
                return False
    return True

def has_velocities(traj):
    for p in traj.points:
        if len(p.velocities) != len(p.positions):
            return False
    return True

def within_tolerance(a_vec, b_vec, tol_vec):
    for a, b, tol in zip(a_vec, b_vec, tol_vec):
        if abs(a - b) > tol:
            return False
    return True

class URServiceProvider(object):
    def __init__(self, robot):
        self.robot = robot
        rospy.Service('ur_driver/setPayload', SetPayload, self.setPayload)

    def set_robot(self, robot):
        self.robot = robot

    def setPayload(self, req):
        if req.payload < min_payload or req.payload > max_payload:
            print('ERROR: Payload ' + str(req.payload) + ' out of bounds (' + str(min_payload) + ', ' + str(max_payload) + ')')
            return False
        
        if self.robot:
            self.robot.send_payload(req.payload)
        else:
            return False
        return True

class URTrajectoryFollower(object):
    RATE = 0.02
    def __init__(self, robot, goal_time_tolerance=None):
        self.goal_time_tolerance = goal_time_tolerance or rospy.Duration(0.0)
        self.joint_goal_tolerances = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
        self.following_lock = threading.Lock()
        self.T0 = time.time()
        self.robot = robot
        self.server = actionlib.ActionServer("follow_joint_trajectory",
                                             FollowJointTrajectoryAction,
                                             self.on_goal, self.on_cancel, auto_start=False)

        self.goal_handle = None
        self.traj = None
        self.traj_t0 = 0.0
        self.first_waypoint_id = 10
        self.tracking_i = 0
        self.pending_i = 0
        self.last_point_sent = True

        self.update_timer = rospy.Timer(rospy.Duration(self.RATE), self._update)

    def set_robot(self, robot):
        # Cancels any goals in progress
        if self.goal_handle:
            self.goal_handle.set_canceled()
            self.goal_handle = None
        self.traj = None
        self.robot = robot
        if self.robot:
            self.init_traj_from_robot()

    # Sets the trajectory to remain stationary at the current position
    # of the robot.
    def init_traj_from_robot(self):
        if not self.robot: raise Exception("No robot connected")
        # Busy wait (avoids another mutex)
        state = self.robot.get_joint_states()
        while not state:
            time.sleep(0.1)
            state = self.robot.get_joint_states()
        self.traj_t0 = time.time()
        self.traj = JointTrajectory()
        self.traj.joint_names = joint_names
        self.traj.points = [JointTrajectoryPoint(
            positions = state.position,
            velocities = [0] * 6,
            accelerations = [0] * 6,
            time_from_start = rospy.Duration(0.0))]

    def start(self):
        self.init_traj_from_robot()
        self.server.start()
        print("The action server for this driver has been started")

    def on_goal(self, goal_handle):
        log("on_goal")

        # Checks that the robot is connected
        if not self.robot:
            rospy.logerr("Received a goal, but the robot is not connected")
            goal_handle.set_rejected()
            return

        # Checks if the joints are just incorrect
        if set(goal_handle.get_goal().trajectory.joint_names) != set(joint_names):
            rospy.logerr("Received a goal with incorrect joint names: (%s)" % \
                         ', '.join(goal_handle.get_goal().trajectory.joint_names))
            goal_handle.set_rejected()
            return

        if not traj_is_finite(goal_handle.get_goal().trajectory):
            rospy.logerr("Received a goal with infinites or NaNs")
            goal_handle.set_rejected(text="Received a goal with infinites or NaNs")
            return
        
        # Checks that the trajectory has velocities
        if not has_velocities(goal_handle.get_goal().trajectory):
            rospy.logerr("Received a goal without velocities")
            goal_handle.set_rejected(text="Received a goal without velocities")
            return

        # Checks that the velocities are withing the specified limits
        if not has_limited_velocities(goal_handle.get_goal().trajectory):
            message = "Received a goal with velocities that are higher than %f" % max_velocity
            rospy.logerr(message)
            goal_handle.set_rejected(text=message)
            return

        # Orders the joints of the trajectory according to joint_names
        reorder_traj_joints(goal_handle.get_goal().trajectory, joint_names)
                
        with self.following_lock:
            if self.goal_handle:
                # Cancels the existing goal
                self.goal_handle.set_canceled()
                self.first_waypoint_id += len(self.goal_handle.get_goal().trajectory.points)
                self.goal_handle = None

            # Inserts the current setpoint at the head of the trajectory
            now = time.time()
            point0 = sample_traj(self.traj, now - self.traj_t0)
            point0.time_from_start = rospy.Duration(0.0)
            goal_handle.get_goal().trajectory.points.insert(0, point0)
            self.traj_t0 = now

            # Replaces the goal
            self.goal_handle = goal_handle
            self.traj = goal_handle.get_goal().trajectory
            self.goal_handle.set_accepted()

    def on_cancel(self, goal_handle):
        log("on_cancel")
        if goal_handle == self.goal_handle:
            with self.following_lock:
                # Uses the next little bit of trajectory to slow to a stop
                STOP_DURATION = 0.5
                now = time.time()
                point0 = sample_traj(self.traj, now - self.traj_t0)
                point0.time_from_start = rospy.Duration(0.0)
                point1 = sample_traj(self.traj, now - self.traj_t0 + STOP_DURATION)
                point1.velocities = [0] * 6
                point1.accelerations = [0] * 6
                point1.time_from_start = rospy.Duration(STOP_DURATION)
                self.traj_t0 = now
                self.traj = JointTrajectory()
                self.traj.joint_names = joint_names
                self.traj.points = [point0, point1]
                
                self.goal_handle.set_canceled()
                self.goal_handle = None
        else:
            goal_handle.set_canceled()

    last_now = time.time()
    def _update(self, event):
        if self.robot and self.traj:
            now = time.time()
            if (now - self.traj_t0) <= self.traj.points[-1].time_from_start.to_sec():
                self.last_point_sent = False #sending intermediate points
                setpoint = sample_traj(self.traj, now - self.traj_t0)
                try:
                    self.robot.send_servoj(999, setpoint.positions, 4 * self.RATE)
                except socket.error:
                    pass
                    
            elif not self.last_point_sent:
                # All intermediate points sent, sending last point to make sure we
                # reach the goal.
                # This should solve an issue where the robot does not reach the final
                # position and errors out due to not reaching the goal point.
                last_point = self.traj.points[-1]
                state = self.robot.get_joint_states()
                position_in_tol = within_tolerance(state.position, last_point.positions, self.joint_goal_tolerances)
                # Performing this check to try and catch our error condition.  We will always
                # send the last point just in case.
                if not position_in_tol:
                    rospy.logwarn("Trajectory time exceeded and current robot state not at goal, last point required")
                    rospy.logwarn("Current trajectory time: %s, last point time: %s" % \
                                (now - self.traj_t0, self.traj.points[-1].time_from_start.to_sec()))
                    rospy.logwarn("Desired: %s\nactual: %s\nvelocity: %s" % \
                                          (last_point.positions, state.position, state.velocity))
                setpoint = sample_traj(self.traj, self.traj.points[-1].time_from_start.to_sec())

                try:
                    self.robot.send_servoj(999, setpoint.positions, 4 * self.RATE)
                    self.last_point_sent = True
                except socket.error:
                    pass
                    
            else:  # Off the end
                if self.goal_handle:
                    last_point = self.traj.points[-1]
                    state = self.robot.get_joint_states()
                    position_in_tol = within_tolerance(state.position, last_point.positions, [0.1]*6)
                    velocity_in_tol = within_tolerance(state.velocity, last_point.velocities, [0.05]*6)
                    if position_in_tol and velocity_in_tol:
                        # The arm reached the goal (and isn't moving).  Succeeding
                        self.goal_handle.set_succeeded()
                        self.goal_handle = None
                    #elif now - (self.traj_t0 + last_point.time_from_start.to_sec()) > self.goal_time_tolerance.to_sec():
                    #    # Took too long to reach the goal.  Aborting
                    #    rospy.logwarn("Took too long to reach the goal.\nDesired: %s\nactual: %s\nvelocity: %s" % \
                    #                      (last_point.positions, state.position, state.velocity))
                    #    self.goal_handle.set_aborted(text="Took too long to reach the goal")
                    #    self.goal_handle = None


# joint_names: list of joints
#
# returns: { "joint_name" : joint_offset }
def load_joint_offsets(joint_names):
    from lxml import etree
    robot_description = rospy.get_param("robot_description")
    doc = etree.fromstring(robot_description)

    # select only 'calibration_offset' elements whose parent is a joint
    # element with a specific value for the name attribute
    expr = "/robot/joint[@name=$name]/calibration_offset"
    result = {}
    for joint in joint_names:
        joint_elt = doc.xpath(expr, name=joint)
        if len(joint_elt) == 1:
            calibration_offset = float(joint_elt[0].get("value"))
            result[joint] = calibration_offset
            rospy.loginfo("Found calibration offset for joint \"%s\": %.4f" % (joint, calibration_offset))
        elif len(joint_elt) > 1:
            rospy.logerr("Too many joints matched on \"%s\". Please report to package maintainer(s)." % joint)
        else:
            rospy.logwarn("No calibration offset for joint \"%s\"" % joint)
    return result

def get_my_ip(robot_ip, port):
    s = socket.create_connection((robot_ip, port))
    tmp = s.getsockname()[0]
    s.close()
    return tmp

def handle_set_io(req):
    r = getConnectedRobot(wait=False)
    if r:
        if req.fun == req.FUN_SET_DIGITAL_OUT:
            r.set_digital_out(req.pin, req.state)
            return True
        elif req.fun == req.FUN_SET_FLAG:
            r.set_flag(req.pin, req.state)
            return True
        elif req.fun == req.FUN_SET_ANALOG_OUT:
            r.set_analog_out(req.pin, req.state)
            return True
        elif req.fun == req.FUN_SET_TOOL_VOLTAGE:
            r.set_tool_voltage(req.pin)
            return True
    else:
        raise ROSServiceException("Robot not connected")

def set_io_server():
    s= rospy.Service('set_io', SetIO, handle_set_io)

def reconfigure_callback(config, level):
    global prevent_programming
    prevent_programming = config.prevent_programming
    ## What about updating the value on the parameter server?
    return config

def main():
    rospy.init_node('ur_driver', disable_signals=True)
    if rospy.get_param("use_sim_time", False):
        rospy.logwarn("use_sim_time is set!!!")
    
    global prevent_programming
    reconfigure_srv = Server(URDriverConfig, reconfigure_callback)
    
    prefix = rospy.get_param("~prefix", "")
    print("Setting prefix to %s" % prefix)
    global joint_names
    joint_names = [prefix + name for name in JOINT_NAMES]

    # Parses command line arguments
    parser = optparse.OptionParser(usage="usage: %prog robot_hostname [reverse_port]")
    (options, args) = parser.parse_args(rospy.myargv()[1:])
    if len(args) < 1:
        parser.error("You must specify the robot hostname")
    elif len(args) == 1:
        robot_hostname = args[0]
        reverse_port = DEFAULT_REVERSE_PORT
    elif len(args) == 2:
        robot_hostname = args[0]
        reverse_port = int(args[1])
        if not (0 <= reverse_port <= 65535):
                parser.error("You entered an invalid port number")
    else:
        parser.error("Wrong number of parameters")

    # Reads the calibrated joint offsets from the URDF
    global joint_offsets
    joint_offsets = load_joint_offsets(joint_names)
    if len(joint_offsets) > 0:
        rospy.loginfo("Loaded calibration offsets from urdf: %s" % joint_offsets)
    else:
        rospy.loginfo("No calibration offsets loaded from urdf")

    # Reads the maximum velocity
    # The max_velocity parameter is only used for debugging in the ur_driver. It's not related to actual velocity limits
    global max_velocity
    max_velocity = rospy.get_param("~max_velocity", MAX_VELOCITY) # [rad/s] 
    rospy.loginfo("Max velocity accepted by ur_driver: %s [rad/s]" % max_velocity)
    
    # Reads the minimum payload
    global min_payload
    min_payload = rospy.get_param("~min_payload", MIN_PAYLOAD)
    # Reads the maximum payload
    global max_payload
    max_payload = rospy.get_param("~max_payload", MAX_PAYLOAD)
    rospy.loginfo("Bounds for Payload: [%s, %s]" % (min_payload, max_payload))
    

    # Sets up the server for the robot to connect to
    server = TCPServer(("", reverse_port), CommanderTCPHandler)
    thread_commander = threading.Thread(name="CommanderHandler", target=server.serve_forever)
    thread_commander.daemon = True
    thread_commander.start()

    with open(roslib.packages.get_pkg_dir('ur_driver') + '/prog') as fin:
        program = fin.read() % {"driver_hostname": get_my_ip(robot_hostname, PORT), "driver_reverseport": reverse_port}
    connection = URConnection(robot_hostname, PORT, program)
    connection.connect()
    connection.send_reset_program()
    
    connectionRT = URConnectionRT(robot_hostname, RT_PORT)
    connectionRT.connect()
    
    set_io_server()
    
    service_provider = None
    action_server = None
    try:
        while not rospy.is_shutdown():
            # Checks for disconnect
            if getConnectedRobot(wait=False):
                time.sleep(0.2)
                try:
                    prevent_programming = rospy.get_param("~prevent_programming")
                    update = {'prevent_programming': prevent_programming}
                    reconfigure_srv.update_configuration(update)
                except KeyError as ex:
                    print("Parameter 'prevent_programming' not set. Value: " + str(prevent_programming))
                    pass
                if prevent_programming:
                    print("Programming now prevented")
                    connection.send_reset_program()
            else:
                print("Disconnected.  Reconnecting")
                if action_server:
                    action_server.set_robot(None)

                rospy.loginfo("Programming the robot")
                while True:
                    # Sends the program to the robot
                    while not connection.ready_to_program():
                        print("Waiting to program")
                        time.sleep(1.0)
                    try:
                        prevent_programming = rospy.get_param("~prevent_programming")
                        update = {'prevent_programming': prevent_programming}
                        reconfigure_srv.update_configuration(update)
                    except KeyError as ex:
                        print("Parameter 'prevent_programming' not set. Value: " + str(prevent_programming))
                        pass
                    connection.send_program()

                    r = getConnectedRobot(wait=True, timeout=1.0)
                    if r:
                        break
                rospy.loginfo("Robot connected")

                #provider for service calls
                if service_provider:
                    service_provider.set_robot(r)
                else:
                    service_provider = URServiceProvider(r)
                
                if action_server:
                    action_server.set_robot(r)
                else:
                    action_server = URTrajectoryFollower(r, rospy.Duration(1.0))
                    action_server.start()

    except KeyboardInterrupt:
        try:
            r = getConnectedRobot(wait=False)
            rospy.signal_shutdown("KeyboardInterrupt")
            if r: r.send_quit()
        except:
            pass
        raise

if __name__ == '__main__': main()
