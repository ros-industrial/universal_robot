#!/usr/bin/env python
import roslib; roslib.load_manifest('ur5_driver')
import time, sys, threading, math
import copy
import datetime
import socket, select
import struct
import traceback, code
import SocketServer

import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

HOSTNAME='ur-xx'
HOSTNAME="10.0.1.20"
PORT=30001

MSG_OUT = 1
MSG_QUIT = 2
MSG_JOINT_STATES = 3
MSG_MOVEJ = 4
MSG_WAYPOINT_FINISHED = 5
MSG_STOPJ = 6
MSG_SERVOJ = 7
MULT_jointstate = 1000.0
MULT_time = 1000000.0
MULT_blend = 1000.0

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]
  

connected_robot = None
connected_robot_lock = threading.Lock()
connected_robot_cond = threading.Condition(connected_robot_lock)
pub_joint_states = rospy.Publisher('joint_states', JointState)
dump_state = open('dump_state', 'wb')

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
    print "\n".join(code)

def log(s):
    print "[%s] %s" % (datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'), s)

def setConnectedRobot(r):
    global connected_robot, connected_robot_lock
    with connected_robot_lock:
        connected_robot = r
        connected_robot_cond.notify()

def getConnectedRobot(wait=False):
    with connected_robot_lock:
        if wait:
            while not connected_robot:
                connected_robot_cond.wait(0.5)
                #print "Threads:"
                #dumpstacks()
        return connected_robot

# Receives messages from the robot over the socket
class CommanderTCPHandler(SocketServer.BaseRequestHandler):
        
    def recv_more(self):
        while True:
            r, _, _ = select.select([self.request], [], [], 0.2)
            if r:
                more = self.request.recv(4096)
                if not more:
                    raise EOF()
                return more
            else:
                if self.last_joint_states and \
                        self.last_joint_states.header.stamp.to_sec() < time.time() + 1.0:
                    rospy.logerr("Stopped hearing from robot.  Disconnected")
                    raise EOF()

    def handle(self):
        self.socket_lock = threading.Lock()
        self.last_joint_states = None
        setConnectedRobot(self)
        print "Handling a request"
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

                elif mtype == MSG_JOINT_STATES:
                    while len(buf) < 3*(6*4):
                        buf = buf + self.recv_more()
                    state_mult = struct.unpack_from("!%ii" % (3*6), buf, 0)
                    buf = buf[3*6*4:]
                    state = [s / MULT_jointstate for s in state_mult]

                    msg = JointState()
                    msg.header.stamp = rospy.get_rostime()
                    msg.name = JOINT_NAMES
                    msg.position = state[:6]
                    msg.velocity = state[6:12]
                    msg.effort = state[12:18]
                    pub_joint_states.publish(msg)
                    self.last_joint_states = msg
                elif mtype == MSG_QUIT:
                    print "Quitting"
                    raise EOF()
                elif mtype == MSG_WAYPOINT_FINISHED:
                    while len(buf) < 4:
                        buf = buf + self.recv_more()
                    waypoint_id = struct.unpack_from("!i", buf, 0)[0]
                    buf = buf[4:]
                    print "Waypoint finished (not handled)"
                else:
                    raise Exception("Unknown message type: %i" % mtype)

                if not buf:
                    buf = buf + self.recv_more()
        except EOF:
            print "Connection closed (command)"
            setConnectedRobot(None)

    def send_quit(self):
        with self.socket_lock:
            self.request.send(struct.pack("!i", MSG_QUIT))
            
    def send_movej(self, waypoint_id, q, a=3, v=0.75, t=0, r=0):
        assert(len(q) == 6)
        params = [MSG_MOVEJ, waypoint_id] + \
                 [MULT_jointstate * qq for qq in q] + \
                 [MULT_jointstate * a, MULT_jointstate * v, MULT_time * t, MULT_blend * r]
        buf = struct.pack("!%ii" % len(params), *params)
        with self.socket_lock:
            self.request.send(buf)

    def send_servoj(self, waypoint_id, q, t):
        assert(len(q) == 6)
        params = [MSG_SERVOJ, waypoint_id] + \
                 [MULT_jointstate * qq for qq in q] + \
                 [MULT_time * t]
        buf = struct.pack("!%ii" % len(params), *params)
        with self.socket_lock:
            self.request.send(buf)
        

    def send_stopj(self):
        with self.socket_lock:
            self.request.send(struct.pack("!i", MSG_STOPJ))

    def set_waypoint_finished_cb(self, cb):
        self.waypoint_finished_cb = cb

    # Returns the last JointState message sent out
    def get_joint_states(self):
        return self.last_joint_states
    

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
    return JointTrajectoryPoint(q, qdot, qddot, rospy.Duration(t_abs))

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

class UR5TrajectoryFollower(object):
    RATE = 0.02
    def __init__(self, robot):
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
        self.traj.joint_names = JOINT_NAMES
        self.traj.points = [JointTrajectoryPoint(
            positions = state.position,
            velocities = [0] * 6,
            accelerations = [0] * 6,
            time_from_start = rospy.Duration(0.0))]

    def start(self):
        self.init_traj_from_robot()
        self.server.start()

    def on_goal(self, goal_handle):
        log("on_goal")

        # Checks that the robot is connected
        if not self.robot:
            rospy.logerr("Received a goal, but the robot is not connected")
            goal_handle.set_rejected()
            return

        # Checks if the joints are just incorrect
        if set(goal_handle.get_goal().trajectory.joint_names) != set(JOINT_NAMES):
            rospy.logerr("Received a goal with incorrect joint names: (%s)" % \
                         ', '.join(goal_handle.get_goal().trajectory.joint_names))
            goal_handle.set_rejected()
            return

        # Orders the joints of the trajectory according to JOINT_NAMES
        reorder_traj_joints(goal_handle.get_goal().trajectory, JOINT_NAMES)
                
        with self.following_lock:
            if self.goal_handle:
                # Cancels the existing goal
                self.goal_handle.set_canceled()
                self.first_waypoint_id += len(self.goal_handle.get_goal().trajectory.points)
                self.goal_handle = None

            # Inserts the current setpoint at the head of the trajectory
            now = time.time()
            point0 = sample_traj(self.traj, now)
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
                self.traj.joint_names = JOINT_NAMES
                self.traj.points = [point0, point1]
                
                self.goal_handle.set_canceled()
                self.goal_handle = None
        else:
            goal_handle.set_canceled()

    def _update(self, event):
        if self.robot and self.traj:
            now = time.time()
            if (now - self.traj_t0) <= self.traj.points[-1].time_from_start.to_sec():
                setpoint = sample_traj(self.traj, now - self.traj_t0)
                try:
                    self.robot.send_servoj(999, setpoint.positions, 2 * self.RATE)
                except socket.error:
                    pass
            else:  # Off the end
                if self.goal_handle:
                    self.goal_handle.set_succeeded()
                    self.goal_handle = None

sock = None
def main():
    global sock
    rospy.init_node('ur5_driver', disable_signals=True)
    if rospy.get_param("use_sim_time", False):
        rospy.logfatal("use_sim_time is set!!!")
        sys.exit(1)

    # Sets up the server for the robot to connect to
    server = TCPServer(("", 50001), CommanderTCPHandler)
    thread_commander = threading.Thread(name="CommanderHandler", target=server.serve_forever)
    thread_commander.daemon = True
    thread_commander.start()
    
    action_server = None
    try:
        while True:
            # Checks for disconnect
            if getConnectedRobot(wait=False):
                time.sleep(0.2)
            else:
                print "Disconnected.  Reconnecting"
                if action_server:
                    action_server.set_robot(None)


                # Sends the program to the robot
                sock = socket.create_connection((HOSTNAME, PORT))
                with open('prog') as fin:
                    program = fin.read()
                    sock.sendall(program % {"driver_hostname": socket.getfqdn()})

                # Useful for determining if the robot failed to compile the program
                if False:
                    print "Dump"
                    print "="*70
                    o = ""
                    while len(o) < 4096:
                        r = sock.recv(1024)
                        if not r: break
                        o = o + r
                    print o

                rospy.loginfo("Waiting for the robot to connect back to the driver")
                print "Commander thread:", thread_commander.is_alive()
                r = getConnectedRobot(wait=True)
                rospy.loginfo("Robot connected")

                if action_server:
                    action_server.set_robot(r)
                else:
                    action_server = UR5TrajectoryFollower(r)
                    action_server.start()

    except KeyboardInterrupt:
        try:
            rospy.signal_shutdown("KeyboardInterrupt")
            if r: r.send_quit()
        except:
            pass
        raise

if __name__ == '__main__': main()
