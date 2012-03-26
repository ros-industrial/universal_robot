#!/usr/bin/env python
import roslib; roslib.load_manifest('ur5_driver')
import time, sys, threading
import socket
import struct
import SocketServer
import rospy
from sensor_msgs.msg import JointState

HOSTNAME='ur-xx'
HOSTNAME="10.0.1.20"
PORT=30001

MSG_OUT = 1
MSG_QUIT = 2
MSG_JOINT_STATES = 3
MULT_jointstate = 1000.0

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

pub_joint_states = rospy.Publisher('joint_states', JointState)

class EOF(Exception): pass

dump_state = open('dump_state', 'wb')

# Receives messages from the robot over the socket
class CommanderTCPHandler(SocketServer.BaseRequestHandler):
    def recv_more(self):
        more = self.request.recv(4096)
        if not more:
            raise EOF()
        return more
    
    def handle(self):
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
                    print "Out: %s" % s
                elif mtype == MSG_JOINT_STATES:
                    while len(buf) < 3*(6*4):
                        buf = buf + self.recv_more()
                    state_mult = struct.unpack_from("!" + ("i"*3*6), buf, 0)
                    buf = buf[3*6*4:]
                    state = [s / MULT_jointstate for s in state_mult]

                    msg = JointState()
                    msg.header.stamp = rospy.get_rostime()
                    msg.name = JOINT_NAMES
                    msg.position = state[:6]
                    msg.velocity = state[6:12]
                    msg.effort = state[12:18]
                    pub_joint_states.publish(msg)
                elif mtype == MSG_QUIT:
                    print "Quitting"
                    sys.exit(0)
                else:
                    raise Exception("Unknown message type: %i" % mtype)

                if not buf:
                    buf = buf + self.recv_more()
        except EOF:
            print "Connection closed (command)"
            

class TCPServer(SocketServer.TCPServer):
    allow_reuse_address = True
    timeout = 5


# Waits until all threads have completed.  Allows KeyboardInterrupt to occur
def joinAll(threads):
    while any(t.isAlive() for t in threads):
        for t in threads:
            t.join(0.1)

sock = None
def main():
    global sock
    rospy.init_node('ur5_driver', disable_signals=True)

    # Sends the program to the robot
    sock = socket.create_connection((HOSTNAME, PORT))
    with open('prog') as fin:
        sock.sendall(fin.read())

    server = TCPServer(("", 50001), CommanderTCPHandler)

    thread_commander = threading.Thread(name="CommanderHandler", target=server.handle_request)
    thread_commander.daemon = True
    thread_commander.start()

    # Waits for the threads to finish
    joinAll([thread_commander])
    
    print "Dump"
    print "="*70
    o = ""
    while len(o) < 4096:
        r = sock.recv(1024)
        if not r: break
        o = o + r
    print o
        

if __name__ == '__main__': main()
