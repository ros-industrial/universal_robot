#!/usr/bin/env python
import roslib; roslib.load_manifest('universal_robot_driver')
import time, sys
import socket
import struct
import SocketServer

HOSTNAME='ur-xx'
HOSTNAME="10.0.1.20"
PORT=30001

MSG_OUT = 1
MSG_QUIT = 2
MSG_JOINT_STATES = 3
MULT_jointstate = 1000.0

class EOF(Exception): pass

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
                print "Buf:", [ord(b) for b in buf]

                # Unpacks the message type
                mtype = struct.unpack_from("!i", buf, 0)[0]
                buf = buf[4:]
                print "Message type:", mtype

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
                elif mtype == MSG_QUIT:
                    print "Quitting"
                    sys.exit(0)
                elif mtype == MSG_JOINT_STATES:
                    while len(buf) < 3*(6*4):
                        buf = buf + self.recv_more()
                    state_mult = struct.unpack_from("!" + ("i"*3*6), buf, 0)
                    buf = buf[3*6*4:]
                    state = [s / MULT_jointstate for s in state_mult]
                    print "Joint state:", state
                else:
                    raise Exception("Unknown message type: %i" % mtype)

                if not buf:
                    buf = buf + self.recv_more()
        except EOF:
            print "Connection closed"
            

class TCPServer(SocketServer.TCPServer):
    allow_reuse_address = True
    timeout = 5

sock = None
def main():
    global sock

    # Sends the program to the robot
    sock = socket.create_connection((HOSTNAME, PORT))
    with open('prog') as fin:
        sock.sendall(fin.read())

    server = TCPServer(("", 50001), CommanderTCPHandler)
    server.handle_request()
        

if __name__ == '__main__': main()
