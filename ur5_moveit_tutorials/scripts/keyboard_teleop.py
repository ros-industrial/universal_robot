#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty
import threading
import moveit_commander

GROUP_NAME = "manipulator"

def get_end_effector_name():
    moveit_commander.roscpp_initialize(sys.argv)
    group = moveit_commander.MoveGroupCommander(GROUP_NAME)
    end_effector_link = group.get_end_effector_link()
    rospy.loginfo("End effector link for group '%s': %s", GROUP_NAME, end_effector_link)

    return end_effector_link

class TeleopTwistKeyboard:
    def __init__(self):
        self.end_effector_link = get_end_effector_name()
        # Key mappings for linear and angular velocities
        self.key_bindings = {
            'w': (1, 0, 0, 0, 0, 0),  # Move forward in x
            's': (-1, 0, 0, 0, 0, 0), # Move backward in x
            'a': (0, 1, 0, 0, 0, 0),  # Move left in y
            'd': (0, -1, 0, 0, 0, 0), # Move right in y
            'r': (0, 0, 1, 0, 0, 0),  # Move up in z
            'f': (0, 0, -1, 0, 0, 0), # Move down in z
            'i': (0, 0, 0, 1, 0, 0),  # Roll +
            'k': (0, 0, 0, -1, 0, 0), # Roll -
            'j': (0, 0, 0, 0, 1, 0),  # Pitch +
            'l': (0, 0, 0, 0, -1, 0), # Pitch -
            'u': (0, 0, 0, 0, 0, 1),  # Yaw +
            'o': (0, 0, 0, 0, 0, -1),  # Yaw -
            'n': (0, 0, 0, 0, 0, 0)  # Reset
        }

        # Initialize twist message and node parameters
        self.last_twist = TwistStamped()
        self.linear_increment = 0.02
        self.angular_increment = 0.02
        self.finished_work = False

        self.pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Start the publishing thread
        self.pub_thread = threading.Thread(target=self.publish_twist)
        self.pub_thread.start()

    def publish_twist(self):
        rate = rospy.Rate(50)  # Publish rate in Hz
        while not self.finished_work and not rospy.is_shutdown():
            self.last_twist.header.stamp = rospy.Time.now()
            self.last_twist.header.frame_id = self.end_effector_link
            self.pub.publish(self.last_twist)
            rate.sleep()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def reset_velocities(self):
        self.last_twist = TwistStamped()  # Reset velocities

    def run(self):
        try:
            print("""
            Control the end-effector using keyboard commands:
            ------------------------------------------------
            Linear velocity controls:
            w/s : increase/decrease x-axis velocity
            a/d : increase/decrease y-axis velocity
            r/f : increase/decrease z-axis velocity

            Angular velocity controls:
            i/k : increase/decrease roll (x-axis rotation)
            j/l : increase/decrease pitch (y-axis rotation)
            u/o : increase/decrease yaw (z-axis rotation)
                  
            n: Reset all velocities

            Press 'Ctrl+C' to exit.
            """)

            while not rospy.is_shutdown():
                key = self.get_key()
                if key in self.key_bindings:
                    # Get the corresponding linear and angular directions
                    lin_x, lin_y, lin_z, ang_x, ang_y, ang_z = self.key_bindings[key]

                    # Update the twist message with incremental values
                    self.last_twist.twist.linear.x += lin_x * self.linear_increment
                    self.last_twist.twist.linear.y += lin_y * self.linear_increment
                    self.last_twist.twist.linear.z += lin_z * self.linear_increment
                    self.last_twist.twist.angular.x += ang_x * self.angular_increment
                    self.last_twist.twist.angular.y += ang_y * self.angular_increment
                    self.last_twist.twist.angular.z += ang_z * self.angular_increment

                if key == 'n':
                    self.reset_velocities()
                if key == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt

        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo("Shutting down. Publishing zero velocities to stop the robot.")
            self.reset_velocities()

        finally:
            # Reset terminal settings on exit and signal the publishing thread to stop
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.finished_work = True
            self.pub_thread.join()


if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('minimal_teleop_twist_keyboard')
    teleop = TeleopTwistKeyboard()
    teleop.run()
