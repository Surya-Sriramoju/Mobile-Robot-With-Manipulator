#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from pynput import keyboard

# Define key codes
a,b,c,d,e,f = 0.0,0.0,0.0,0.0,0.0,0.0

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel = 0.0
        steer_angle = 0.0
        a,b,c,d,e,f = 0.0,0.0,0.0,0.0,0.0,0.0

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'p':  # Quit
                    linear_vel = 0.0
                    steer_angle = 0.0
                elif key == 'a':
                    a += 0.1
                elif key == 's':
                    b += 0.1
                elif key == 'd':
                    c += 0.1
                elif key == 'f':
                    d += 0.1
                elif key == 'g':
                    e += 0.1

                elif key == 'q':
                    a -= 0.1
                elif key == 'w':
                    b -= 0.1
                elif key == 'e':
                    c -= 0.1
                elif key == 'r':
                    d -= 0.1
                elif key == 't':
                    e -= 0.1

                wheel_velocities.data = [linear_vel, -linear_vel, linear_vel, -linear_vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                joint_positions.data = [steer_angle, steer_angle, a, b, c, d, e, f]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    rclpy.shutdown()

if __name__ == '__main__':
    main()