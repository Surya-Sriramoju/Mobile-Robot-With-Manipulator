#!/usr/bin/env python3
from IK_numerical import *
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster
from  math import pi
import numpy as np
from utils import *
from params import *
import time
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
import sys


def toggle_gripper(node, state):
    gripper_switch_client = node.create_client(SetBool, '/demo/custom_switch')
    request = SetBool.Request()
    request.data = state
    future = gripper_switch_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f"Gripper switched {'on' if state else 'off'} successfully.")
    else:
        node.get_logger().error("Failed to switch gripper.")

def publish_joints(joints_list):
    for joint in joints_list:
        pub_joint = [0.0, 0.0] + list(joint)
        time.sleep(0.1)
        joint_positions.data = pub_joint
        joint_position_pub.publish(joint_positions)


if __name__ =="__main__":

    rclpy.init(args=None)
    node = rclpy.create_node("main_ur5_mobile_robot")

    rate = node.create_rate(10)

    joint_position_pub = node.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
    wheel_velocities_pub = node.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
    joint_state_pub = node.create_publisher(JointState, '/joint_states', 10)
    joint_positions = Float64MultiArray()
    wheel_velocities = Float64MultiArray()

    picking_theta = np.array([-2.0000000000000004, 0.7999999999999999, 0.0, -2.3000000000000007, -1.6000000000000003, 0.0])
    leaning_theta = np.array([-pi / 2, pi/2, -0.1, -1.7000000000000002, -pi / 2, 0.0])
    placing_theta = np.array([-0.7999999999999997, 1.3, -0.7, -2.700000000000001, -1.6000000000000003, 0.0])

    node.get_logger().info("\nMoving Robot to Zero location   : %s" % str(picking_theta))

    joint_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_position_pub.publish(joint_positions)


    Tsd_Pick, _ = FKinDHParam(picking_theta, Table_DHParam)
    Tsd_Lean, _ = FKinDHParam(leaning_theta, Table_DHParam)
    Tsd_Place, _ = FKinDHParam(placing_theta, Table_DHParam)

    picking_theta_guess = np.array([-2.2995 + pi, -0.6288, 1.2288, 0.970846, -1.5707 + pi, .72872 + pi])
    leaning_theta_guess = np.array([pi / 1.5, -pi / 1.5, pi / 1.6, -pi / 16, pi / 2, pi / 2])
    placing_theta_guess = np.array([0.8420926 + pi / 3, -0.6288, 1.2288, 0.970846, -1.5707 + pi, .72872 + pi])



    picking_theta_sol, done = ik(Table_DHParam, Tsd_Pick, picking_theta_guess, 0.0001)
    if done:
        node.get_logger().info("Computed Pick location : %s" % str(picking_theta_sol[-1]))
    else:
        node.get_logger().info("IK failed for Pick")



    leaning_theta_sol, done = ik(Table_DHParam, Tsd_Lean, leaning_theta_guess, 0.0001)
    if done:
        node.get_logger().info("Computed Lean location : %s" % str(leaning_theta_sol[-1]))
    else:
        node.get_logger().info("IK failed for Lean")



    placing_theta_sol, done = ik(Table_DHParam, Tsd_Place, placing_theta_guess, 0.0001)
    if done:
        node.get_logger().info("Computed Place location : %s" % str(placing_theta_sol[-1]))
    else:
        node.get_logger().info("IK failed for Place")


    pick = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.8999999999999999, 0.0, 0.0, 0.0, 0.0],
    [-0.7999999999999999, 1.3, 0.0, 0.0, 0.0, 0.0],
    [-2.1000000000000005, 1.3, 0.0, 0.0, 0.0, 0.0],
    [-2.1000000000000005, 1.3, 0.0, 0.0, 0.0, 0.0],
    [-2.1000000000000005, 1.2, 0.0, 0.0, 0.0, 0.0],
    [-1.9000000000000004, 0.9999999999999999, 0.1, 0.0, 0.0, 0.0],
    [-1.9000000000000004, 0.9999999999999999, 0.2, 0.0, 0.0, 0.0],
    [-2.1000000000000005, 0.9999999999999999, 0.1, 0.0, 0.0, 0.0],
    [-2.1000000000000005, 0.7999999999999999, 0.0, 0.0, 0.0, 0.0],
    [-2.1000000000000005, 0.8999999999999999, 0.1, 0.0, 0.0, 0.0],
    [-2.1000000000000005, 0.8999999999999999, 0.1, -0.30000000000000004, 0.0, 0.0],
    [-2.1000000000000005, 0.8999999999999999, 0.1, -0.8999999999999999, 0.1, 0.0],
    [-2.1000000000000005, 0.8999999999999999, 0.1, -1.4000000000000001, -0.5, 0.0],
    [-2.1000000000000005, 0.8999999999999999, 0.1, -1.3, -1.4000000000000001, 0.0],
    [-2.1000000000000005, 0.8999999999999999, 0.1, -1.8000000000000005, -1.4000000000000001, 0.0],
    [-2.1000000000000005, 0.8999999999999999, 0.0, -1.8000000000000005, -1.4000000000000001, 0.0],
    [-2.1000000000000005, 0.8999999999999999, 0.0, -1.8000000000000005, -1.4000000000000001, 0.0],
    [-2.1000000000000005, 0.8999999999999999, 0.0, -2.2000000000000006, -1.4000000000000001, 0.0],
    [-2.1000000000000005, 0.8999999999999999, -0.2, -2.2000000000000006, -1.4000000000000001, 0.0],
    [-2.1000000000000005, 0.8999999999999999, -0.2, -2.2000000000000006, -1.4000000000000001, 0.0],
    [-2.0000000000000004, 0.8999999999999999, -0.2, -2.1000000000000005, -1.4000000000000001, 0.0],
    [-2.1000000000000005, 0.9999999999999999, -0.2, -2.1000000000000005, -1.4000000000000001, 0.0],
    [-2.1000000000000005, 0.9999999999999999, -0.2, -2.1000000000000005, -1.4000000000000001, 0.0]]

    node.get_logger().info("Picking the object")
    publish_joints(smooth(pick))
    toggle_gripper(node,True)
    time.sleep(2)

    lean = [[-2.2000000000000006, 1.0999999999999999, -0.2, -2.3000000000000007, -1.7000000000000004, 0.0],
    [-1.7000000000000002, 1.4000000000000001, -0.2, -2.3000000000000007, -1.7000000000000004, 0.0],
    [-1.5, 1.5000000000000002, -0.2, -2.3000000000000007, -1.7000000000000004, 0.0],
    [-1.5, 1.5000000000000002, -0.2, -2.3000000000000007, -1.7000000000000004, 0.0],
    [-1.5, 1.4000000000000001, -0.2, -2.3000000000000007, -1.7000000000000004, 0.0],
    [-1.5, 1.7000000000000004, -0.2, -2.3000000000000007, -1.7000000000000004, 0.0],
    [-1.5, 1.7000000000000004, -0.6, -2.3000000000000007, -1.7000000000000004, 0.0],
    [-1.2999999999999998, 1.8000000000000005, -0.6, -2.3000000000000007, -1.7000000000000004, 0.0],
    [-1.2999999999999998, 1.8000000000000005, -0.7, -2.3000000000000007, -1.7000000000000004, 0.0]]

    node.get_logger().info("going to lean position")
    publish_joints(smooth(lean))
    time.sleep(2)

    place = [[-1.2999999999999998, 1.8000000000000005, -0.7, -2.3000000000000007, -1.7000000000000004, 0.0],
    [-0.9999999999999997, 1.8000000000000005, -0.6, -2.400000000000001, -1.6000000000000003, 0.0],
    [-0.9999999999999997, 1.4000000000000001, -0.6, -2.400000000000001, -1.6000000000000003, 0.0],
    [-0.9999999999999997, 1.4000000000000001, -0.6, -2.400000000000001, -1.6000000000000003, 0.0],
    [-0.9999999999999997, 1.4000000000000001, -0.5, -2.400000000000001, -1.6000000000000003, 0.0],
    [-0.8999999999999997, 1.2, -0.5, -2.400000000000001, -1.6000000000000003, 0.0],
    [-0.8999999999999997, 1.2, -0.4, -2.400000000000001, -1.6000000000000003, 0.0],
    [-0.8999999999999997, 1.2, -0.4, -2.400000000000001, -1.6000000000000003, 0.0],
    [-0.8999999999999997, 1.2, -0.4, -2.400000000000001, -1.6000000000000003, 0.0],
    [-0.8999999999999997, 1.2, -0.4, -2.3000000000000007, -1.6000000000000003, 0.0],
    [-0.8999999999999997, 1.2, -0.5, -2.400000000000001, -1.6000000000000003, 0.0],
    [-0.8999999999999997, 1.2, -0.5, -2.400000000000001, -1.6000000000000003, 0.0]]

    node.get_logger().info("Placing the object")
    publish_joints(smooth(place))
    time.sleep(1)
    toggle_gripper(node,False)
    time.sleep(1)
    place.reverse()
    publish_joints(smooth(place))
    time.sleep(1)
    sys.exit(0)

    while rclpy.ok():
        rate.sleep()

