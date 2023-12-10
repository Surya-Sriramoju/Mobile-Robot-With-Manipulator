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
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray


# joint_position_pub = create_publisher(Float64MultiArray, '/position_controller/commands', 10)
# wheel_velocities_pub = create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
# joint_state_pub = create_publisher(JointState, '/joint_states', 10)
# joint_positions = Float64MultiArray()
# wheel_velocities = Float64MultiArray()
# Functions to call ros service for vacuum gripper

def gripper_on(node):
    # Create a client to call the gripper service
    try:
        gripper_client = node.create_client(Empty, "/demo/vacuum_gripper/on")
        if not gripper_client.service_is_ready():
            gripper_client.wait_for_service()
        req = Empty.Request()
        future = gripper_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            return True
        else:
            return False
    except Exception as e:
        print(f"Failed to call gripper service: {e}")
        return False

def gripper_off(node):
    # Create a client to call the gripper service
    try:
        gripper_client = node.create_client(Empty, "/demo/vacuum_gripper/off")
        if not gripper_client.service_is_ready():
            gripper_client.wait_for_service()
        req = Empty.Request()
        future = gripper_client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            return True
        else:
            return False
    except Exception as e:
        print(f"Failed to call gripper service: {e}")
        return False

# def publish_joints(joints_list):
#     node.get_logger().info("inside loop")
#     # for joint in joints_list:
        
#     #     pub_joints = [0.0, 0.0] + list(joint)
#     #     time.sleep(1)
#     #     joint_positions.data = pub_joints
#     #     joint_position_pub.publish(joint_positions)




# Main
if __name__ =="__main__":

    #initializing the node 
    rclpy.init(args=None)
    node = rclpy.create_node("main_ur5_mobile_robot")

    rate = node.create_rate(10)

    joint_position_pub = node.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
    wheel_velocities_pub = node.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
    joint_state_pub = node.create_publisher(JointState, '/joint_states', 10)
    joint_positions = Float64MultiArray()
    wheel_velocities = Float64MultiArray()
    # joint_publisher = robot_joint_publisher(node)

    # The known positions for thetas to pick and place the object
    theta_pick = np.array([0.8420926, -0.6288, 1.2288, 0.970846, -1.5707 + pi, .72872 + pi])
    theta_lean = np.array([pi / 2, -pi / 1.5, pi / 1.5, -pi / 16, pi / 2, pi / 2])
    theta_place = np.array([0.8420926 + pi / 3, -0.6288, 1.2288, 0.970846, -1.5707 + pi, .72872 + pi])

    # Home position for the robot
    theta_zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    node.get_logger().info("\nMoving Robot to Zero location   : %s" % str(theta_pick))
    # joint_publisher.publish_joint(theta_zero)

    # Setting up the Forward kinematics, DH table
    Tsd_Pick, _ = FKinDHParam(theta_pick, Table_DHParam)
    Tsd_Lean, _ = FKinDHParam(theta_lean, Table_DHParam)
    Tsd_Place, _ = FKinDHParam(theta_place, Table_DHParam)

    # node.get_logger().info(str(theta_pick))
    # node.get_logger().info(str(Tsd_Pick.flatten()))

    # Putting in the approximate guess values for the pick and place for Newton-Raphson method
    theta_pick_guess = np.array([-2.2995 + pi, -0.6288, 1.2288, 0.970846, -1.5707 + pi, .72872 + pi])
    theta_lean_guess = np.array([pi / 1.5, -pi / 1.5, pi / 1.6, -pi / 16, pi / 2, pi / 2])
    theta_place_guess = np.array([0.8420926 + pi / 3, -0.6288, 1.2288, 0.970846, -1.5707 + pi, .72872 + pi])

    # Calculating the inverse kinematics for the pick, place, and guess using the inverse kinematics solver
    # node.get_logger().info("\nActual Pick location   : %s" % str(theta_pick))
    # node.get_logger().info("Guess Pick location    : %s" % str(theta_pick_guess))
    # node.get_logger().info("Running IK for Pick location : ")



    theta_pick_sol, done = ik(Table_DHParam, Tsd_Pick, theta_pick_guess, 0.0001)
    if done:
        node.get_logger().info("Computed Pick location : %s" % str(theta_pick_sol[-1]))
    else:
        node.get_logger().info("IK failed for Pick")

    # node.get_logger().info("\nActual Lean location   : %s" % str(theta_lean))
    # node.get_logger().info("Guess Lean location    : %s" % str(theta_lean_guess))
    # node.get_logger().info("Running IK for Lean location : ")



    theta_lean_sol, done = ik(Table_DHParam, Tsd_Lean, theta_lean_guess, 0.0001)
    if done:
        node.get_logger().info("Computed Lean location : %s" % str(theta_lean_sol[-1]))
    else:
        node.get_logger().info("IK failed for Lean")



    # node.get_logger().info("\nActual Place location   : %s" % str(theta_place))
    # node.get_logger().info("Guess Place location    : %s" % str(theta_place_guess))
    # node.get_logger().info("Running IK for Place location : ")


    theta_place_sol, done = ik(Table_DHParam, Tsd_Place, theta_place_guess, 0.0001)
    if done:
        node.get_logger().info("Computed Place location : %s" % str(theta_place_sol[-1]))
    else:
        node.get_logger().info("IK failed for Place")

    # Publishing the Pick, Place, Lean, and Home joint angles to the joint publisher
    # joint_publisher.publish_joint(theta_pick_sol)
    # gripper_on(node)
    # joint_publisher.publish_joint(theta_place_sol)
    # time.sleep(2)
    # gripper_off(node)
    # joint_publisher.publish_joint(theta_lean_sol)
    # joint_publisher.publish_joint(theta_zero)

    # node.get_logger().info("Pick and Place completed, now returning to home")

    # publish_joints(theta_pick_sol)
    # time.sleep(1)
    # publish_joints(theta_lean_sol)
    # time.sleep(1)
    # publish_joints(theta_place_sol)
    node.get_logger().info("Computed Place location : %d" % len(theta_pick_sol))


    while rclpy.ok():
        rate.sleep()

