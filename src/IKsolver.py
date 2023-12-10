#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
from control_msgs.action import FollowJointTrajectory as FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sympy import symbols,cos,sin,simplify,Matrix,pprint,evalf,diff,sqrt
import math
from functools import partial
from mpl_toolkits.mplot3d import axes3d, Axes3D
import time
from std_srvs.srv import SetBool

from rclpy.task import Future

class Circle_UR5(Node):

    def __init__(self):
        super().__init__('node_circle')
        self.q_pub1 = []
        self.q_pub2 = []
        self.q_pub3 = []
        self.q_pub4 = []
        self.q_pub5 = []
        self.q_pub6 = []
        self.round3 = partial(round, ndigits=3)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
                # Create a client for the gripper service
        self.gripper_switch_client = self.create_client(SetBool, '/demo/custom_switch')
        ##############################
        self.joint_positions = Float64MultiArray()
        self.wheel_velocities = Float64MultiArray()
        self.joint_positions.data= [float(0.0), float(0.0), float(0.0), float(math.pi/2), float(math.pi/2), float(0.0), float(0.0000000000000001), float(0.001)]
        self.joint_position_pub.publish(self.joint_positions)

    def numerical_IK(self):
        
    #Transformation for A1
        theta1,d1 = symbols('theta1 d1', real=True)
        R_z_1 = Matrix(([cos(theta1),-sin(theta1),0,0],[sin(theta1),cos(theta1),0,0],[0,0,1,0],[0,0,0,1]))
        T_z_1 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d1],[0,0,0,1]))
        R_x_1 = Matrix(([1,0,0,0],[0,round(math.cos(math.radians(90))),-round(math.sin(math.radians(90))),0],[0,round(math.sin(math.radians(90))),round(math.cos(math.radians(90))),0],[0,0,0,1]))
        print("Transformation matrix for A1:")
        A1 = R_z_1*T_z_1*R_x_1
        pprint(A1)
        print("\n")

        #Transformation for A2
        theta2,a2 = symbols('theta2 a2', real=True)
        R_z_2 = Matrix(([cos(theta2),-sin(theta2),0,0],[sin(theta2),cos(theta2),0,0],[0,0,1,0],[0,0,0,1]))
        T_x_2 = Matrix(([1,0,0,a2],[0,1,0,0],[0,0,1,0],[0,0,0,1]))
        print("Transformation matrix for A2:")
        A2 = R_z_2*T_x_2
        pprint(A2)
        print("\n")

        # Transformation for A3
        theta3,a3 = symbols('theta3 a3', real=True)
        R_z_3 = Matrix(([cos(theta3),-sin(theta3),0,0],[sin(theta3),cos(theta3),0,0],[0,0,1,0],[0,0,0,1]))
        T_x_3 = Matrix(([1,0,0,a3],[0,1,0,0],[0,0,1,0],[0,0,0,1]))
        print("Transformation matrix for A3:")
        A3 = R_z_3*T_x_3
        pprint(A3)
        print("\n")

        #Transformation for A4
        theta4,d4 = symbols('theta4 d4', real=True)
        R_z_4 = Matrix(([cos(theta4),-sin(theta4),0,0],[sin(theta4),cos(theta4),0,0],[0,0,1,0],[0,0,0,1]))
        T_z_4 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d4],[0,0,0,1]))
        R_x_4 = Matrix(([1,0,0,0],[0,round(math.cos(math.radians(90))),-round(math.sin(math.radians(90))),0],[0,round(math.sin(math.radians(90))),round(math.cos(math.radians(90))),0],[0,0,0,1]))
        print("Transformation matrix for A4:")
        A4 = R_z_4*T_z_4*R_x_4
        pprint(A4)
        print("\n")

        #Transformation for A5
        theta5,d5 = symbols('theta5 d5', real=True)
        R_z_5 = Matrix(([cos(theta5),-sin(theta5),0,0],[sin(theta5),cos(theta5),0,0],[0,0,1,0],[0,0,0,1]))
        T_z_5 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d5],[0,0,0,1]))
        R_x_5 = Matrix(([1,0,0,0],[0,round(math.cos(math.radians(-90))),-round(math.sin(math.radians(-90))),0],[0,round(math.sin(math.radians(-90))),round(math.cos(math.radians(-90))),0],[0,0,0,1]))
        print("Transformation matrix for A5:")
        A5 = R_z_5*T_z_5*R_x_5
        pprint(A5)
        print("\n")


        #Transformation for A6
        theta6,d6 = symbols('theta6 d6', real=True)
        R_z_6 = Matrix(([cos(theta6),-sin(theta6),0,0],[sin(theta6),cos(theta6),0,0],[0,0,1,0],[0,0,0,1]))
        T_z_6 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d6],[0,0,0,1]))

        print("Transformation matrix for A6:")
        A6 = R_z_6*T_z_6
        pprint(A6)
        print("\n")



        #Final Transformation matrix 
        x1,x2,x3,y1,y2,y3,z1,z2,z3,p1,p2,p3 = symbols('x1 x2 x3 y1 y2 y3 z1 z2 z3 p1 p2 p3 ', real=True)
        A= A1*A2*A3*A4*A5*A6
        A_Final = A.subs([(a3,-0.39225),(a2,-0.42500),(d1,0.089159),(d4,0.10915),(d5,0.09465),(d6,0.0823)])
        print("The Final Transformation matrix in terms of joint angles as variables:")
        # A_Final
                    
        Ab = A_Final.subs([(theta1, 4.3521),(theta2,-0.6278),(theta4,0.9720),(theta5,-1.5708),(theta6,0.3603),(theta3, 1.2265)])
        Ab.applyfunc(self.round3)
        A_1=simplify(A)
        A_1
        # Qans = (A2*A3*A4*A5*A6).subs([(theta1,math.radians(0)),(theta2,math.radians(0)),(theta4,math.radians(0)),(theta3,math.radians(0))])
        # simplify(Qans.inv())
        #pose = Matrix(([1,0,0,p1],[0,1,0,p2],[0,0,1,p3],[0,0,0,1]))
        #pose
        #((A2*A3*A4*A5*A6).T).subs([(theta1,math.radians(0)),(theta2,math.radians(0)),(theta4,math.radians(0)),(theta3,math.radians(0))])

        # %%
        A10 = A1
        A20 = A1*A2
        A30 = A1*A2*A3
        A40 = A1*A2*A3*A4
        A50 = A1*A2*A3*A4*A5
        A60 = A1*A2*A2*A3*A4*A5

        # %%
        import time
        from matplotlib import pyplot as plt
        from mpl_toolkits.mplot3d import axes3d, Axes3D
        start = time.time() #start time


        # %%
        Xp0 = A60.col(3)
        Xp0.row_del(3)
        pprint(Xp0)

        # %%
        #Calculating Partial Differentials for the jacobian
        dele1 = diff(Xp0, theta1)
        dele2 = diff(Xp0, theta2)
        dele3 = diff(Xp0, theta3)
        dele4 = diff(Xp0, theta4)
        dele5 = diff(Xp0, theta5)
        dele6 = diff(Xp0, theta6)

        # %%
        #Calculating Z column vectors for Jacobian
        Z1 = A10.col(2)
        Z1.row_del(3)
        Z2 = A20.col(2)
        Z2.row_del(3)
        Z3 = A30.col(2)
        Z3.row_del(3)
        Z4 = A40.col(2)
        Z4.row_del(3)
        Z5 = A50.col(2)
        Z5.row_del(3)
        Z6 = A60.col(2)
        Z6.row_del(3)

        # %%
        #Defining the Jacobian Matrix
        Jacob = Matrix([[dele1,dele2,dele3,dele4,dele5,dele6],[Z1,Z2,Z3,Z4,Z5,Z6]])
        J=Jacob.evalf()
        pprint(J)


        # %%
        #Writing Geometric dimensions of robot in Jacobian and End effector transformation wrt Base
        J =J.subs([(a3,-0.39225),(a2,-0.42500),(d1,0.089159),(d4,0.10915),(d5,0.09465),(d6,0.0823)])
        A07 =A60.subs([(a3,-0.39225),(a2,-0.42500),(d1,0.089159),(d4,0.10915),(d5,0.09465),(d6,0.0823)])

        #Defining the initial value of joint angles given in question
        # q =  Matrix([[0.0], [math.pi/2], [math.pi/2], [0.0], [0.0000000000000001], [0.001]])
        q =  Matrix([[0.0], [math.pi/2], [math.pi/2], [0.0], [0.0000000000000001], [0.001]])

        #Defining the list for storing the values of x, y, and z of end effectors
        y = []
        z = []
        x = []

        i=0 #Defining the iterators

        # %%
        #Printing the initial Jacobian Matrix
        J2 = J
        J2 = J2.subs({theta1:q[0,0],theta2:q[1,0],theta3:q[2,0],theta4:q[3,0],theta5:q[4,0],theta6:q[5,0]}).evalf()
        pprint(J2)

        # %%
        #Define time step for loop
        T = 5 #Time required to complete one revolution
        N = 100 #No. of Iterations
        dt = T / N #Time Step
        i = 0
        
        # %%
        while (i<=100):  ## run this for loop for more than 1000 time to make sure that circle is being perfactly drawn
            ## divide the entire circle to be drawing in 1000 points
            # print(i)
            x_dot = -0.435987207*0.4*math.pi*sin((2*math.pi/100)*i)
            y_dot = 0.435987207*0.4*math.pi*cos((2*math.pi/100)*i)

            V = Matrix([x_dot,y_dot,0.0, 0.0, 0.0, 0.0]).evalf()
            
            ## find the transformation matrix from base to end effector in each iteration
            ## take theta3 = 0
            A = A07.subs({theta1:q[0,0],theta2:q[1,0],theta3:q[2,0],theta4:q[3,0],theta5:q[4,0],theta6:q[5,0]}).evalf()

            # position of ball point in 7th frame
            P7 = Matrix([0,0,0,1])

            ##Storing q values
            self.q_pub1.append(q[0,0])
            self.q_pub2.append(q[1,0])
            self.q_pub3.append(q[2,0])
            self.q_pub4.append(q[3,0])
            self.q_pub5.append(q[4,0])
            self.q_pub6.append(q[5,0])
            
            ## position of ball point in the origin frame
            P07 = A*P7
            ## add the value of x,y and z in the list at each iteration
            ## This will be used for plotting the circle later
            y.append(P07[1,0])
            z.append(P07[2,0])
            x.append(P07[0,0])
            ## jacobian to find the jacobian matrix for each iteration
            ## take theta3 = 0
            J1 = J.subs({theta1:q[0,0],theta2:q[1,0],theta3:q[2,0],theta4:q[3,0],theta5:q[4,0],theta6:q[5,0]}).evalf()
            J_inv = J1.inv('LU')
            # # find the difference in the position of end effector in each iteration
            # ## update the value of q 
            q = q + (J_inv*V*dt)
            i+=1



        # %%
        # plot the circle after finding x and z value at each iteration
        plt.plot(x,y)
        plt.xlabel("X coordinate")
        plt.ylabel("Y coordinate")
        plt.axis("equal")
        
        # plt.pause(0.05)
        plt.show()
        end = time.time()

    # %%

    # Inside Circle_UR5 class
    def toggle_gripper(self, state):
        request = SetBool.Request()
        request.data = state
        future = self.gripper_switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Gripper switched {'on' if state else 'off'} successfully.")
        else:
            self.get_logger().error("Failed to switch gripper.")


    # def move_joints(self):
    #     joint_positions = Float64MultiArray()
    #     wheel_velocities = Float64MultiArray()
    #     linear_vel = 0.0
    #     steer_angle = 0.0
        
    #     pprint("here")
    #     j=0

    #     while (j<=100):
    #         #qpub = [q_pub1[j],q_pub2[j],q_pub3[j],q_pub4[j],q_pub5[j],q_pub6[j]]
          
    #         wheel_velocities.data = [linear_vel, -linear_vel, linear_vel, -linear_vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #         joint_positions.data = [steer_angle, steer_angle, float(self.q_pub1[j]), float(self.q_pub2[j]), float(self.q_pub3[j]),float(self.q_pub4[j]),float(self.q_pub5[j]),float(self.q_pub6[j])]
    #         time.sleep(0.05)
    #         self.joint_position_pub.publish(joint_positions)
    #         self.wheel_velocities_pub.publish(wheel_velocities)
    #         j+=1

    # Inside Circle_UR5 class
    def move_joints(self):
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel = 0.0
        steer_angle = 0.0

        # Move to home position
        self.toggle_gripper(False)  # Ensure gripper is off
        self.go_to_home_position()

        # Pick operation
        self.toggle_gripper(True)  # Turn on gripper
        self.pick_object()

        # Move to home position (optional, adjust as needed)
        # self.go_to_home_position()


        j = 0
        while (j <= 100):
            wheel_velocities.data = [linear_vel, -linear_vel, linear_vel, -linear_vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            joint_positions.data = [steer_angle, steer_angle, float(self.q_pub1[j]), float(self.q_pub2[j]),
                                    float(self.q_pub3[j]), float(self.q_pub4[j]), float(self.q_pub5[j]), float(self.q_pub6[j])]
            time.sleep(0.05)
            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)
            j += 1

        # Place operation
        self.toggle_gripper(False)  # Turn off gripper
        # self.place_object()
        self.go_to_home_position()
        

    # Inside Circle_UR5 class
    def go_to_home_position(self):
        print("go to home position")
        # Assuming the home position is a predefined joint configuration
        home_joint_positions = [0.0, 0.0, 0.0, math.pi/2, math.pi/2, 0.0, 0.0000000000000001, 0.001]
        self.move_to_joint_positions(home_joint_positions)

    def pick_object(self):
        print("enter pick object")
        # Move the gripper down to pick up the object
        # pick_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000000000000001, 0.001]
        # self.move_to_joint_positions(pick_joint_positions)

        # Close the gripper to pick up the object
        self.toggle_gripper(True)

    def place_object(self):
        print("enter place object")
        # Assuming the place position is another predefined joint configuration
        place_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000000000000001, 0.001]
        self.move_to_joint_positions(place_joint_positions)

        # Open the gripper to release the object
        self.toggle_gripper(False)

    # Add this helper method to simplify joint position commands
    def move_to_joint_positions(self, joint_positions):
        joint_positions_msg = Float64MultiArray()
        joint_positions_msg.data = joint_positions
        self.joint_position_pub.publish(joint_positions_msg)
        time.sleep(2)  # Adjust sleep time based on your robot's motion time


    
def main(args=None):
    

    rclpy.init(args=args)
    node = Circle_UR5()
    node.numerical_IK()
    node.move_joints()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

