import rclpy
import numpy as np
from numpy import cos, sin
from sympy import eye
from std_msgs.msg import Float64
from params import DHParam

np.set_printoptions(suppress=True)


def FKinDHParam(config, DHParam):
    tf_base_list = []
    tf_link_list = []

    tf_fixed_to_end_effector = np.eye(4)

    # Perform Forward Kinematics to find the Transformations and
    # end effector location
    for i in range(1, len(DHParam) + 1):
        joint = 'J' + str(i)

        # Compute transformation matrix from i-1 to ith link
        # if DHParam[joint].type == 'R':
        d = DHParam[joint].d
        a = DHParam[joint].a
        alpha = DHParam[joint].alpha
        theta = config[i - 1]

        tf = np.array([
            [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
            [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ], dtype='f')

        # Find Each link's transformation with respect to the fixed frame
        tf_fixed_to_end_effector = tf_fixed_to_end_effector @ tf
        tf_link_list.append(tf)
        tf_base_list.append(tf_fixed_to_end_effector)

    return tf_fixed_to_end_effector, tf_base_list


def get_jacobian(config, DHParam):
    # Create five configurations of the robot with different values for theta1 to theta8
    tf_fixed_to_end_effector, tf_list = FKinDHParam(config, DHParam)

    # Append points to the list
    t_ef = tf_fixed_to_end_effector

    # Find jacobian from the transformations
    tf_list.insert(0, eye(4))
    jacobian = []
    # J[i] = [Jv{3x1} Jw{3x1}] {6x1} --> each column
    for i in range(1, len(DHParam) + 1):
        joint = 'J' + str(i)

        # if DHParam[joint].type == 'R':

        # J linear velocity component for the revolute joint is given as \
        # Zi
        ji_angular = np.squeeze(tf_list[i][:3, 2])

        # All w.r.t to fixed frame i.e., Zi-1 to {0} frame, Oi-1, On etc
        zi_1 = np.squeeze(tf_list[i - 1][:3, 2])
        oi_1 = np.squeeze(tf_list[i - 1][:3, 3])
        on = np.squeeze(tf_list[len(tf_list) - 1][:3, 3])

        o_diff = on - oi_1

        # J linear velocity component for the revolute joint is given as
        # Zi-1 x (On - Oi-1)
        ji_linear = np.cross(zi_1, o_diff)

        # Concatenate linear and angular velocity components
        ji = np.concatenate((ji_angular, ji_linear), axis=0)
        jacobian.append(ji)

    # Take transpose to match dimensions as [6xn]
    jacobian = np.array(jacobian, dtype=np.float32).T

    return jacobian, t_ef