import rclpy
import numpy as np
from numpy import cos, sin
from sympy import eye
from std_msgs.msg import Float64
from params import DHParam
from scipy.interpolate import CubicSpline
import numpy as np

np.set_printoptions(suppress=True)


def FKinDHParam(config, DHParam):
    tf_base_list = []
    tf_link_list = []
    tf_fixed_to_end_effector = np.eye(4)
    for i in range(1, len(DHParam) + 1):
        joint = 'J' + str(i)
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
        tf_fixed_to_end_effector = tf_fixed_to_end_effector @ tf
        tf_link_list.append(tf)
        tf_base_list.append(tf_fixed_to_end_effector)
    return tf_fixed_to_end_effector, tf_base_list


def get_jacobian(config, DHParam):
    tf_fixed_to_end_effector, tf_list = FKinDHParam(config, DHParam)
    t_ef = tf_fixed_to_end_effector
    tf_list.insert(0, eye(4))
    jacobian = []
    for i in range(1, len(DHParam) + 1):
        joint = 'J' + str(i)
        ji_angular = np.squeeze(tf_list[i][:3, 2])
        zi_1 = np.squeeze(tf_list[i - 1][:3, 2])
        oi_1 = np.squeeze(tf_list[i - 1][:3, 3])
        on = np.squeeze(tf_list[len(tf_list) - 1][:3, 3])
        o_diff = on - oi_1
        ji_linear = np.cross(zi_1, o_diff)
        ji = np.concatenate((ji_angular, ji_linear), axis=0)
        jacobian.append(ji)
    jacobian = np.array(jacobian, dtype=np.float32).T
    return jacobian, t_ef

def smooth(joint_angles):
    joint_angles = np.array(joint_angles)
    time_steps = np.linspace(0, 1, len(joint_angles))
    spline_functions = [CubicSpline(time_steps, joint_angles[:, i], bc_type='natural') for i in range(joint_angles.shape[1])]
    num_interpolation_points = 200
    interpolation_time_steps = np.linspace(0, 1, num_interpolation_points)
    interpolated_joint_angles = np.array([spline_function(interpolation_time_steps) for spline_function in spline_functions]).T
    return interpolated_joint_angles
