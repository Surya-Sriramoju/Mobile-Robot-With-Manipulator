import numpy as np
np.set_printoptions(suppress=True)
from utils import *
from tqdm import *
import rclpy
from rclpy.logging import get_logger

logger = get_logger(__name__)
# Creating Gradient for the Newton Rapson method
def GetGradient(joints, DHTable):
    delta = 0.0001
    jac = np.zeros((16, joints.shape[0]))
    for i, joint in enumerate(joints):
        joints_m = joints.copy()
        joints_p = joints.copy()
        joints_m[i] -= delta
        joints_p[i] += delta
        Tm ,_ = FKinDHParam(joints_m, DHTable)
        Tp ,_ = FKinDHParam(joints_p, DHTable)
        jac[:, i] = (Tp - Tm).flatten() / (2 * delta)
    # print(jac)
    return jac


# Finding the joint angle using the ik solver
def ik(DHTable, T_tar, thetalist0, tolerance = 1e-17):
    step = 0.5
    joints = thetalist0.copy()
    joint_list = []
    joint_list.append(joints)
    for i in tqdm(range(10000)):
        T_cur,_ = FKinDHParam(joints, DHTable)  
        deltaT = (T_tar - T_cur).flatten()
        error = np.linalg.norm(deltaT)
        if error < tolerance:
            hai = len(joint_list)
            logger.info("The number of joint iterations are: "+str(hai))
            return joint_list, True
        jac = GetGradient(joints, DHTable)
        deltaq = jac.T@np.linalg.pinv((jac@jac.T)) @ deltaT
        joints = joints + step * deltaq
        
        joint_list.append(joints)
    hai = len(joint_list)
    logger.info("The number of joint iterations are: "+str(hai))
    return joint_list, False