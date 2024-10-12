# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.ปานศิริ_6538
2.พรพิพัฒน์_6571
'''
# FRA333_HW3_xx_xx.py
import numpy as np
from HW3_utils import FKHW3
import roboticstoolbox as rtb
from spatialmath import SE3
import math
from math import pi
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#      From external function       
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#=============================================<คำตอบข้อ 1>======================================================#
# ฟังก์ชันในการคำนวณ Jacobian
# #Input:
# q: มุมข้อต่อ (joint angles) ซึ่งเป็นลิสต์หรือนัมพายอาเรย์ของมุมข้อต่อ (3 ข้อต่อ) ที่ใช้ในการคำนวณ
# Output:
# J_e: แมทริกซ์ Jacobian 6x3 ซึ่งประกอบด้วยส่วนเชิงเส้น (linear velocity) และส่วนเชิงมุม (angular velocity) ของ end-effector
def endEffectorJacobianHW3(q):
    R, P, R_e, p_e = FKHW3(q)
    
    Jv = np.zeros((3, 3))  # ส่วนเชิงเส้น (linear velocity)
    Jw = np.zeros((3, 3))  # ส่วนเชิงมุม (angular velocity)

    # คำนวณ Jacobian สำหรับทุกข้อต่อ
    for i in range(3):
        # Get the rotation matrix and position of each joint
        R_i = R[:, :, i]
        P_i = P[:, i]
        
        # Calculate linear velocity Jacobian
        z = R_i @ np.array([0, 0, 1])  # Axis of rotation for each joint
        Jv[:, i] = np.cross(z, (p_e - P_i))  # Cross product
        
        # Calculate angular velocity Jacobian
        Jw[:, i] = z

    # รวมเป็น Jacobian 6x3
    J_e = np.vstack([Jv, Jw])
    return J_e
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
# ฟังก์ชันในการตรวจสอบสภาวะ Singularities
# Input:
# q: มุมข้อต่อ (joint angles) ซึ่งเป็นลิสต์หรือนัมพายอาเรย์ของมุมข้อต่อ (3 ข้อต่อ)
# epsilon: ค่าเกณฑ์ที่ใช้ในการตรวจสอบสภาวะ Singularities (ค่าเริ่มต้นคือ 0.001)
# Output:
# 1: ถ้าหุ่นยนต์อยู่ในสภาวะ Singularities (determinant ของ Jacobian ส่วนเชิงเส้นมีค่าน้อยกว่า epsilon)
# 0: ถ้าไม่อยู่ในสภาวะ Singularities
def checkSingularityHW3(q, epsilon=0.001):
    J_e = endEffectorJacobianHW3(q)
    Jv = J_e[:3, :]  # ส่วนเชิงเส้นของ Jacobian
    det_Jv = np.linalg.det(Jv)
    return 1 if abs(det_Jv) < epsilon else 0
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
# ฟังก์ชันในการคำนวณ Effort ของข้อต่อ
# Input:
# q: มุมข้อต่อ (joint angles) ซึ่งเป็นลิสต์หรือนัมพายอาเรย์ของมุมข้อต่อ (3 ข้อต่อ)
# w: แรงที่กระทำกับ end-effector ในรูปแบบของเวกเตอร์แรงขนาด 6 มิติ (3 มิติสำหรับแรง และ 3 มิติสำหรับโมเมนต์)
# Output:
# tau: แรงบิดที่แต่ละข้อต่อได้รับในรูปแบบของนัมพายอาเรย์
def computeEffortHW3(q, w):
    J_e = endEffectorJacobianHW3(q)
    tau = np.dot(J_e.T, w)  # คำนวณ torque
    return tau
#==============================================================================================================#
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#           From roboticstoolbox 
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# ฟังก์ชัน create_robot
# Input: ไม่มี
# Output:
# robot: โมเดลหุ่นยนต์ (DHRobot) ที่ถูกสร้างขึ้นตามพารามิเตอร์ DH
def create_robot():
    d_1 = 0.0892
    a_2 = -0.425
    a_3 = -0.39243
    d_4 = 0.109
    d_5 = 0.093
    d_6 = 0.082
    L1 = rtb.RevoluteMDH(d_1, offset=pi)
    L2 = rtb.RevoluteMDH(alpha=pi/2)
    L3 = rtb.RevoluteMDH(a=a_2)
    
    # Define the tool transformation matrix
    tool_transformation = (
        SE3.Tx(a_3) @ SE3.Tz(d_4) @ SE3.Rx(np.pi/2) @
        SE3.Tz(d_5) @ SE3.Rz(-np.pi/2) @ SE3.Rx(-np.pi/2) @ SE3.Tz(-d_6)
    )
    
    # Create the robot model and pass the tool transformation
    robot = rtb.DHRobot([L1, L2, L3], tool=tool_transformation, name='RRR_Robot')
    
    return robot
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# Compute the joint torques given a wrench
# Input:
# robot: โมเดลหุ่นยนต์ที่สร้างขึ้นจากฟังก์ชัน create_robot()
# q: มุมข้อต่อ (joint angles) ของหุ่นยนต์
# wrench: แรงและโมเมนต์ที่กระทำกับ end-effector ในรูปของเวกเตอร์ขนาด 6 มิติ
# Output:
# tau: แรงบิดที่แต่ละข้อต่อได้รับในรูปแบบของนัมพายอาเรย์
def compute_torques(robot, q, wrench):
    """Calculate joint torques from the applied wrench."""
    J_e = robot.jacob0(q)  # Calculate the Jacobian at the given joint angles q
    tau = J_e.T @ wrench   # Calculate the joint torques using the Jacobian transpose method
    return tau
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# Check for singularity in the robot configuration
# Input:
# robot: โมเดลหุ่นยนต์ที่สร้างขึ้น
# q: มุมข้อต่อ (joint angles)
# Output:
# True: ถ้าหุ่นยนต์อยู่ในสภาวะ Singularities
# False: ถ้าหุ่นยนต์ไม่อยู่ในสภาวะ Singularities
def check_singularity(robot, q):
    """Check if the robot is in a singular configuration."""
    J_e = robot.jacob0(q)  # Get the full Jacobian matrix
    Jv = J_e[:3, :]        # Extract the linear velocity part (top 3 rows)
    
    # Check if the Jacobian has full rank (for 3DOF, the rank should be 3)
    rank = np.linalg.matrix_rank(Jv)
    if rank < 3:
        print("Singularity detected! The robot is in a singular configuration.")
        return True
    else:
        print("No singularity detected.")
        return False
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# Function to compute the Jacobian and check forward kinematics
# Input:
# q: มุมข้อต่อ (joint angles)
# Output:
# T: Transformation matrix ของ end-effector ที่ได้จากการคำนวณ Forward Kinematics
# J: Jacobian matrix ของ end-effector
def check_q_and_jacobian(q):
    robot = create_robot()  # Create the robot from DH Parameters
    T = robot.fkine(q)      # Compute forward kinematics for given q
    J = robot.jacob0(q)     # Compute the Jacobian matrix for given q
    return T, J
#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# Function to compare the two Jacobians
# Input:
# J1: Jacobian matrix ตัวที่ 1
# J2: Jacobian matrix ตัวที่ 2
# tolerance: ค่าความคลาดเคลื่อนที่ยอมรับได้สำหรับการเปรียบเทียบ
# Output:
# True: ถ้า Jacobian ทั้งสองเหมือนกันภายในค่าคลาดเคลื่อนที่กำหนด
# False: ถ้า Jacobian ทั้งสองไม่เหมือนกัน
def compare_jacobians(J1, J2, tolerance=0.01):
    if J1.shape != J2.shape:
        print("Jacobian matrices have different shapes and cannot be compared.")
        return False
    
    # Check if all elements are within the given tolerance
    if np.allclose(J1, J2, atol=tolerance):
        print("The Jacobian matrices are the same within the tolerance of ±0.01.")
        return True
    else:
        print("The Jacobian matrices are not the same.")
        return False