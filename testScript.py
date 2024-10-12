import numpy as np
from FRA333_HW3_38_71 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3,create_robot,compute_torques,check_singularity,check_q_and_jacobian,compare_jacobians
import roboticstoolbox as rtb
from spatialmath import SE3
import math
from math import pi
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
# Function to print a matrix with values rounded to 3 decimal places
def print_matrix_rounded(matrix):
    np.set_printoptions(precision=3, suppress=True)
    print(matrix)

# Test the end-effector Jacobian and compare
def test_endEffectorJacobianHW3():
    q_test = [0.0, 0.0, 0.0]
    
    # Jacobian from external function
    J_e = endEffectorJacobianHW3(q_test)
    print("Jacobian Matrix from external function (rounded to 3 decimals):")
    print_matrix_rounded(J_e)
    
    # Jacobian from roboticstoolbox
    T, J_rtb = check_q_and_jacobian(q_test)
    print("Jacobian Matrix from roboticstoolbox (rounded to 3 decimals):")
    print_matrix_rounded(J_rtb)
    
    # Compare both Jacobians
    compare_jacobians(J_e, J_rtb)
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
# Create a robot using DH Parameters

# Test the singularity check
def test_checkSingularityHW3():
    q_test = [0.0, 0.0, 0.0]
    
    # Singularity check using custom function
    singularity = checkSingularityHW3(q_test)
    print("Singularity Status from external function (1=Yes, 0=No):", singularity)
    
    # Singularity check using roboticstoolbox
    robot = create_robot()
    is_singular = check_singularity(robot, q_test)
    print(f"Singularity Status from roboticstoolbox (True=Yes, False=No): {is_singular}")
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
# Test the effort calculation
def test_computeEffortHW3():
    q_test = [0.0, 0.0, 0.0]
    w_test = [10, 0, 0, 0, 0, 5]  # Example wrench
    
    # Compute effort using external function
    tau_external = computeEffortHW3(q_test, w_test)
    print("Effort on each joint from external function (rounded to 3 decimals):")
    print_matrix_rounded(tau_external)
    
    # Compute effort using roboticstoolbox
    robot = create_robot()
    tau_roboticstoolbox = compute_torques(robot, q_test, w_test)
    print("Effort on each joint from roboticstoolbox (rounded to 3 decimals):")
    print_matrix_rounded(tau_roboticstoolbox)
#==============================================================================================================#
# Call test functions
if __name__ == "__main__":
    test_endEffectorJacobianHW3()
    test_checkSingularityHW3()
    test_computeEffortHW3()