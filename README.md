
  

# FRA333_HW3: Jacobian, Singularity, and Torque Calculation

  

This project provides two solutions for calculating and comparing the **Jacobian matrix**, checking for **singularities**, and calculating the joint **torques (τ)** for a robot. The solutions involve using both an **external custom implementation** and the **`roboticstoolbox`** library.

  

## Project Structure

  

-  **FRA333_HW3_38_71.py**: Contains the custom implementation for calculating the Jacobian, checking for singularities, and computing the torques.

-  **testScript.py**: This script tests the functionalities using both the custom solution and `roboticstoolbox`.

  

## Installation

  

Make sure you have the following libraries installed:

  

-  `numpy`

-  `roboticstoolbox`

-  `spatialmath`

-  `scipy`

  

To install the dependencies, run:
  

```bash

pip  install  numpy  roboticstoolbox-python  spatialmath  scipy

```

download this zip folder extract and open folder with VScode


## How to Run the Project

  

1.  **Run the test script:**

  

This script will test the Jacobian calculation, singularity detection, and torque computation using both the custom implementation and `roboticstoolbox`. You can change value in `q_test` , `w_test` in testScript.py

  

```bash

python testScript.py

```

  

2.  **Testing the Jacobian Matrix Calculation**

  

- The function `test_endEffectorJacobianHW3()` in `testScript.py` calculates the Jacobian matrix using:

- Custom implementation from `FRA333_HW3_38_71.py`.

-  `roboticstoolbox`.

- The results from both are compared with a tolerance of ±0.01 to check if the two Jacobian matrices are equivalent.

  

3.  **Checking Singularity**

  

- The function `test_checkSingularityHW3()` checks for singularity using the custom implementation in `FRA333_HW3_38_71.py`.

- The `check_singularity` function in the `roboticstoolbox` also checks if the Jacobian matrix is rank-deficient (indicating a singular configuration).

  

4.  **Calculating Joint Torques (τ)**

  

- The function `test_computeEffortHW3()` calculates the joint torques based on an applied wrench (forces and torques).

- The custom implementation in `FRA333_HW3_38_71.py` is compared with the `pay()` function in `roboticstoolbox`.

  

# Robotic Toolbox and Custom Solution for Jacobian, Singularity Detection, and Torque Calculation

  

This project compares custom solutions for calculating the Jacobian matrix, detecting singularities, and calculating joint torques (τ) with the equivalent capabilities provided by `roboticstoolbox`. Below are the details of both the custom and `roboticstoolbox` solutions.

  

---

  

## Custom Solution Details

  

### 1. Jacobian Matrix Calculation

  

In `FRA333_HW3_38_71.py`, the Jacobian matrix is computed using the forward kinematics of an RRR (Revolute-Revolute-Revolute) robot. The Jacobian matrix can be divided into two main parts:

  

As shown in the image, the Jacobian matrix (\( J \)) is expressed as:

  

![image](https://github.com/user-attachments/assets/d3e33736-8749-45b8-a4b4-4de44050d6bd)

  

![image](https://github.com/user-attachments/assets/30581c9b-73a2-45ba-85cb-e7ba0402ef35)


  

**Formula used in the code:**

  

The Jacobian can be calculated by finding the derivatives of the position and orientation of the end-effector with respect to the joint angles \( q \):

  

![image](https://github.com/user-attachments/assets/2f915a5e-7e75-4294-9810-c25149445dc8)

  

![image](https://github.com/user-attachments/assets/e7e60c5a-2c88-406f-9fe6-19cc2cd2ec84)


  

**Example usage:**

  

```python

J_e = endEffectorJacobianHW3(q_test)

```

  

To compare the Jacobian matrix with the result from `roboticstoolbox`:

  

```python

J_toolbox = robot.jacob0(q_test)

```

  

### 2. Singularity Detection

  

In an RRR robotic system, singularities can be detected by examining the determinant of the Jacobian matrix, particularly the determinant of \( J_v \) (the linear part). If the determinant equals zero, it indicates that the robot is in a singular configuration, meaning it loses its ability to move freely in certain directions.

  

**Formula used for detection:**

  

![image](https://github.com/user-attachments/assets/f8266c74-af3a-492c-b2a7-7e769ab5c3f6)


  

**Example usage:**

  

```python

singularity = checkSingularityHW3(q_test)

```

  

### 3. Torque Calculation (τ)

  

The torque at the robot's joints can be calculated by applying the wrench to the end-effector and using the Jacobian matrix. The calculation uses the transpose of both the linear Jacobian (\( J_v \)) and the angular Jacobian (\( J_w \)) to transform the wrench from the end-effector to the joint torques.

  

**Formula used for torque calculation:**

  

![image](https://github.com/user-attachments/assets/30f3fd08-505c-4dab-b263-cf9a59082759)

  

**Example usage:**

  

```python

tau = computeEffortHW3(q_test, wrench)

```

  

---

  

## Roboticstoolbox Solution Details

  

In `roboticstoolbox`, there are built-in functions for computing the Jacobian matrix, detecting singularities, and calculating joint torques. These functions simplify the process and allow for quick calculations without manually deriving the equations.

  

### 1. Jacobian Calculation

  

The `jacob0(q)` function in `roboticstoolbox` computes the Jacobian matrix, which relates the joint velocities to the linear and angular velocities of the end-effector in the space frame.

  

**Formula used in Roboticstoolbox:**

  

The Jacobian matrix is calculated based on the forward kinematics of the robot, which establishes the relationship between the change in joint angles and the motion of the end-effector:

  
![image](https://github.com/user-attachments/assets/8ae6631e-5f4a-46bd-9d98-2560a08da1f6)


  

**Example:**

  

```python

J = robot.jacob0(q_test)

```

  

### 2. Singularity Detection

  

`roboticstoolbox` provides the `is_singular(q)` function to detect whether the robot is in a singular configuration by evaluate whether the Jacobian matrix loses rank in a particular configuration. If the Jacobian matrix loses rank, the robot enters a singular configuration.

  

**Formula used in Roboticstoolbox:**

  

The function `robot.is_singular(q)` uses the Jacobian matrix calculated by `jacob0(q)` and checks whether. If the determinant is zero, the robot is in a singular configuration.

  

![image](https://github.com/user-attachments/assets/68d266b5-4932-4458-85ac-f606d4a0da73)


  

If the determinant of the Jacobian matrix is zero, the system identifies the robot as being in singularity.

  

**Example:**

  

```python

singularity_toolbox = robot.is_singular(q_test)

```

  

### 3. Torque Calculation (τ)

  

The `pay()` function in `roboticstoolbox` computes the joint torques by taking the wrench applied to the end-effector and using the Jacobian matrix to transform that force into joint torques.

  

**Formula used in Roboticstoolbox:**

  

The function `robot.pay(q, wrench)` uses the Jacobian matrix \( J \) and the force \( F \) to compute the torque (\( au \)) at the joints by using the transpose of the Jacobian matrix:

  

![image](https://github.com/user-attachments/assets/d8598df0-53b8-47f6-891e-a86f03c39511)


  

**Example:**

  

```python

tau_toolbox = robot.pay(q_test, wrench)

```

  

---
  

## Comparison of the Two Solutions

  

-  **Jacobian Comparison**:

- The `compare_jacobians()` function compares the Jacobian matrices from the custom solution and `roboticstoolbox` to ensure they are equivalent within a tolerance of ±0.01.

  

-  **Singularity Detection**:

- Both solutions detect singularities by checking the rank of the Jacobian matrix. If the Jacobian loses rank, the robot is in a singular configuration.

  

-  **Torque Calculation**:

- Both solutions compute the joint torques based on the applied wrench using the Jacobian matrix.

  

## Example Output

  

Here’s a breakdown of the output from running `testScript.py`:

  

###Question 1 - Jacobian Matrix Calculation (External Function vs Roboticstoolbox)

  

#### **External Function:**

  

```

Jacobian Matrix from external function (rounded to 3 decimals):

[[-0.110 -0.093 -0.093]

[ 0.899 0. 0. ]

[ 0. -0.899 -0.474]

[ 0. 0. 0. ]

[ 1. 1. 1. ]

[ 1. 0. 0. ]]

```

  

This matrix is computed using the custom forward kinematics function from `FRA333_HW3_38_71.py`. The first three rows represent the **linear velocity components** of the Jacobian matrix, and the last three rows represent the **angular velocity components**.

  

#### **Roboticstoolbox Function:**

  

```

Jacobian Matrix from roboticstoolbox (rounded to 3 decimals):

[[-0.110 -0.093 -0.093]

[ 0.899 0. 0. ]

[ 0. -0.899 -0.474]

[ 0. 0. 0. ]

[ 1. 1. 1. ]

[ 1. 0. 0. ]]

```

  

This matrix is computed using the `jacob0()` function from the `roboticstoolbox` library. Similar to the custom function, the first three rows represent the **linear velocity**, and the last three rows represent the **angular velocity**.

  

#### **Comparison Result:**

  

```

The Jacobian matrices are the same within the tolerance of ±0.01.

```


  ![image](https://github.com/user-attachments/assets/ba0c2f1f-b80d-4e6b-adbb-ada092082c50)


This confirms that the two methods of calculating the Jacobian matrix yield results that are equivalent, with differences that fall within the specified tolerance of ±0.01.



---

  

### Question 2 - Singularity Check

  

#### **External Function:**

  

```

Singularity Status from external function (1=Yes, 0=No): 0

No singularity detected.

```

  

In this case, the custom singularity check reports that the robot is **not in a singular configuration** (`0` means no singularity). The check evaluates the determinant of the Jacobian matrix's linear velocity part (first three rows) to see if the robot's configuration leads to any loss of mobility.

  

#### **Roboticstoolbox Function:**

  

```

Singularity Status from roboticstoolbox (True=Yes, False=No): False

```

  
![image](https://github.com/user-attachments/assets/462ec039-b4f5-4d44-827d-6a661b3e5fd4)

Similarly, the `check_singularity` function from `roboticstoolbox` confirms that there is **no singularity** in the robot's configuration (`False` means no singularity detected).

  

---

  

### Question 3 - Joint Torque (τ) Calculation from Applied Wrench

  

#### **External Function:**

  

```

Effort on each joint from external function (rounded to 3 decimals):

[ 3.91 -0.93 -0.93]

```

  

This shows the torques applied to the robot's joints based on an applied wrench (force and torque at the end-effector). The custom function calculates these values using the transpose of the Jacobian matrix and the applied wrench vector.

  

#### **Roboticstoolbox Function:**

  

```

Effort on each joint from roboticstoolbox (rounded to 3 decimals):

[ 3.91 -0.93 -0.93]

```
![image](https://github.com/user-attachments/assets/750fd2d0-94cd-47ea-a606-f9dc57f1bb68)

  

The result is identical to the custom function, as `roboticstoolbox` uses the same method (Jacobian transpose) to compute the joint torques. The consistency between the two methods confirms that both approaches are correct.

![image](https://github.com/user-attachments/assets/67b43595-4e37-4edd-9965-ee6fcdffb760)
---

 
