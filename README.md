
  

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

  

## How to Run the Project

  

1.  **Run the test script:**

  

This script will test the Jacobian calculation, singularity detection, and torque computation using both the custom implementation and `roboticstoolbox`.

  

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

  

## Custom Solution Details

  

### 1. Jacobian Matrix Calculation

  

In `FRA333_HW3_38_71.py`, the Jacobian matrix is computed using the robot’s forward kinematics, and the result is compared with the output from `roboticstoolbox`. The `compare_jacobians()` function checks if the results from both solutions are within a tolerance of ±0.01.

  

**Example:**

  

```python

J_e = endEffectorJacobianHW3(q_test)

```

  

### 2. Singularity Detection

  

The `checkSingularityHW3()` function checks if the Jacobian matrix is singular by evaluating its determinant. This is compared with the singularity detection in `roboticstoolbox`.

  

**Example:**

  

```python

singularity = checkSingularityHW3(q_test)

```

  

### 3. Torque Calculation (τ)

  

The `computeEffortHW3()` function calculates the joint torques based on the wrench applied at the end-effector.

  

**Example:**

  

```python

tau = computeEffortHW3(q_test, wrench)

```

  

## Roboticstoolbox Solution Details

  

The `roboticstoolbox` solution provides the following capabilities:

  

1.  **Jacobian Calculation:**

  

Use the `jacob0()` method from `roboticstoolbox` to compute the Jacobian matrix for the robot.

  

**Example:**

  

```python

J = robot.jacob0(q_test)

```

  

2.  **Singularity Detection:**

  

Use the custom `check_singularity()` function in `roboticstoolbox` to check if the robot is in a singular configuration.

  

3.  **Torque Calculation:**

  

Use the `pay()` method in `roboticstoolbox` to compute the joint torques from the applied wrench.

  

**Example:**

  

```python

tau = robot.pay(q_test, wrench)

```

  

## Comparison of the Two Solutions

  

-  **Jacobian Comparison**:

- The `compare_jacobians()` function compares the Jacobian matrices from the custom solution and `roboticstoolbox` to ensure they are equivalent within a tolerance of ±0.01.

  

-  **Singularity Detection**:

- Both solutions detect singularities by checking the rank of the Jacobian matrix. If the Jacobian loses rank, the robot is in a singular configuration.

  

-  **Torque Calculation**:

- Both solutions compute the joint torques based on the applied wrench using the Jacobian matrix.

  

## Example Output

  

Here’s a breakdown of the output from running `testScript.py`:

  

### 1. Jacobian Matrix Calculation (External Function vs Roboticstoolbox)

  

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

  

### 2. Singularity Check

  

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

  

### 3. Joint Torque (τ) Calculation from Applied Wrench

  

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

 
