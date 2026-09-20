'''
*****************************************************************************************
*
*        =================================================
*             Echo Balancer (EB) Theme (eYRC 2026-27)
*        =================================================
*
*  This script is to implement Task 1A of Echo Balancer (EB) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ 2243 ]
# Author List:      [ Akanksha Singh, Kirtee Mishra, Koshika Verma, Srinidhi Sivakumar ]
# Filename:         Task1A.py
# Functions:        find_equilibrium_points, find_A_B_matrices,
#                   find_eigen_values, compute_lqr_gain
# Global variables: theta, omega, u, theta_dot, omega_dot, STATES

import sympy as sp
import numpy as np
import control


############################################################################
#                             THE SYSTEM                                   #
############################################################################
# The pendulum-with-torque system with one added damping term:
#
#     theta_dot = omega
#     omega_dot = -10*sin(theta) - omega + u
#
#   theta = angle from vertical   omega = angular rate   u = applied torque

# Define the symbolic variables
theta, omega, u = sp.symbols('theta, omega, u')

# Define the differential equations
theta_dot = omega
omega_dot = -10*sp.sin(theta) - omega + u

# Store the order of the states (angle first, then its rate).
STATES = [theta, omega]

############################################################################


def find_equilibrium_points():
    '''
    Purpose:
    ---
    Find every point where the pendulum is not accelerating: switch off the
    input, set both derivatives to zero, and solve for theta and omega
    together.

    Returns:
    ---
    `equi_points`: [ list of tuples ] one (theta, omega) pair per equilibrium

    sp.solve() takes a list of expressions (assumed equal to zero) and
    a list of unknowns. sin(theta) = 0 has infinitely many roots in theory;
    only two are physically distinct.

    Example Call:
    ---
    find_equilibrium_points()
    '''

    # (expression).subs(x,0) will substitue x=0 in the given expression.
    # Substitute the state equations and list of unknowns in sp.solve() function 
    # and store the equilibrium points in 'equi_points'.
    equi_points=sp.solve([omega, (-10*sp.sin(theta) - omega + u).subs(u,0)], [theta,omega])

    return equi_points


def find_A_B_matrices(eq_points):
    '''
    Purpose
    ---
    Linearise the system at every equilibrium point: f.jacobian(STATES)
    gives A and f.jacobian([u]) gives B, where f = [theta_dot, omega_dot].
    At each point, substitute the state values and u = 0.

    Input Arguments:
    ---
    `eq_points`: [ list of tuples ] the points from find_equilibrium_points()

    Returns:
    ---
    `A_matrices`, `B_matrices`: [ lists of sympy Matrix ]

    Note: B does not depend on theta here, so it is the same at both points.

    Example Call:
    ---
    find_A_B_matrices(eq_points)
    '''
    # Store the matrix [(thets_dot, omega_dot)] in f.
    f = sp.Matrix([theta_dot, omega_dot])

    # Define set of A and B matrices for each equilibrium point.
    A_matrices, B_matrices = [], []

    # Calculate the Jacobian matrix A and store it in J_A
    J_A= f.jacobian(STATES)

    # Calculate the Jacobian matrix B and store it in J_B
    J_B= f.jacobian([u])

    # Jacobian matrices at equilibrium point 1
    A1= J_A.subs([(theta, eq_points[0][0]), (omega, eq_points[0][1]), (u, 0)])
    B1= J_B.subs(u, 0)

    # Jacobian matrices at equilibrium point 2
    A2= J_A.subs([(theta, eq_points[1][0]), (omega, eq_points[1][1]), (u, 0)])
    B2= J_B.subs(u, 0)

    # Storing the A and B matrices for each equilibrium point in A_matrices, B_matrices respectively 
    A_matrices= [A1,A2]
    B_matrices= [B1,B2]

    return A_matrices, B_matrices


def find_eigen_values(A_matrices):
    '''
    Purpose:
    ---
    Work out whether the pendulum, left alone, falls back or falls away at
    each equilibrium: call .eigenvals() on each A matrix, then mark it
    'Stable' if every eigenvalue has a strictly negative real part, else
    'Unstable'.

    Input Arguments:
    ---
    `A_matrices`: [ list of sympy Matrix ] from find_A_B_matrices()

    Returns:
    ---
    `eigen_values`: [ list ] one .eigenvals() dict per point
    `stability`:    [ list of str ] 'Stable' or 'Unstable' per point

    sp.re(value) gives the real part. A complex pair with a negative
    real part is still stable; it just oscillates on the way back.
    '''
    # Define the eigen_value and stability matrices. 
    eigen_values = []
    stability = []

    # Iterate through each 'A' matrix in A_matrices.
    for A in A_matrices:

        # Compute the eigen value of matrix A as dictionary keys and store it in eig_dict
        eig_dict = A.eigenvals()

        # Store the computed eigen values as dictionary keys in eigen_vales matrix.
        eigen_values.append(eig_dict)

        # eig_dict.keys() : extracts the eigenvalues from the dictionary that are stored as keys.
        # for val in eig_dict.keys() : loop that iterates through every key in the dictionary and stores it in the variable 'val'
        # Check if the real part of the eigen value is negative
        if all(sp.re(val)<0 for val in eig_dict.keys()):

            # If real part is negative then the system is stable at that point, store "Stable" in stability matrix
            stability.append("Stable")

        # If real part of eigen value is >= 0 then system is unstable, store "Unstable" in stability matrix   
        else:
            stability.append("Unstable")
    

    return eigen_values, stability


def compute_lqr_gain(A_matrices, B_matrices, stability):
    '''
    Purpose:
    ---
    Design the controller for the one equilibrium that needs it: hanging
    down settles back on its own, balanced upright does not.

    Input Arguments:
    ---
    `A_matrices`, `B_matrices`: [ lists of sympy Matrix ]
    `stability`: [ list of str ] from find_eigen_values()

    Returns:
    ---
    `K`: [ numpy array ] the LQR gain, one entry per state

    Steps: pick out the A and B at the unstable equilibrium, convert both to
    float numpy arrays (control.lqr() will not take sympy Matrix objects),
    then call control.lqr(A, B, Q, R) and keep the first of its three
    return values.
    '''
    # Define the Q and R matrices
    Q = np.eye(2)        # State weighting matrix
    R = np.array([1])    # Control weighting matrix

    # Convert every entry of stability into string to compare it easily.
    stability_list= [str(s) for s in stability]

    # Stores the position of Unstable equilibrium point.
    idx= stability_list.index("Unstable")

    # Convert the A matrix at the unstable eq_point into numpy array with data type as 64-bit float entries.
    A= np.array(A_matrices[idx], dtype= np.float64)

    # Convert the B matrix at the unstable eq_point into numpy array with data type as 64-bit float entries.
    B= np.array(B_matrices[idx], dtype= np.float64)

    # Solve the LQR problem and return gain.
    K, _, _ = control.lqr(A, B, Q, R)

    # K = (m,n) , m columns for number of inputs, n rows for number of states. K[0] gives the first row as 1D array.
    K = K[0]

    return K


def main_function():    # Don't change anything in this function
    eq_points = find_equilibrium_points()

    if not eq_points:
        print("No equilibrium points found.")
        return None, None, None, None, None, None

    A_matrices, B_matrices = find_A_B_matrices(eq_points)
    eigen_values, stability = find_eigen_values(A_matrices)
    K = compute_lqr_gain(A_matrices, B_matrices, stability)

    return eq_points, A_matrices, B_matrices, eigen_values, stability, K


def task1a_output(eq_points, A_matrices, B_matrices, eigen_values, stability, K):
    '''
    This function prints the results you have obtained.
    '''
    print("Equilibrium Points:")
    for i, point in enumerate(eq_points):
        print(f"  Point {i + 1}: theta = {point[0]}, omega = {point[1]}")

    print("\nA Matrices at Equilibrium Points:")
    for i, matrix in enumerate(A_matrices):
        print(f"  At Point {i + 1}:")
        print(sp.pretty(matrix, use_unicode=False))

    print("\nB Matrices at Equilibrium Points:")
    for i, matrix in enumerate(B_matrices):
        print(f"  At Point {i + 1}: {sp.Matrix(matrix).T.tolist()[0]}")

    print("\nEigenvalues at Equilibrium Points:")
    for i, eigvals in enumerate(eigen_values):
        eigvals_str = ', '.join([f"{val}: {count}" for val, count in eigvals.items()])
        print(f"  At Point {i + 1}: {eigvals_str}")

    print("\nStability of Equilibrium Points:")
    for i, status in enumerate(stability):
        print(f"  At Point {i + 1}: {status}")

    print("\nLQR Gain Matrix K at the unstable Equilibrium Point:")
    print(K)


if __name__ == "__main__":
    results = main_function()
    eq_points, A_matrices, B_matrices, eigen_values, stability, K = results
    task1a_output(eq_points, A_matrices, B_matrices, eigen_values, stability, K)
