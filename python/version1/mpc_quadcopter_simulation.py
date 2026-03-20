import numpy as np
import scipy.signal as signal
import cvxpy as cp
from trajectory import generate_trajectory
from animation import run_animation

# 1. System Parameters

g = 9.81
m = 1.2
Ixx = 0.015
Iyy = 0.015
Izz = 0.025

# 2. Linearized State-Space Formulation
# State: x = [x, y, z, u, v, w, phi, theta, psi, p, q, r]^T
A = np.zeros((12, 12))
A[0:3, 3:6] = np.eye(3)
A[6:9, 9:12] = np.eye(3)
A[3, 7] = -g
A[4, 6] = g

B = np.zeros((12, 4))
B[5, 0]  = 1/m
B[9, 1]  = 1/Ixx
B[10, 2] = 1/Iyy
B[11, 3] = 1/Izz

# Discretize using Scipy (Zero-Order Hold)
Ts = 0.1
sys_c = signal.StateSpace(A, B, np.eye(12), np.zeros((12, 4)))
sys_d = sys_c.to_discrete(Ts)
Ad, Bd = sys_d.A, sys_d.B

# 3. MPC Setup
Np = 20
Q = np.diag([10, 10, 10, 0, 0, 0, 1, 1, 10, 0, 0, 0])
R = np.diag([0.1, 0.1, 0.1, 0.1])

u_max = np.array([m*g*0.5, 0.5, 0.5, 0.5])
u_min = np.array([-m*g*0.5, -0.5, -0.5, -0.5])

# 4. Simulation Initialization
T_total = 25
steps = int(T_total / Ts)

x = np.zeros(12)
u_prev = np.zeros(4)

x_log = np.zeros((12, steps))
u_log = np.zeros((4, steps))
r_log = np.zeros((12, steps))

radius, height, omega = 3.0, 5.0, 0.4

# 5. Main Control Loop
print("Executing CVXPY Model Predictive Control Loop...")
for k in range(steps):
    t_current = k * Ts
    t_horizon = t_current + np.arange(Np + 1) * Ts
    r_horizon = generate_trajectory(t_horizon, radius, height, omega)
    
    # Define CVXPY Optimization Variables
    x_var = cp.Variable((12, Np + 1))
    u_var = cp.Variable((4, Np))
    
    cost = 0
    constraints = [x_var[:, 0] == x]
    
    # Build Cost and Constraints over Prediction Horizon
    for i in range(Np):
        cost += cp.quad_form(x_var[:, i] - r_horizon[i, :], Q)
        
        # Penalize control increments (ManipulatedVariablesRate equivalent)
        if i == 0:
            cost += cp.quad_form(u_var[:, i] - u_prev, R)
        else:
            cost += cp.quad_form(u_var[:, i] - u_var[:, i-1], R)
            
        constraints += [x_var[:, i+1] == Ad @ x_var[:, i] + Bd @ u_var[:, i]]
        constraints += [u_var[:, i] <= u_max, u_var[:, i] >= u_min]
        
    cost += cp.quad_form(x_var[:, Np] - r_horizon[Np, :], Q)
    
    # Solve the Quadratic Program
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, warm_start=True)
    
    if u_var.value is None:
        print(f"Optimization failed to converge at step {k}")
        break
        
    # Apply optimal control input to plant
    u = u_var[:, 0].value
    x = Ad @ x + Bd @ u
    u_prev = u
    
    # Log data
    x_log[:, k] = x
    u_log[:, k] = u
    r_log[:, k] = r_horizon[0, :]

print("Simulation Complete.")

# 6. Run Visualization
run_animation(x_log, r_log, Ts)