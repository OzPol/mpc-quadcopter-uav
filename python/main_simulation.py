import os
import numpy as np
import scipy.signal as signal
import cvxpy as cp

# Import our modular classes
from trajectory import TrajectoryGenerator
from animation import run_animation
from plotter import generate_scientific_plots

# ==========================================
# 1. PARAMETER LIBRARY
# ==========================================
ROBOT_CONFIGS = {
    'crazyflie_2_1': {
        'm': 0.027, 'L': 0.046, 
        'Ixx': 2.3951e-5, 'Iyy': 2.3951e-5, 'Izz': 3.2347e-5, 'F_max': 0.15
    },
    'standard_uav': {
        'm': 1.2, 'L': 0.3, 
        'Ixx': 0.015, 'Iyy': 0.015, 'Izz': 0.025, 'F_max': 7.0
    }
}

# The Explicit List Version (for targeted experiments):
# --- EXPERIMENT SETUP ---
# Define a list of (robot, trajectory) tuples
experiment_queue = [
    ('standard_uav', 'circle'),
    ('standard_uav', 'lemniscate'),
    ('standard_uav', 'helix'),
    ('crazyflie_2_1', 'circle'),
    ('crazyflie_2_1', 'lemniscate'),
    ('crazyflie_2_1', 'helix'),
    # Just comment out the lines to skip!
]

"""
    # Full Grid Search Version (for full combinatorial testing):
        # --- EXPERIMENT SETUP ---
        robots = ['standard_uav', 'crazyflie_2_1']
        trajectories = ['circle', 'helix', 'lemniscate']
        # Automatically creates every possible combination
        experiment_queue = list(itertools.product(robots, trajectories))
"""


for active_robot, active_trajectory in experiment_queue:
    print(f"\n==================================================")
    print(f"STARTING EXPERIMENT: {active_robot} | {active_trajectory}")
    print(f"==================================================")

    params = ROBOT_CONFIGS[active_robot]
    g = 9.81
    m, L = params['m'], params['L']
    Ixx, Iyy, Izz = params['Ixx'], params['Iyy'], params['Izz']
    F_max = params['F_max']

    # Dynamic Output Setup
    prefix = f"{active_robot}_{active_trajectory}"
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'output')
    os.makedirs(output_dir, exist_ok=True)

    # ==========================================
    # 2. STATE-SPACE FORMULATION
    # ==========================================
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

    Ts = 0.1
    sys_c = signal.StateSpace(A, B, np.eye(12), np.zeros((12, 4)))
    sys_d = sys_c.to_discrete(Ts)
    Ad, Bd = sys_d.A, sys_d.B

    # ==========================================
    # 3. MPC SETUP & CONSTRAINTS
    # ==========================================
    Np = 20
    Q = np.diag([10, 10, 10, 0, 0, 0, 1, 1, 10, 0, 0, 0])
    R = np.diag([0.1, 0.1, 0.1, 0.1])

    max_total_thrust_delta = (F_max * 4) - (m * g) 
    max_tau_xy = F_max * L    
    max_tau_z = F_max * 0.05  

    u_max = np.array([max_total_thrust_delta, max_tau_xy, max_tau_xy, max_tau_z])
    u_min = np.array([-max_total_thrust_delta, -max_tau_xy, -max_tau_xy, -max_tau_z])

    # ==========================================
    # 4. SIMULATION INITIALIZATION
    # ==========================================
    T_total = 25
    steps = int(T_total / Ts)

    x = np.zeros(12)
    u_prev = np.zeros(4)

    x_log = np.zeros((12, steps))
    u_log = np.zeros((4, steps))
    r_log = np.zeros((12, steps))

    # Instantiate trajectory class
    traj_gen = TrajectoryGenerator(radius=3.0, height=5.0, omega=0.4, scale_lemniscate=4.0)

    # ==========================================
    # 5. MAIN CONTROL LOOP
    # ==========================================
    print(f"Executing MPC Loop for {prefix}...")
    for k in range(steps):
        t_current = k * Ts
        t_horizon = t_current + np.arange(Np + 1) * Ts
        
        # Get dynamic reference path from class
        r_horizon = traj_gen.get_reference(t_horizon, mode=active_trajectory)
        
        x_var = cp.Variable((12, Np + 1))
        u_var = cp.Variable((4, Np))
        
        cost = 0
        constraints = [x_var[:, 0] == x]
        
        for i in range(Np):
            cost += cp.quad_form(x_var[:, i] - r_horizon[i, :], Q)
            if i == 0:
                cost += cp.quad_form(u_var[:, i] - u_prev, R)
            else:
                cost += cp.quad_form(u_var[:, i] - u_var[:, i-1], R)
                
            constraints += [x_var[:, i+1] == Ad @ x_var[:, i] + Bd @ u_var[:, i]]
            constraints += [u_var[:, i] <= u_max, u_var[:, i] >= u_min]
            
        cost += cp.quad_form(x_var[:, Np] - r_horizon[Np, :], Q)
        
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, warm_start=True)
        
        if u_var.value is None:
            print(f"Optimization failed to converge at step {k}")
            break
            
        u = u_var[:, 0].value
        x = Ad @ x + Bd @ u
        u_prev = u
        
        x_log[:, k] = x
        u_log[:, k] = u
        r_log[:, k] = r_horizon[0, :]

    print("Simulation Complete.")

    # ==========================================
    # 6. OUTPUT GENERATION
    # ==========================================
    # 1. Scientific Metrics
    generate_scientific_plots(x_log, r_log, u_log, Ts, output_dir, prefix)

    # 2. 3D Animation
    run_animation(x_log, r_log, Ts, L, output_dir, prefix)
