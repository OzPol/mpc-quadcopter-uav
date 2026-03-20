% mpc_quadcopter_simulation.m
% -------------------------------------------------------------------------
% Description: Model Predictive Control for a Quadcopter UAV
% This script explicitly constructs the $A$ and $B$ matrices using 
% the exact 12-state vector 
% $\mathbf{x} = [x, y, z, u, v, w, \phi, \theta, \psi, p, q, r]^T$
%
% Architecture: Linearized 12-state State-Space Model with Receding Horizon
% -------------------------------------------------------------------------

clear; clc; close all;

%% 1. System Parameters
% Physical constants of the UAV
g = 9.81;       % Gravity (m/s^2)
m = 1.2;        % Mass (kg)
Ixx = 0.015;    % Roll moment of inertia (kg*m^2)
Iyy = 0.015;    % Pitch moment of inertia (kg*m^2)
Izz = 0.025;    % Yaw moment of inertia (kg*m^2)

%% 2. Linearized State-Space Formulation
% State vector x = [x, y, z, u, v, w, phi, theta, psi, p, q, r]'
% Input vector u = [Delta_T, tau_phi, tau_theta, tau_psi]'

% Initialize continuous-time system matrix A (12x12)
A = zeros(12, 12);
% Kinematics: derivative of position is linear velocity
A(1:3, 4:6) = eye(3); 
% Kinematics: derivative of Euler angles is angular velocity (hover approx)
A(7:9, 10:12) = eye(3);
% Dynamics: gravity coupling on linear acceleration from roll/pitch
A(4, 8) = -g;  % dot_u = -g * theta
A(5, 7) = g;   % dot_v =  g * phi

% Initialize continuous-time input matrix B (12x4)
B = zeros(12, 4);
% Dynamics: effect of control inputs on accelerations
B(6, 1)  = 1/m;    % dot_w = Delta_T / m
B(10, 2) = 1/Ixx;  % dot_p = tau_phi / Ixx
B(11, 3) = 1/Iyy;  % dot_q = tau_theta / Iyy
B(12, 4) = 1/Izz;  % dot_r = tau_psi / Izz

% Output matrix C (Assuming full state observability) and D
C = eye(12);
D = zeros(12, 4);

% Create continuous-time state-space system
sys_c = ss(A, B, C, D);

% Discretize system using Zero-Order Hold (ZOH)
Ts = 0.1; % Sampling time (seconds)
sys_d = c2d(sys_c, Ts, 'zoh');

%% 3. Model Predictive Control (MPC) Initialization
% Prediction and Control Horizons
Np = 20; % N_p: Prediction horizon
Nc = 5;  % N_c: Control horizon

% Initialize MPC Object
mpcobj = mpc(sys_d, Ts, Np, Nc);

% Define State Weighting Matrix (Q) via MPC Toolbox Outputs
% Prioritizing tracking precision for 
%   x, y, z (states 1-3) and yaw(state 9)
% Q-matrix equivalent weights:
mpcobj.Weights.OutputVariables = [10 10 10  0 0 0  1 1 10  0 0 0];

% Define Control Increment Weighting Matrix (R)
% Penalizes aggressive control actions to maintain smooth flight
mpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];

% Actuator Constraints (Physical limits on inputs)
mpcobj.MV(1).Min = -m*g * 0.5; % Minimum thrust decrement
mpcobj.MV(1).Max = m*g * 0.5;  % Maximum thrust increment
for i = 2:4
    mpcobj.MV(i).Min = -0.5;   % Torque limits
    mpcobj.MV(i).Max = 0.5;
end

%% 4. Closed-Loop Simulation Setup
T_total = 25;           % Total simulation duration (s)
steps = T_total / Ts;   % Total discrete time steps

% Pre-allocate arrays for data logging
x = zeros(12, 1);       % Initial state (hovering at origin)
x_log = zeros(12, steps);
u_log = zeros(4, steps);
r_log = zeros(12, steps); % Reference logging

% Initialize MPC state
mpcstate_obj = mpcstate(mpcobj);

% Trajectory geometry
radius = 3.0; 
height = 5.0; 
omega = 0.4;  % Angular speed

%% 5. Main Control Loop (Receding Horizon)
disp('Executing MPC Closed-Loop Simulation...');
for k = 1:steps
    t_current = (k-1)*Ts;
    
    % Generate reference trajectory over the prediction horizon Np
    t_horizon = t_current + (0:Np-1)' * Ts;
    r_horizon = Trajectory(t_horizon, radius, height, omega);
    
    % Compute optimal control sequence and apply the first element (u_t)
    u = mpcmove(mpcobj, mpcstate_obj, x, r_horizon);
    
    % Plant update: Propagate dynamics forward in time
    x = sys_d.A * x + sys_d.B * u;
    
    % Log states, inputs, and reference
    x_log(:, k) = x;
    u_log(:, k) = u;
    r_log(:, k) = r_horizon(1, :)';
end
disp('Simulation Successfully Completed.');

%% 6. Visualization
Animation(x_log, r_log, Ts);