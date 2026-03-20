function ref_matrix = Trajectory(t_vec, radius, height, omega)
    % TRAJECTORY Generates the reference state matrix for the MPC horizon
    % Inputs:
    %   t_vec  - Column vector of time steps over the prediction horizon
    %   radius - Radius of the circular path (m)
    %   height - Commanded z-altitude (m)
    %   omega  - Angular velocity around the circle (rad/s)
    % Output:
    %   ref_matrix - Np x 12 matrix of reference states
    
    Np = length(t_vec);
    ref_matrix = zeros(Np, 12);
    
    % Spatial coordinates (x, y, z)
    ref_matrix(:, 1) = radius * cos(omega * t_vec);
    ref_matrix(:, 2) = radius * sin(omega * t_vec);
    ref_matrix(:, 3) = height * ones(Np, 1);
    
    % Yaw tracking (psi) - maintain nose tangent to the trajectory path
    ref_matrix(:, 9) = omega * t_vec + pi/2; 
    
    % All other states (velocities, roll, pitch) 
    % default to 0 in the reference
end