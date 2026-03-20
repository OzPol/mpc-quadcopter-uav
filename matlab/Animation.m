function Animation(x_log, r_log, Ts)
    % ANIMATION Renders the 3D flight path and UAV orientation
    % Inputs:
    %   x_log - 12xN array of logged state data
    %   r_log - 12xN array of logged reference data
    %   Ts    - Sampling time
    
    figure('Name', 'UAV Quadcopter Dynamics & MPC', 'Color', 'w');
    
    % Extract Kinematic States from the 12-state vector
    X = x_log(1, :); Y = x_log(2, :); Z = x_log(3, :);
    Phi = x_log(7, :); Theta = x_log(8, :); Psi = x_log(9, :);
    
    % Setup 3D Canvas
    plot3(r_log(1,:), r_log(2,:), r_log(3,:), 'k--', 'LineWidth', 1.5);
    hold on; grid on;
    axis equal;
    axis([-5 5 -5 5 0 8]);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('UAV MPC Trajectory Tracking');
    view(3);
    
    % Quadcopter Geometry
    L = 0.3; % Arm length (m)
    
    % Initialize graphical objects
    arm1 = plot3([0 0], [0 0], [0 0], 'r', 'LineWidth', 3); % X-axis arm
    arm2 = plot3([0 0], [0 0], [0 0], 'b', 'LineWidth', 3); % Y-axis arm
    path_trace = plot3(X(1), Y(1), Z(1), 'g', 'LineWidth', 1.5);
    
    num_steps = size(x_log, 2);
    
    for k = 1:2:num_steps % Render every other frame for smooth playback
        
        % Current translation and rotation
        pos = [X(k); Y(k); Z(k)];
        phi = Phi(k); theta = Theta(k); psi = Psi(k);
        
        % Direction Cosine Matrix (Body to Inertial)
        R_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
        R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
        R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
        R = R_z * R_y * R_x;
        
        % Arm end points in body frame
        p1 = [L; 0; 0]; p2 = [-L; 0; 0];
        p3 = [0; L; 0]; p4 = [0; -L; 0];
        
        % Transform to inertial frame
        P1 = R*p1 + pos; P2 = R*p2 + pos;
        P3 = R*p3 + pos; P4 = R*p4 + pos;
        
        % Update graphics
        set(arm1, 'XData', [P1(1) P2(1)], 'YData', ...
                           [P1(2) P2(2)], 'ZData', ...
                           [P1(3) P2(3)]);

        set(arm2, 'XData', [P3(1) P4(1)], 'YData', ...
                           [P3(2) P4(2)], 'ZData', ...
                           [P3(3) P4(3)]);

        set(path_trace, 'XData', X(1:k), 'YData', ...
            Y(1:k), 'ZData', Z(1:k));
        
        drawnow;
        pause(Ts/2);
    end
end