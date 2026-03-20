import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
import os

def run_animation(x_log, r_log, Ts, L, output_dir, prefix):
    """
    Renders 3D flight path with a 4-color 'X' quadcopter geometry, 
    saves dynamic GIFs and MP4s based on the prefix.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    X, Y, Z = x_log[0, :], x_log[1, :], x_log[2, :]
    Phi, Theta, Psi = x_log[6, :], x_log[7, :], x_log[8, :]
    
    # Setup 3D Canvas
    ax.plot(r_log[0, :], r_log[1, :], r_log[2, :], 'k--', linewidth=1.5, label='Reference')
    ax.set_xlim([-5, 5]); ax.set_ylim([-5, 5]); ax.set_zlim([0, 8])
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title(f'MPC Trajectory Tracking: {prefix}')
    ax.view_init(elev=30, azim=-45) 
    
    # Quadcopter Geometry ('X' Configuration)
    
    # This version made the crazyflie look like a tiny dot, 
    # so we will artificially scale it up for visualization purposes
    # dx = L * np.cos(np.pi/4)
    # dy = L * np.sin(np.pi/4)
    
    #########################
    
    # VISUAL SCALING: Artificially enlarge micro-drones so they are visible on a 10m plot
    visual_L = L * 10 if L < 0.1 else L 
    
    # Front is defined as the +X direction
    dx = visual_L * np.cos(np.pi/4)
    dy = visual_L * np.sin(np.pi/4)
    
    
    # 4 distinct arms starting from the center outward
    arm_fr, = ax.plot([], [], [], 'r', linewidth=3) # Front-Right (Red)
    arm_fl, = ax.plot([], [], [], 'b', linewidth=3) # Front-Left (Blue)
    arm_br, = ax.plot([], [], [], 'm', linewidth=3) # Back-Right (Magenta)
    arm_bl, = ax.plot([], [], [], 'c', linewidth=3) # Back-Left (Cyan)
    
    body, = ax.plot([], [], [], 'ko', markersize=6) # Center mass
    path_trace, = ax.plot([], [], [], 'g', linewidth=1.5, label='UAV Flight Path')
    
    def update(k):
        pos = np.array([X[k], Y[k], Z[k]])
        phi, theta, psi = Phi[k], Theta[k], Psi[k]
        
        # Rotation Matrix
        Rx = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
        Ry = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
        Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        
        # Rotor endpoints relative to center
        # Assuming +X is forward, +Y is left
        p_fr = R @ np.array([dx, -dy, 0]) + pos
        p_fl = R @ np.array([dx, dy, 0]) + pos
        p_br = R @ np.array([-dx, -dy, 0]) + pos
        p_bl = R @ np.array([-dx, dy, 0]) + pos
        
        # Update each arm to draw a line from the center 'pos' to the rotor point
        arm_fr.set_data(np.array([pos[0], p_fr[0]]), np.array([pos[1], p_fr[1]]))
        arm_fr.set_3d_properties(np.array([pos[2], p_fr[2]]))
        
        arm_fl.set_data(np.array([pos[0], p_fl[0]]), np.array([pos[1], p_fl[1]]))
        arm_fl.set_3d_properties(np.array([pos[2], p_fl[2]]))
        
        arm_br.set_data(np.array([pos[0], p_br[0]]), np.array([pos[1], p_br[1]]))
        arm_br.set_3d_properties(np.array([pos[2], p_br[2]]))
        
        arm_bl.set_data(np.array([pos[0], p_bl[0]]), np.array([pos[1], p_bl[1]]))
        arm_bl.set_3d_properties(np.array([pos[2], p_bl[2]]))
        
        body.set_data(np.array([pos[0]]), np.array([pos[1]]))
        body.set_3d_properties(np.array([pos[2]]))
        
        path_trace.set_data(X[:k+1], Y[:k+1])
        path_trace.set_3d_properties(Z[:k+1])
        
        return arm_fr, arm_fl, arm_br, arm_bl, body, path_trace

    frames = range(0, x_log.shape[1], 2)
    anim = FuncAnimation(fig, update, frames=frames, interval=Ts*1000, blit=False)
    
    print(f"Generating 3D animation assets for {prefix}...")
    
    anim.save(os.path.join(output_dir, f'{prefix}_flight.gif'), writer=PillowWriter(fps=15))
    try:
        anim.save(os.path.join(output_dir, f'{prefix}_flight.mp4'), writer=FFMpegWriter(fps=30))
    except Exception as e:
        print("FFmpeg not found on machine, skipping MP4 generation.")
        
    plt.show()