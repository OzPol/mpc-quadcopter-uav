import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
import os

def run_animation(x_log, r_log, Ts):
    """
    Renders 3D flight path, saves static plots, GIFs, and MP4s.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    X, Y, Z = x_log[0, :], x_log[1, :], x_log[2, :]
    Phi, Theta, Psi = x_log[6, :], x_log[7, :], x_log[8, :]
    
    # Setup 3D Canvas
    # ax.plot(r_log[0, :], r_log[1, :], r_log[2, :], 'k--', linewidth=1.5, label='Reference')
    ax.plot(r_log[0, :], r_log[1, :], r_log[2, :], color='cyan',
            linestyle='--', linewidth=2, alpha=0.6, label='Ghost Reference')
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([0, 8])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Python MPC Quadcopter Trajectory Tracking')
    ax.view_init(elev=30, azim=-45) # Matches MATLAB's view(3) default
    
    # Quadcopter Geometry
    L = 0.3
    arm1, = ax.plot([], [], [], 'r', linewidth=3)
    arm2, = ax.plot([], [], [], 'b', linewidth=3)
    # path_trace, = ax.plot([], [], [], 'g', linewidth=1.5)
    # to make the path_trace object stand out from the drone arms:
    path_trace, = ax.plot([], [], [], color='magenta', 
                        linewidth=2.5, label='UAV Flight Path')
    
    def update(k):
        pos = np.array([X[k], Y[k], Z[k]])
        phi, theta, psi = Phi[k], Theta[k], Psi[k]
        
        # Rotation Matrix
        Rx = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
        Ry = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
        Rz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        
        # Arm points
        p1, p2 = R @ np.array([L, 0, 0]) + pos, R @ np.array([-L, 0, 0]) + pos
        p3, p4 = R @ np.array([0, L, 0]) + pos, R @ np.array([0, -L, 0]) + pos
        
        arm1.set_data(np.array([p1[0], p2[0]]), np.array([p1[1], p2[1]]))
        arm1.set_3d_properties(np.array([p1[2], p2[2]]))
        
        arm2.set_data(np.array([p3[0], p4[0]]), np.array([p3[1], p4[1]]))
        arm2.set_3d_properties(np.array([p3[2], p4[2]]))
        
        path_trace.set_data(X[:k+1], Y[:k+1])
        path_trace.set_3d_properties(Z[:k+1])
        
        return arm1, arm2, path_trace

    # Animate skipping frames for playback speed
    frames = range(0, x_log.shape[1], 2)
    anim = FuncAnimation(fig, update, frames=frames, interval=Ts*1000, blit=False)
    
    # Output File Generation
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'output')
    os.makedirs(output_dir, exist_ok=True)
    
    print("Generating digital assets...")
    
    # Save using the robust absolute paths
    plt.savefig(os.path.join(output_dir, 'static_plot.png'), dpi=300)
    anim.save(os.path.join(output_dir, 'flight.gif'), writer=PillowWriter(fps=15))
    
    try:
        anim.save(os.path.join(output_dir, 'flight.mp4'), writer=FFMpegWriter(fps=30))
    except Exception as e:
        print("FFmpeg not found on machine, skipping MP4 generation.")
        
    plt.show()