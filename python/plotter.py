import numpy as np
import matplotlib.pyplot as plt
import os

def generate_scientific_plots(x_log, r_log, u_log, Ts, output_dir, prefix):
    """
    Generates and saves scientific metric subplots dynamically.
    """
    time = [i * Ts for i in range(x_log.shape[1])]
    os.makedirs(output_dir, exist_ok=True)
    
    print("Generating scientific plots...")
    
    # 1. Altitude Tracking Plot
    plt.figure()
    plt.plot(time, x_log[2, :], 'b-', label='UAV Z (Altitude)')
    plt.plot(time, r_log[2, :], 'k--', label='Ref Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title(f'{prefix}: Altitude Tracking')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, f'{prefix}_altitude.png'), dpi=300)
    plt.close()

    # 2. Euler Angles Plot
    plt.figure()
    plt.plot(time, np.degrees(x_log[6, :]), 'r-', label='Roll (phi)')
    plt.plot(time, np.degrees(x_log[7, :]), 'g-', label='Pitch (theta)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title(f'{prefix}: Body Frame Euler Angles')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, f'{prefix}_euler_angles.png'), dpi=300)
    plt.close()
    
    # 3. Control Inputs
    plt.figure()
    plt.plot(time, u_log[0, :], 'b-', label='Total Thrust (N)')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title(f'{prefix}: Primary Control Input')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, f'{prefix}_control_thrust.png'), dpi=300)
    plt.close()