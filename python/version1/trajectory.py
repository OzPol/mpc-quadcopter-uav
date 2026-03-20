import numpy as np

def generate_trajectory(t_vec, radius, height, omega):
    """
    Generates the reference state matrix for the MPC horizon.
    Returns an Np x 12 matrix to match the MATLAB dimensions.
    """
    Np = len(t_vec)
    ref_matrix = np.zeros((Np, 12))
    
    # x, y, z positions
    ref_matrix[:, 0] = radius * np.cos(omega * t_vec)
    ref_matrix[:, 1] = radius * np.sin(omega * t_vec)
    ref_matrix[:, 2] = height
    
    # Yaw (psi) - maintain nose tangent to the trajectory
    ref_matrix[:, 8] = omega * t_vec + np.pi / 2
    
    return ref_matrix