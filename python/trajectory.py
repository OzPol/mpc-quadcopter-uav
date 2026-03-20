import numpy as np

class TrajectoryGenerator:
    def __init__(self, radius=3.0, height=5.0, omega=0.4, scale_lemniscate=4.0):
        self.r = radius
        self.h = height
        self.w = omega
        self.a = scale_lemniscate # Scale for figure-8

    def get_reference(self, t_vec, mode='circle'):
        """
        Generates the reference state matrix for the MPC horizon based on mode.
        modes: 'circle', 'helix', 'lemniscate'
        """
        Np = len(t_vec)
        ref_matrix = np.zeros((Np, 12))
        
        if mode == 'circle':
            ref_matrix[:, 0] = self.r * np.cos(self.w * t_vec)
            ref_matrix[:, 1] = self.r * np.sin(self.w * t_vec)
            ref_matrix[:, 2] = self.h
            ref_matrix[:, 8] = self.w * t_vec + np.pi / 2
            
        elif mode == 'helix':
            ref_matrix[:, 0] = self.r * np.cos(self.w * t_vec)
            ref_matrix[:, 1] = self.r * np.sin(self.w * t_vec)
            ref_matrix[:, 2] = 1.0 + 0.2 * t_vec 
            ref_matrix[:, 8] = self.w * t_vec + np.pi / 2
            
        elif mode == 'lemniscate':
            ref_matrix[:, 0] = self.a * np.sin(self.w * t_vec)
            ref_matrix[:, 1] = self.a * np.sin(self.w * t_vec) * np.cos(self.w * t_vec)
            ref_matrix[:, 2] = self.h
            
            # --- GLOBAL UNWRAP FIX ---
            # To prevent the MPC from spinning at the +/- 180 boundary, 
            # we need to unwrap the angle from t=0 to guarantee absolute continuity.
            
            dt = t_vec[1] - t_vec[0] if len(t_vec) > 1 else 0.1
            num_steps = int(np.round(t_vec[-1] / dt)) + 1
            
            # Generate time array from t=0 to the end of the current horizon
            t_full = np.linspace(0, t_vec[-1], num_steps)
            
            dy_full = self.a * self.w * (np.cos(self.w*t_full)**2 - np.sin(self.w*t_full)**2)
            dx_full = self.a * self.w * np.cos(self.w*t_full)
            
            # Unwrap the entire history so the angle never snaps back
            yaw_full_unwrapped = np.unwrap(np.arctan2(dy_full, dx_full))
            
            # Slice only the horizon chunk the MPC actually requested
            ref_matrix[:, 8] = yaw_full_unwrapped[-Np:]
            
        else:
            raise ValueError(f"Unknown trajectory mode: {mode}")
            
        return ref_matrix