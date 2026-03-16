import numpy as np
from math import pi, sin, cos

class FK:
    def __init__(self):
        # Standard DH
        self.alphas = [0, -pi/2,  pi/2, pi/2,  pi/2,  -pi/2,  pi/2, 0]
        self.a      = [0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0]
        self.d      = [0.141, 0.192, 0,  0.316, 0, 0.384, 0, 0.210]
        # Relative Offset of joint centers compare to the origin of DH frame 
        self.joint_offset_local = np.array([
            [0, 0, 0],  # i=0
            [0, 0, 0],  # i=1
            [0, 0, 0.195],  # i=2
            [0, 0, 0],  # i=3
            [0, 0, 0.125],  # i=4
            [0, 0, -0.015],  # i=5
            [0, 0, 0.051],  # i=6
            [0, 0, 0],  # i=7 
        ], dtype=float)

    def get_transform(self, alpha, a, d, theta):
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),             cos(alpha),            d],
            [0,           0,                      0,                     1]
        ])

    def forward(self, q):
        q = np.asarray(q).copy()
        assert q.shape[0] == 7
        # Offset of Worldframe
        q = np.append(0, q)
        q_offset = np.array([0, 0, 0, 0, pi, 0, -pi, -pi/4], dtype=float)
        jointPositions = np.zeros((8, 3))
        T0e = np.eye(4)
        for i in range(8):
            theta = q[i] + q_offset[i]
            Ai = self.get_transform(self.alphas[i], self.a[i], self.d[i], theta)
            T0e = T0e @ Ai
            p_origin_world = T0e[:3, 3]
            R_world_i      = T0e[:3, :3]
            # Put the relative Offset into the World
            p_joint_world = p_origin_world + R_world_i @ self.joint_offset_local[i]
            jointPositions[i] = p_joint_world
        return jointPositions, T0e

    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame
        """
        axis_of_rotation_list = np.zeros((3, 7))

        # Build cumulative transforms up to each joint frame
        # DH frame 0 is the base (fixed), DH frames 1..7 correspond to joints 0..6
        # The axis of rotation for joint i is the z-axis of the frame just before joint i rotates
        
        q_full = np.append(0, q)  # prepend 0 for base frame
        q_offset = np.array([0, 0, 0, 0, pi, 0, -pi, -pi/4], dtype=float)

        T = np.eye(4)
        # Frame 0 (base): joint 0 rotates about z of frame after DH frame 0
        T_frames = []  # T_frames[i] = T from world to DH frame i
        for i in range(8):
            theta = q_full[i] + q_offset[i]
            Ai = self.get_transform(self.alphas[i], self.a[i], self.d[i], theta)
            T = T @ Ai
            T_frames.append(T.copy())

        # For joint j (0-indexed, j=0..6), the rotation axis is the z-axis of 
        # the DH frame just before that joint's rotation is applied.
        # Joint j corresponds to DH frame j+1 (since frame 0 is base).
        # The z-axis before joint j rotates is the z-axis of T after applying
        # frames 0..j (i.e., including the previous frame but before joint j's A matrix).
        
        # Actually, let's recompute more carefully:
        # We have 8 DH frames (i=0..7). q_full[0]=0 (base), q_full[1..7] = q[0..6]
        # Joint j (j=0..6) is parameterized by q[j] = q_full[j+1]
        # The axis of rotation for joint j is the z-axis of the frame accumulated
        # up to (but not including) DH frame j+1's transform.
        
        T = np.eye(4)
        for i in range(8):
            theta = q_full[i] + q_offset[i]
            Ai = self.get_transform(self.alphas[i], self.a[i], self.d[i], theta)
            if i >= 1 and i <= 7:
                # Before applying frame i, the z-axis is the rotation axis for joint i-1
                # (which corresponds to q[i-1])
                axis_of_rotation_list[:, i-1] = T[:3, 2]
            T = T @ Ai

        return axis_of_rotation_list
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        Ai_list = []
        q_full = np.append(0, q)
        q_offset = np.array([0, 0, 0, 0, pi, 0, -pi, -pi/4], dtype=float)
        
        for i in range(8):
            theta = q_full[i] + q_offset[i]
            A = self.get_transform(self.alphas[i], self.a[i], self.d[i], theta)
            Ai_list.append(A)
        
        return Ai_list
    
if __name__ == "__main__":
    fk = FK()
    q = np.array([0,0,0,0,0,0,0])
    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
