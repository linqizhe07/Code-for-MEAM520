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






    # feel free to define additional helper methods to modularize your solution for lab 1

    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)


