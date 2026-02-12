import numpy as np
from math import pi, sin, cos

class FK:
    def __init__(self):
        self.alphas = [-pi/2,  pi/2, -pi/2, -pi/2,  pi/2, -pi/2, 0]
        self.a      = [0, 0, 0.0825, 0.0825, 0, 0.088, 0]
        self.d      = [0.333, 0, 0.316, 0, 0.384, 0, 0.210]

    def get_transform(self, alpha, a, d, theta):
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),             cos(alpha),            d],
            [0,           0,                      0,                     1]
        ])

    def forward(self, q):
        """
        q: (7,) joint angles [q0..q6]
        returns:
          jointPositions: (8,3) positions of 7 joints + end-effector
          T0e: (4,4) base->end-effector transform
        """
        q = np.asarray(q).copy()  # avoid mutating caller input

        # joint angle offsets (only apply once)
        q_offset = np.zeros(7)
        q_offset[6] = -pi/4

        jointPositions = np.zeros((8, 3))
        T0e = np.eye(4)

        # 7 joints
        for i in range(7):
            theta = q[i] + q_offset[i]
            Ai = self.get_transform(self.alphas[i], self.a[i], self.d[i], theta)
            T0e = T0e @ Ai
            jointPositions[i] = T0e[:3, 3]


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
