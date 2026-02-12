import numpy as np
from math import pi, sin, cos

class FK:
    def __init__(self):
        self.alphas = [0, -pi/2,  pi/2, -pi/2, pi/2,  pi/2, pi/2, 0]
        self.a      = [0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0]
        self.d      = [0.141, 0.192, 0,  0.316, 0, 0.384, 0, 0.210]

    def get_transform(self, alpha, a, d, theta):
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),             cos(alpha),            d],
            [0,           0,                      0,                     1]
        ])

    def forward(self, q):
        q = np.asarray(q).copy()

        q_offset = np.zeros(7)
        #q_offset[6] = -pi/4

        jointPositions = np.zeros((8, 3))
        T0i = np.eye(4)

        # 7 个关节：每做完一次 Ai，就记录该关节(i+1)的位置到 jointPositions[i]
        for i in range(7):
            theta = q[i] + q_offset[i]
            Ai = self.get_transform(self.alphas[i], self.a[i], self.d[i], theta)
            T0i = T0i @ Ai
            jointPositions[i] = T0i[:3, 3]     # 注意：写 i，不是 i+1

        # 最后一段固定变换：flange -> EE（你这里用的是 alphas[7], a[7], d[7]）
        A_ee = self.get_transform(self.alphas[7], self.a[7], self.d[7], -pi/4)
        T0e = T0i @ A_ee
        jointPositions[7] = T0e[:3, 3]        # EE 位置单独放最后一行

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
