import numpy as np
from math import pi, sin, cos

class FK:
    def __init__(self):
        # 再次确认你的 DH 表！以下是标准 DH 的通用结构
        self.alphas = [-pi/2, pi/2, -pi/2, -pi/2, pi/2, -pi/2, 0]
        self.a      = [0, 0, 0.0825, 0.0825, 0, 0.088, 0]
        self.d      = [0.333, 0, 0.316, 0, 0.384, 0, 0.210]

    def get_transform(self, alpha, a, d, theta):
        # 这是 Standard DH 矩阵
        return np.array([
            [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
            [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
            [0,           sin(alpha),             cos(alpha),            d],
            [0,           0,                      0,                     1]
        ])

    def forward(self, q):
        T = np.eye(4)

        # 8x3: joint1..joint7 + end effector
        jointPositions = np.zeros((8, 3))

        for i in range(7):
            Ai = self.get_transform(self.alphas[i], self.a[i], self.d[i], q[i])
            T = T @ Ai
            # 第 i 行就是 joint(i+1)
            jointPositions[i] = T[:3, 3]

        # EE：如果 handout 把 EE 和 joint7 同一点，那就直接用 T
        # 如果 handout 说 EE 在 joint7 的 z 方向还有一个固定长度 d_ee，就在这里再乘一个固定 T7e
        T0e = T
        jointPositions[7] = T0e[:3, 3]

        return jointPositions, T0e


    
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
