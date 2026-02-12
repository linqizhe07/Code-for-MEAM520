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
        q = np.asarray(q).copy()
        q_offset = np.zeros(7)
        q_offset[6] = -pi/4 # 保持你的偏移量逻辑

        jointPositions = np.zeros((8, 3))
        T0i = np.eye(4)
    
        # 明确记录基座位置
        jointPositions[0] = T0i[:3, 3] 

        for i in range(7):
            theta = q[i] + q_offset[i]
            Ai = self.get_transform(self.alphas[i], self.a[i], self.d[i], theta)
            T0i = T0i @ Ai
            # 记录经过第 i 个关节变换后的位置（即关节 i+1 的位置）
            jointPositions[i+1] = T0i[:3, 3]

        return jointPositions, T0i

    # feel free to define additional helper methods to modularize your solution for lab 1
