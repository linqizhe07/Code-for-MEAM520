import numpy as np
from lib.calculateFK import FK

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """
    J = np.zeros((6, 7))

    fk = FK()

    # Get the end effector position
    joint_positions, T0e = fk.forward(q_in)
    o_e = T0e[:3, 3]  # end effector origin in world frame

    # Get axis of rotation for each joint in world frame
    axes = fk.get_axis_of_rotation(q_in)  # 3x7

    # Get the position of each joint's rotation axis origin in world frame
    # We need the origin of the frame just before each joint rotates.
    # Build cumulative transforms to get these origins.
    from math import pi
    q_full = np.append(0, q_in)
    q_offset = np.array([0, 0, 0, 0, pi, 0, -pi, -pi/4], dtype=float)

    T = np.eye(4)
    joint_origins = np.zeros((7, 3))  # origin for each joint axis
    for i in range(8):
        theta = q_full[i] + q_offset[i]
        Ai = fk.get_transform(fk.alphas[i], fk.a[i], fk.d[i], theta)
        if i >= 1 and i <= 7:
            # Origin before applying frame i = current T's origin
            joint_origins[i-1] = T[:3, 3]
        T = T @ Ai

    # Build Jacobian column by column
    for j in range(7):
        z_j = axes[:, j]          # rotation axis for joint j
        o_j = joint_origins[j]    # origin of joint j's axis

        # Linear velocity: z_j x (o_e - o_j)
        J[:3, j] = np.cross(z_j, o_e - o_j)

        # Angular velocity: z_j
        J[3:, j] = z_j

    return J

if __name__ == "__main__":
    q = np.array([0, 0, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 4])
    print(np.round(calcJacobian(q), 3))
