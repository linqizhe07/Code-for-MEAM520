import numpy as np
from lib.calcJacobian import calcJacobian

def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """
    J = calcJacobian(q_in)

    v_in = v_in.flatten()
    omega_in = omega_in.flatten()

    # Stack desired velocity
    xi = np.concatenate([v_in, omega_in])  # 6-element

    # Find which rows are constrained (not NaN)
    constrained = ~np.isnan(xi)

    # Extract constrained rows from J and xi
    J_constrained = J[constrained, :]
    xi_constrained = xi[constrained]

    # Solve using least squares (handles both underdetermined and overdetermined)
    # lstsq minimizes ||dq||_2 when underdetermined, minimizes ||J*dq - xi||_2 when overdetermined
    dq, _, _, _ = np.linalg.lstsq(J_constrained, xi_constrained, rcond=None)

    dq = dq.reshape(1, 7)
    return dq
