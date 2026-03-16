import numpy as np
from math import pi, acos
from scipy.linalg import null_space
from scipy.spatial.transform import Rotation

from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK


class IK:
    # JOINT LIMITS
    lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upper = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    center = lower + (upper - lower) / 2
    fk = FK()

    def __init__(
        self, linear_tol=1e-4, angular_tol=1e-3, max_steps=1000, min_step_size=1e-5
    ):
        self.linear_tol = linear_tol
        self.angular_tol = angular_tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size

    ######################
    ## Helper Functions ##
    ######################

    @staticmethod
    def displacement_and_axis(target, current):
        """
        Computes the displacement vector and axis of rotation from the current
        frame to the target frame.

        INPUTS:
        target - 4x4 numpy array representing the desired transformation
        current - 4x4 numpy array representing the current end effector pose

        OUTPUTS:
        displacement - 3-element numpy array: translation from current to target in world frame
        axis - 3-element numpy array: axis of rotation with magnitude sin(angle),
               expressed in world frame
        """
        # Displacement: simply target origin minus current origin
        displacement = target[:3, 3] - current[:3, 3]

        # Rotation error
        R_curr = current[:3, :3]
        R_des = target[:3, :3]

        # Relative rotation: from current to desired, expressed in current frame
        R_rel = R_curr.T @ R_des

        # Extract axis * sin(theta) from skew-symmetric part of R_rel
        # S = (R_rel - R_rel^T) / 2, then extract vector
        S = (R_rel - R_rel.T) / 2.0
        axis_in_current = np.array([S[2, 1], S[0, 2], S[1, 0]])

        # Transform to world frame
        axis = R_curr @ axis_in_current

        return displacement, axis

    @staticmethod
    def distance_and_angle(G, H):
        """
        Computes the distance and angle between any two transforms.

        INPUTS:
        G - a 4x4 numpy array representing some homogenous transformation
        H - a 4x4 numpy array representing some homogenous transformation

        OUTPUTS:
        distance - the distance in meters between the origins of G & H
        angle - the angle in radians between the orientations of G & H
        """
        # Distance between origins
        distance = np.linalg.norm(G[:3, 3] - H[:3, 3])

        # Angle between orientations
        R_rel = G[:3, :3].T @ H[:3, :3]
        # Use trace to compute angle: trace(R) = 1 + 2*cos(theta)
        cos_angle = (np.trace(R_rel) - 1.0) / 2.0
        # Clamp for numerical stability
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.abs(acos(cos_angle))

        return distance, angle

    def is_valid_solution(self, q, target):
        """
        Check whether q is a valid IK solution: within joint limits and
        achieves the target pose within tolerances.
        """
        # Check joint limits
        if np.any(q < IK.lower) or np.any(q > IK.upper):
            return False, "Solution violates joint limits"

        # Check end effector pose
        _, T0e = IK.fk.forward(q)
        distance, angle = IK.distance_and_angle(target, T0e)

        if distance > self.linear_tol:
            return False, "Solution exceeds linear tolerance (d={:.6f})".format(distance)

        if angle > self.angular_tol:
            return False, "Solution exceeds angular tolerance (ang={:.6f})".format(angle)

        return True, "Solution found within joint limits"

    ####################
    ## Task Functions ##
    ####################

    @staticmethod
    def end_effector_task(q, target):
        """
        Primary task: compute joint velocity to reduce end effector pose error.
        Uses the pseudo-inverse of the Jacobian.
        """
        _, T0e = IK.fk.forward(q)
        displacement, axis = IK.displacement_and_axis(target, T0e)

        # Desired end effector velocity = [displacement; axis]
        # This naturally decays to zero as we approach the target
        xdot = np.concatenate([displacement, axis])  # 6x1

        J = calcJacobian(q)

        # Use pseudo-inverse to solve: J * dq = xdot
        # This gives minimum-norm dq when underdetermined
        dq = np.linalg.lstsq(J, xdot, rcond=None)[0]

        return dq

    @staticmethod
    def joint_centering_task(q, rate=5e-1):
        """
        Secondary task: move joints toward center of range of motion.
        """
        offset = 2 * (q - IK.center) / (IK.upper - IK.lower)
        dq = rate * -offset
        return dq

    ###############################
    ## Inverse Kinematics Solver ##
    ###############################

    def inverse(self, target, seed, alpha):
        """
        Uses gradient descent to solve the full inverse kinematics of the Panda robot.

        INPUTS:
        target - 4x4 numpy array representing the desired end effector pose
        seed - 1x7 vector of joint angles (initial guess)
        alpha - step size for gradient descent

        OUTPUTS:
        q - 1x7 vector of joint angles (solution or closest guess)
        rollout - list of q at each iteration
        success - True if solution found within tolerance
        message - description of result
        """
        q = seed.copy()
        rollout = []

        for step in range(self.max_steps):
            rollout.append(q.copy())

            # Primary Task - Achieve End Effector Pose
            dq_ik = IK.end_effector_task(q, target)

            # Secondary Task - Center Joints
            dq_center = IK.joint_centering_task(q)

            ## Task Prioritization
            # Project secondary task into null space of Jacobian
            J = calcJacobian(q)
            # Pseudo-inverse of J
            J_pinv = np.linalg.lstsq(J, np.eye(6), rcond=None)[0]  # 7x6
            # Null space projector: I - J_pinv * J
            N = np.eye(7) - J_pinv @ J
            # Combined: primary + null-space projected secondary
            dq = dq_ik + N @ dq_center

            # Scale by alpha
            dq = alpha * dq

            # Check termination: step too small
            if np.linalg.norm(dq) < self.min_step_size:
                break

            # Update q
            q = q + dq

        success, message = self.is_valid_solution(q, target)
        return q, rollout, success, message


################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=5)

    ik = IK()

    # matches figure in the handout
    seed = np.array([0, 0, 0, -pi / 2, 0, pi / 2, pi / 4])

    target = np.array(
        [
            [0, -1, 0, -0.2],
            [-1, 0, 0, 0],
            [0, 0, -1, 0.5],
            [0, 0, 0, 1],
        ]
    )

    # Using pseudo-inverse
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = ik.inverse(
        target, seed, alpha=0.5
    )

    for i, q_i in enumerate(rollout_pseudo):
        joints, pose = ik.fk.forward(q_i)
        d, ang = IK.distance_and_angle(target, pose)
        print(
            "iteration:",
            i,
            " q =",
            q_i,
            " d={d:3.4f}  ang={ang:3.3f}".format(d=d, ang=ang),
        )

    # compare
    print("\nmethod: J_pseudo-inverse")
    print("   Success: ", success_pseudo, ":  ", message_pseudo)
    print("   Solution: ", q_pseudo)
    print("   #Iterations : ", len(rollout_pseudo))
