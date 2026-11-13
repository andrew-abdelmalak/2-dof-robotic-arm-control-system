"""Euler-Lagrange dynamics and planar 2-DOF manipulator kinematics."""

import numpy as np
from numpy.typing import NDArray

from arm_control.params import ArmParams, MotorParams


def mass_matrix(q: NDArray[np.float64], arm: ArmParams) -> NDArray[np.float64]:
    """Compute the symmetric positive-definite 2x2 inertia matrix M(q)."""
    c2 = float(np.cos(q[1]))
    m11 = (
        arm.i1_total
        + arm.i2_total
        + arm.m1_total * arm.a1**2
        + arm.m2_total * (arm.l1**2 + arm.a2**2 + 2.0 * arm.l1 * arm.a2 * c2)
    )
    m12 = arm.i2_total + arm.m2_total * (arm.a2**2 + arm.l1 * arm.a2 * c2)
    m22 = arm.i2_total + arm.m2_total * arm.a2**2
    return np.array([[m11, m12], [m12, m22]], dtype=np.float64)


def coriolis_matrix(
    q: NDArray[np.float64], dq: NDArray[np.float64], arm: ArmParams
) -> NDArray[np.float64]:
    """Compute the 2x2 Coriolis/centrifugal matrix C(q, dq) satisfying Mdot - 2C skew-symmetry."""
    h = arm.m2_total * arm.l1 * arm.a2 * float(np.sin(q[1]))
    dq1 = float(dq[0])
    dq2 = float(dq[1])
    return np.array(
        [
            [-h * dq2, -h * (dq1 + dq2)],
            [h * dq1, 0.0],
        ],
        dtype=np.float64,
    )


def gravity_vector(q: NDArray[np.float64], arm: ArmParams) -> NDArray[np.float64]:
    """Compute the 2x1 gravity torque vector G(q) (zero in horizontal planar configuration)."""
    q1 = float(q[0])
    q2 = float(q[1])
    g1 = arm.m1_total * arm.a1 * arm.g * float(np.cos(q1)) + arm.m2_total * arm.g * (
        arm.l1 * float(np.cos(q1)) + arm.a2 * float(np.cos(q1 + q2))
    )
    g2 = arm.m2_total * arm.a2 * arm.g * float(np.cos(q1 + q2))
    return np.array([g1, g2], dtype=np.float64)


def actuator_torque(
    voltage: NDArray[np.float64], dq: NDArray[np.float64], motor: MotorParams
) -> NDArray[np.float64]:
    """Compute DC gear-motor torque tau = B_mot * V - D_mot * dq."""
    return motor.b_matrix @ voltage - motor.d_matrix @ dq


def forward_dynamics(
    q: NDArray[np.float64],
    dq: NDArray[np.float64],
    voltage: NDArray[np.float64],
    arm: ArmParams,
    motor: MotorParams,
    saturate_voltage: bool = True,
) -> NDArray[np.float64]:
    """Compute joint accelerations ddq = M(q)^(-1) * (tau_m - C(q, dq)*dq - G(q))."""
    u = np.clip(voltage, -motor.v_max, motor.v_max) if saturate_voltage else voltage
    m_mat = mass_matrix(q, arm)
    c_mat = coriolis_matrix(q, dq, arm)
    g_vec = gravity_vector(q, arm)
    tau_m = actuator_torque(u, dq, motor)
    return np.linalg.solve(m_mat, tau_m - c_mat @ dq - g_vec)


def forward_kinematics(
    q: NDArray[np.float64], l1: float = 0.200, l2: float = 0.200
) -> NDArray[np.float64]:
    """Compute planar 2-DOF end-effector position [x, y] from joint angles [q1, q2]."""
    q1, q2 = float(q[0]), float(q[1])
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return np.array([x, y], dtype=np.float64)


def inverse_kinematics(
    xy: NDArray[np.float64], l1: float = 0.200, l2: float = 0.200, elbow_up: bool = False
) -> NDArray[np.float64]:
    """Compute analytical 2-DOF planar inverse kinematics matching end_effector_circle.ino."""
    x, y = float(xy[0]), float(xy[1])
    r2 = x * x + y * y
    cos_q2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if cos_q2 < -1.0 - 1e-9 or cos_q2 > 1.0 + 1e-9:
        r_min, r_max = abs(l1 - l2), l1 + l2
        raise ValueError(f"Target ({x}, {y}) lies outside reachable annulus [{r_min}, {r_max}].")
    cos_q2 = float(np.clip(cos_q2, -1.0, 1.0))
    sin_q2 = float(np.sqrt(max(0.0, 1.0 - cos_q2 * cos_q2)))
    if elbow_up:
        sin_q2 = -sin_q2
    q2 = float(np.arctan2(sin_q2, cos_q2))
    q1 = float(np.arctan2(y, x) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2)))
    return np.array([q1, q2], dtype=np.float64)
