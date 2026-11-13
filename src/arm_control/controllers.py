"""Computed-torque feedback linearization controller and cubic trajectory generator."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from arm_control.dynamics import coriolis_matrix, gravity_vector, mass_matrix
from arm_control.params import ArmParams, MotorParams


@dataclass(frozen=True)
class ComputedTorqueController:
    """Feedback linearization + PD controller matching Advanced_trial.slx."""

    arm: ArmParams
    motor: MotorParams
    kp: tuple[float, float] | None = None
    kd: tuple[float, float] | None = None
    saturate: bool = True

    def compute_voltage(
        self,
        q: NDArray[np.float64],
        dq: NDArray[np.float64],
        q_d: NDArray[np.float64],
        dq_d: NDArray[np.float64],
        ddq_d: NDArray[np.float64] | None = None,
    ) -> NDArray[np.float64]:
        """Compute motor terminal voltage V = B^-1 (tau_des + D*dq)."""
        kp_vals = (self.arm.kp1, self.arm.kp2) if self.kp is None else self.kp
        kd_vals = (self.arm.kd1, self.arm.kd2) if self.kd is None else self.kd
        kp_mat = np.diag(kp_vals)
        kd_mat = np.diag(kd_vals)

        ddq_ref = np.zeros(2, dtype=np.float64) if ddq_d is None else ddq_d
        err = q_d - q
        derr = dq_d - dq
        v_accel = ddq_ref + kp_mat @ err + kd_mat @ derr

        m_mat = mass_matrix(q, self.arm)
        c_mat = coriolis_matrix(q, dq, self.arm)
        g_vec = gravity_vector(q, self.arm)

        tau_des = m_mat @ v_accel + c_mat @ dq + g_vec
        voltage = np.linalg.solve(self.motor.b_matrix, tau_des + self.motor.d_matrix @ dq)
        if self.saturate:
            voltage = np.clip(voltage, -self.motor.v_max, self.motor.v_max)
        return voltage


def cubic_polynomial_trajectory(
    t: float,
    duration: float,
    q0: NDArray[np.float64],
    qf: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Evaluate rest-to-rest cubic polynomial (qd, dqd, ddqd) matching Arduino firmware."""
    if duration <= 0.0:
        raise ValueError(f"Trajectory duration must be positive, got {duration}.")
    s = float(np.clip(t / duration, 0.0, 1.0))
    delta = qf - q0
    qd = q0 + delta * (3.0 * s**2 - 2.0 * s**3)
    dqd = (6.0 * delta / duration) * (s - s**2)
    ddqd = (6.0 * delta / (duration**2)) * (1.0 - 2.0 * s)
    if t >= duration:
        dqd = np.zeros_like(delta)
        ddqd = np.zeros_like(delta)
    return qd, dqd, ddqd
