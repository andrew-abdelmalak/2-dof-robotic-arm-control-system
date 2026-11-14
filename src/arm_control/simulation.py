"""Closed-loop ODE simulations reproducing Simulink and Arduino control modes."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.integrate import solve_ivp

from arm_control.controllability import compute_pole_placement, linearize_at_origin
from arm_control.controllers import ComputedTorqueController
from arm_control.dynamics import forward_dynamics, forward_kinematics, inverse_kinematics
from arm_control.params import ArmParams, MotorParams


@dataclass(frozen=True)
class RegulationMetrics:
    """Summary metrics for step position regulation."""

    t: NDArray[np.float64]
    q: NDArray[np.float64]
    dq: NDArray[np.float64]
    voltage: NDArray[np.float64]
    settling_time_s: float
    max_overshoot_rad: float
    steady_state_error_rad: float


@dataclass(frozen=True)
class CircleTrackingMetrics:
    """Summary metrics for circular trajectory tracking."""

    t: NDArray[np.float64]
    q: NDArray[np.float64]
    q_d: NDArray[np.float64]
    voltage: NDArray[np.float64]
    steady_amplitude_error_pct: float
    steady_phase_error_rad: float
    peak_q1_rad: float
    peak_q1_time_s: float


def simulate_pole_placement_regulation(
    arm: ArmParams | None = None,
    motor: MotorParams | None = None,
    x0: NDArray[np.float64] | None = None,
    poles: NDArray[np.float64] | None = None,
    t_end: float = 5.0,
) -> tuple[NDArray[np.float64], NDArray[np.float64], float]:
    """Reproduce matlab/check_controllability.m nonlinear closed-loop regulation."""
    arm_p = ArmParams() if arm is None else arm
    motor_p = MotorParams() if motor is None else motor
    init_x = np.array([0.1, -0.08, 0.05, -0.03], dtype=np.float64) if x0 is None else x0
    des_poles = np.array([-4.0, -5.0, -6.0, -7.0], dtype=np.float64) if poles is None else poles

    a_mat, b_mat = linearize_at_origin(arm_p, motor_p)
    k_gain = compute_pole_placement(a_mat, b_mat, des_poles)

    def rhs(_t: float, x: NDArray[np.float64]) -> NDArray[np.float64]:
        u = -k_gain @ x
        ddq = forward_dynamics(x[:2], x[2:], u, arm_p, motor_p, saturate_voltage=False)
        return np.concatenate([x[2:], ddq])

    t_eval = np.linspace(0.0, t_end, 501)
    sol = solve_ivp(rhs, (0.0, t_end), init_x, t_eval=t_eval, rtol=1e-7, atol=1e-9)
    final_norm = float(np.linalg.norm(sol.y[:, -1]))
    return sol.t, sol.y.T, final_norm


def simulate_position_regulation(
    q_target: tuple[float, float] = (0.5, 0.5),
    kp: tuple[float, float] = (50.0, 50.0),
    kd: tuple[float, float] = (10.0, 10.0),
    t_end: float = 5.0,
    arm: ArmParams | None = None,
    motor: MotorParams | None = None,
) -> RegulationMetrics:
    """Reproduce Simulink POS mode step regulation from rest."""
    arm_p = ArmParams() if arm is None else arm
    motor_p = MotorParams() if motor is None else motor
    ctrl = ComputedTorqueController(arm_p, motor_p, kp=kp, kd=kd, saturate=True)
    qd = np.array(q_target, dtype=np.float64)
    dqd = np.zeros(2, dtype=np.float64)

    def rhs(_t: float, x: NDArray[np.float64]) -> NDArray[np.float64]:
        u = ctrl.compute_voltage(x[:2], x[2:], qd, dqd)
        ddq = forward_dynamics(x[:2], x[2:], u, arm_p, motor_p, saturate_voltage=True)
        return np.concatenate([x[2:], ddq])

    t_eval = np.linspace(0.0, t_end, 1001)
    sol = solve_ivp(rhs, (0.0, t_end), np.zeros(4), t_eval=t_eval, rtol=1e-7, atol=1e-9)
    states = sol.y.T
    q = states[:, :2]
    dq = states[:, 2:]
    voltages = np.array([ctrl.compute_voltage(q[i], dq[i], qd, dqd) for i in range(len(t_eval))])

    # 2% settling time across both joints
    settling_times = []
    overshoots = []
    for idx in range(2):
        target = qd[idx]
        if abs(target) < 1e-12:
            settling_times.append(0.0)
            overshoots.append(float(np.max(np.abs(q[:, idx]))))
            continue
        band = 0.02 * abs(target)
        outside = np.where(np.abs(q[:, idx] - target) > band)[0]
        last_out = int(outside[-1]) if outside.size > 0 else 0
        settling_times.append(float(t_eval[min(last_out + 1, len(t_eval) - 1)]))
        overshoots.append(float(max(0.0, np.max(np.sign(target) * q[:, idx] - abs(target)))))

    ss_err = float(np.max(np.abs(q[-1] - qd)))
    return RegulationMetrics(
        t=t_eval,
        q=q,
        dq=dq,
        voltage=voltages,
        settling_time_s=max(settling_times),
        max_overshoot_rad=max(overshoots),
        steady_state_error_rad=ss_err,
    )


def simulate_joint_circle(
    radius_rad: float = 0.3,
    omega_rad_s: float = 0.5,
    kp: tuple[float, float] = (50.0, 50.0),
    kd: tuple[float, float] = (10.0, 10.0),
    include_feedforward_accel: bool = False,
    t_end: float = 20.0,
    arm: ArmParams | None = None,
    motor: MotorParams | None = None,
) -> CircleTrackingMetrics:
    """Reproduce Simulink CIRCLE mode (chart_27.xml + chart_35.xml)."""
    arm_p = ArmParams() if arm is None else arm
    motor_p = MotorParams() if motor is None else motor
    ctrl = ComputedTorqueController(arm_p, motor_p, kp=kp, kd=kd, saturate=True)

    def ref(t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        qd = np.array([radius_rad * np.cos(omega_rad_s * t), radius_rad * np.sin(omega_rad_s * t)])
        dqd = np.array(
            [
                -radius_rad * omega_rad_s * np.sin(omega_rad_s * t),
                radius_rad * omega_rad_s * np.cos(omega_rad_s * t),
            ]
        )
        ddqd = (
            np.array(
                [
                    -radius_rad * (omega_rad_s**2) * np.cos(omega_rad_s * t),
                    -radius_rad * (omega_rad_s**2) * np.sin(omega_rad_s * t),
                ]
            )
            if include_feedforward_accel
            else np.zeros(2, dtype=np.float64)
        )
        return qd, dqd, ddqd

    def rhs(t: float, x: NDArray[np.float64]) -> NDArray[np.float64]:
        qd, dqd, ddqd = ref(t)
        u = ctrl.compute_voltage(x[:2], x[2:], qd, dqd, ddqd)
        ddq = forward_dynamics(x[:2], x[2:], u, arm_p, motor_p, saturate_voltage=True)
        return np.concatenate([x[2:], ddq])

    t_eval = np.linspace(0.0, t_end, 2001)
    sol = solve_ivp(rhs, (0.0, t_end), np.zeros(4), t_eval=t_eval, rtol=1e-7, atol=1e-9)
    states = sol.y.T
    q = states[:, :2]
    dq = states[:, 2:]

    qd_hist = np.zeros_like(q)
    voltages = np.zeros_like(q)
    for i, t_val in enumerate(t_eval):
        qd_i, dqd_i, ddqd_i = ref(float(t_val))
        qd_hist[i] = qd_i
        voltages[i] = ctrl.compute_voltage(q[i], dq[i], qd_i, dqd_i, ddqd_i)

    # Steady-state window: second half of simulation
    ss_idx = len(t_eval) // 2
    r_actual = np.sqrt(q[ss_idx:, 0] ** 2 + q[ss_idx:, 1] ** 2)
    amp_err_pct = float(np.max(np.abs(r_actual - radius_rad)) / radius_rad * 100.0)

    phi_actual = np.unwrap(np.arctan2(q[ss_idx:, 1], q[ss_idx:, 0]))
    phi_ref = np.unwrap(np.arctan2(qd_hist[ss_idx:, 1], qd_hist[ss_idx:, 0]))
    phase_err = float(np.max(np.abs(phi_actual - phi_ref)))

    peak_idx = int(np.argmax(q[:, 0]))
    return CircleTrackingMetrics(
        t=t_eval,
        q=q,
        q_d=qd_hist,
        voltage=voltages,
        steady_amplitude_error_pct=amp_err_pct,
        steady_phase_error_rad=phase_err,
        peak_q1_rad=float(q[peak_idx, 0]),
        peak_q1_time_s=float(t_eval[peak_idx]),
    )


def simulate_cartesian_circle(
    center: tuple[float, float] = (0.18, 0.0),
    radius_m: float = 0.04,
    omega_rad_s: float = 2.0,
    l1: float = 0.12,
    l2: float = 0.15,
    t_end: float = 6.0,
) -> float:
    """Verify end_effector_circle.ino analytical IK circular path accuracy [m]."""
    t_eval = np.linspace(0.0, t_end, 601)
    max_pos_err = 0.0
    for t_val in t_eval:
        target = np.array(
            [
                center[0] + radius_m * np.cos(omega_rad_s * t_val),
                center[1] + radius_m * np.sin(omega_rad_s * t_val),
            ]
        )
        q_ik = inverse_kinematics(target, l1=l1, l2=l2)
        xy_fk = forward_kinematics(q_ik, l1=l1, l2=l2)
        max_pos_err = max(max_pos_err, float(np.linalg.norm(xy_fk - target)))
    return max_pos_err
