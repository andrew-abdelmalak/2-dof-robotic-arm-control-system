"""Unit and integration tests for 2-DOF robotic arm control verification."""

import re
from pathlib import Path

import numpy as np
import pytest

from arm_control import (
    ArmParams,
    ComputedTorqueController,
    MotorParams,
    compute_pole_placement,
    coriolis_matrix,
    cubic_polynomial_trajectory,
    evaluate_nonlinear_controllability_grid,
    forward_dynamics,
    forward_kinematics,
    gravity_vector,
    inverse_kinematics,
    kalman_controllability_rank,
    kalman_observability_rank,
    linearize_at_origin,
    mass_matrix,
    nonlinear_lie_controllability,
    simulate_cartesian_circle,
    simulate_joint_circle,
    simulate_pole_placement_regulation,
    simulate_position_regulation,
    solve_lyapunov_certificate,
)

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_params_match_matlab_init_params() -> None:
    """Verify ArmParams and MotorParams match matlab/init_params.m values."""
    arm = ArmParams()
    motor = MotorParams()
    vec = arm.to_matlab_vector()
    mvec = motor.to_matlab_vector()

    assert vec.shape == (17,)
    assert mvec.shape == (4,)
    assert np.isclose(arm.m1_total, 0.05259, atol=1e-9)
    assert np.isclose(arm.m2_total, 0.05289, atol=1e-9)
    assert np.isclose(arm.i1_total, 9.4284e-5, atol=1e-12)
    assert np.isclose(arm.i2_total, 2.18751e-5, atol=1e-12)

    init_src = (REPO_ROOT / "matlab" / "init_params.m").read_text(encoding="utf-8")
    l1_match = re.search(r"l1\s*=\s*([0-9.]+)", init_src)
    b1_match = re.search(r"b1\s*=\s*([0-9.]+)", init_src)
    assert l1_match is not None and float(l1_match.group(1)) == pytest.approx(arm.l1)
    assert b1_match is not None and float(b1_match.group(1)) == pytest.approx(motor.b1)


@pytest.mark.parametrize("q2", [-np.pi, -np.pi / 2, 0.0, np.pi / 3, np.pi])
def test_mass_matrix_symmetric_positive_definite(q2: float) -> None:
    """Inertia matrix M(q) must be symmetric and strictly positive definite."""
    arm = ArmParams()
    m_mat = mass_matrix(np.array([0.4, q2]), arm)
    assert np.allclose(m_mat, m_mat.T, atol=1e-15)
    eigvals = np.linalg.eigvalsh(m_mat)
    assert np.all(eigvals > 0.0)


def test_skew_symmetry_mdot_minus_2c() -> None:
    """Verify (Mdot - 2C) is skew-symmetric for arbitrary state (q, dq)."""
    arm = ArmParams()
    q = np.array([0.7, -1.1])
    dq = np.array([1.3, -0.8])
    eps = 1e-7
    m_plus = mass_matrix(q + eps * dq, arm)
    m_minus = mass_matrix(q - eps * dq, arm)
    m_dot = (m_plus - m_minus) / (2.0 * eps)
    c_mat = coriolis_matrix(q, dq, arm)
    skew = m_dot - 2.0 * c_mat
    assert np.allclose(skew + skew.T, np.zeros((2, 2)), atol=1e-7)


def test_gravity_vector_planar_and_vertical() -> None:
    """Planar g=0 produces zero gravity torque; g=9.81 produces nonzero torque."""
    arm_planar = ArmParams(g=0.0)
    assert np.allclose(gravity_vector(np.array([0.5, 0.5]), arm_planar), 0.0)

    arm_vert = ArmParams(g=9.81)
    g_vec = gravity_vector(np.array([0.0, 0.0]), arm_vert)
    assert g_vec[0] > g_vec[1] > 0.0


def test_forward_dynamics_voltage_saturation() -> None:
    """Forward dynamics clips terminal voltage at +/- v_max when saturate_voltage=True."""
    arm = ArmParams()
    motor = MotorParams(v_max=12.0)
    q = np.array([0.1, -0.2])
    dq = np.array([0.0, 0.0])
    ddq_high = forward_dynamics(q, dq, np.array([50.0, -50.0]), arm, motor, saturate_voltage=True)
    ddq_clip = forward_dynamics(q, dq, np.array([12.0, -12.0]), arm, motor, saturate_voltage=True)
    assert np.allclose(ddq_high, ddq_clip)


@pytest.mark.parametrize("elbow_up", [False, True])
def test_kinematics_roundtrip(elbow_up: bool) -> None:
    """Analytical IK and FK round-trip within machine precision."""
    target = np.array([0.18 + 0.04 * np.cos(0.9), 0.04 * np.sin(0.9)])
    q_ik = inverse_kinematics(target, l1=0.12, l2=0.15, elbow_up=elbow_up)
    xy_fk = forward_kinematics(q_ik, l1=0.12, l2=0.15)
    assert np.allclose(xy_fk, target, atol=1e-12)


def test_inverse_kinematics_unreachable_raises() -> None:
    """Unreachable targets outside [|l1-l2|, l1+l2] must fail fast with ValueError."""
    with pytest.raises(ValueError, match="outside reachable annulus"):
        inverse_kinematics(np.array([0.5, 0.0]), l1=0.12, l2=0.15)


def test_cubic_polynomial_trajectory_boundary_conditions() -> None:
    """Cubic trajectory satisfies zero-velocity boundaries and rejects invalid duration."""
    q0 = np.array([0.0, -0.2])
    qf = np.array([1.0, 0.6])
    q_start, dq_start, _ = cubic_polynomial_trajectory(0.0, 1.5, q0, qf)
    q_mid, dq_mid, ddq_mid = cubic_polynomial_trajectory(0.75, 1.5, q0, qf)
    q_end, dq_end, ddq_end = cubic_polynomial_trajectory(1.5, 1.5, q0, qf)

    assert np.allclose(q_start, q0)
    assert np.allclose(dq_start, 0.0)
    assert np.allclose(q_mid, 0.5 * (q0 + qf))
    assert np.all(dq_mid > 0.0)
    assert np.allclose(ddq_mid, 0.0, atol=1e-12)
    assert np.allclose(q_end, qf)
    assert np.allclose(dq_end, 0.0)
    assert np.allclose(ddq_end, 0.0)

    with pytest.raises(ValueError, match="positive"):
        cubic_polynomial_trajectory(0.0, 0.0, q0, qf)


def test_linear_controllability_observability_pole_placement_and_lyapunov() -> None:
    """Verify rank-4 controllability/observability, pole placement, and Lyapunov P > 0."""
    arm = ArmParams()
    motor = MotorParams()
    a_mat, b_mat = linearize_at_origin(arm, motor)
    c_mat = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])

    assert kalman_controllability_rank(a_mat, b_mat) == 4
    assert kalman_observability_rank(a_mat, c_mat) == 4

    poles = np.array([-4.0, -5.0, -6.0, -7.0])
    k_gain = compute_pole_placement(a_mat, b_mat, poles)
    a_cl = a_mat - b_mat @ k_gain
    eigs_cl = np.sort(np.linalg.eigvals(a_cl).real)
    assert np.allclose(eigs_cl, np.sort(poles), atol=1e-7)

    p_mat, eig_p = solve_lyapunov_certificate(a_cl)
    assert np.all(eig_p > 0.0)
    residual = a_cl.T @ p_mat + p_mat @ a_cl + np.eye(4)
    assert np.allclose(residual, np.zeros((4, 4)), atol=1e-10)


@pytest.mark.parametrize(
    "q",
    [
        np.array([0.0, 0.0]),
        np.array([np.pi / 2, np.pi / 2]),
        np.array([-np.pi, np.pi]),
        np.array([0.7, -1.9]),
    ],
)
def test_nonlinear_lie_controllability_pointwise(q: np.ndarray) -> None:
    """Lie-bracket distribution [g1, g2, [f,g1], [f,g2]] must have rank 4."""
    arm = ArmParams()
    motor = MotorParams()
    x = np.array([q[0], q[1], 0.0, 0.0])
    rank, svals = nonlinear_lie_controllability(x, arm, motor)
    assert rank == 4
    assert svals[-1] > 100.0


def test_nonlinear_lie_controllability_full_grid() -> None:
    """Verify full rank 4 across all 120x120 = 14,400 grid configurations on [-pi, pi]^2."""
    res = evaluate_nonlinear_controllability_grid(ArmParams(), MotorParams(), n_grid=120)
    assert res.total_points == 14_400
    assert res.singular_points == 0
    assert res.all_full_rank
    assert res.min_singular_value > 300.0


def test_computed_torque_feedback_linearization_exact_cancellation() -> None:
    """With unsaturated voltage, closed-loop acceleration equals v_accel."""
    arm = ArmParams()
    motor = MotorParams()
    ctrl = ComputedTorqueController(arm, motor, saturate=False)
    q = np.array([0.4, -0.7])
    dq = np.array([0.9, -0.5])
    q_d = np.array([0.6, 0.2])
    dq_d = np.array([0.1, -0.2])
    ddq_d = np.array([0.3, -0.4])

    voltage = ctrl.compute_voltage(q, dq, q_d, dq_d, ddq_d)
    ddq_actual = forward_dynamics(q, dq, voltage, arm, motor, saturate_voltage=False)
    ddq_expected = (
        ddq_d + np.diag([arm.kp1, arm.kp2]) @ (q_d - q) + np.diag([arm.kd1, arm.kd2]) @ (dq_d - dq)
    )
    assert np.allclose(ddq_actual, ddq_expected, atol=1e-11)


def test_pole_placement_nonlinear_regulation_converges() -> None:
    """Nonlinear closed-loop under pole-placement K converges from perturbation."""
    _, _, final_norm = simulate_pole_placement_regulation()
    assert final_norm < 1e-6


def test_position_regulation_settling_and_error() -> None:
    """POS mode meets < 2.0 s settling and < 0.01 rad steady-state error."""
    res = simulate_position_regulation(q_target=(0.5, 0.5), kp=(50.0, 50.0), kd=(10.0, 10.0))
    assert res.settling_time_s < 2.0
    assert res.steady_state_error_rad < 0.01

    kd_crit = 2.0 * np.sqrt(50.0)
    res_crit = simulate_position_regulation(
        q_target=(0.5, 0.5), kp=(50.0, 50.0), kd=(kd_crit, kd_crit)
    )
    assert res_crit.max_overshoot_rad < 1e-4


def test_circular_trajectory_tracking_high_and_low_gain() -> None:
    """CIRCLE mode meets < 1% amplitude and < 0.02 rad phase error at (50, 10)."""
    hi = simulate_joint_circle(kp=(50.0, 50.0), kd=(10.0, 10.0))
    assert hi.steady_amplitude_error_pct < 1.0
    assert hi.steady_phase_error_rad < 0.02

    lo = simulate_joint_circle(kp=(5.0, 5.0), kd=(1.0, 1.0))
    assert lo.peak_q1_rad > 0.35
    assert lo.steady_amplitude_error_pct > hi.steady_amplitude_error_pct


def test_cartesian_circle_ik_accuracy() -> None:
    """Verify end_effector_circle.ino Cartesian circle path IK round-trip accuracy."""
    max_err = simulate_cartesian_circle()
    assert max_err < 1e-12


def test_arduino_sketches_use_consistent_quadrature_scale() -> None:
    """All 4 Arduino sketches use the 44 CPR * 44.727 gear ratio encoder constant."""
    sketch_dir = REPO_ROOT / "arduino"
    ino_files = sorted(sketch_dir.glob("*/*.ino"))
    assert len(ino_files) == 4
    for path in ino_files:
        text = path.read_text(encoding="utf-8")
        assert "44.0" in text and "44.727" in text, f"{path.name} missing 44.0 * 44.727"
