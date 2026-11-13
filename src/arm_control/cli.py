"""CLI entry point printing deterministic control verification and simulation results."""

import numpy as np

from arm_control.controllability import (
    compute_pole_placement,
    evaluate_nonlinear_controllability_grid,
    kalman_controllability_rank,
    kalman_observability_rank,
    linearize_at_origin,
    solve_lyapunov_certificate,
)
from arm_control.dynamics import mass_matrix
from arm_control.params import ArmParams, MotorParams
from arm_control.simulation import (
    simulate_cartesian_circle,
    simulate_joint_circle,
    simulate_pole_placement_regulation,
    simulate_position_regulation,
)


def main() -> None:
    """Run analytical verification and closed-loop simulations and print summary."""
    arm = ArmParams()
    motor = MotorParams()
    m0 = mass_matrix(np.zeros(2), arm)
    a_mat, b_mat = linearize_at_origin(arm, motor)
    c_mat = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])

    rank_co = kalman_controllability_rank(a_mat, b_mat)
    rank_ob = kalman_observability_rank(a_mat, c_mat)

    desired_poles = np.array([-4.0, -5.0, -6.0, -7.0])
    k_gain = compute_pole_placement(a_mat, b_mat, desired_poles)
    a_cl = a_mat - b_mat @ k_gain
    _, eig_p = solve_lyapunov_certificate(a_cl)

    _, _, reg_norm = simulate_pole_placement_regulation(arm, motor)
    lie_res = evaluate_nonlinear_controllability_grid(arm, motor, n_grid=120)

    pos_res = simulate_position_regulation(q_target=(0.5, 0.5), kp=(50.0, 50.0), kd=(10.0, 10.0))
    circ_hi = simulate_joint_circle(kp=(50.0, 50.0), kd=(10.0, 10.0))
    circ_lo = simulate_joint_circle(kp=(5.0, 5.0), kd=(1.0, 1.0))
    cart_err = simulate_cartesian_circle()

    print("=== 2-DOF Robotic Arm Control Verification ===")
    print(f"m1_total = {arm.m1_total:.5f} kg, m2_total = {arm.m2_total:.5f} kg")
    print(f"I1_total = {arm.i1_total:.6e} kg*m^2, I2_total = {arm.i2_total:.6e} kg*m^2")
    print(f"det(M(0)) = {np.linalg.det(m0):.6e} kg^2*m^4")
    print(f"Linear controllability rank = {rank_co}/4, observability rank = {rank_ob}/4")
    print(f"Lyapunov P eigenvalues = {np.array2string(eig_p, precision=5)}")
    print(f"Pole-placement nonlinear final ||x|| (t=5s) = {reg_norm:.4e}")
    print(
        f"Lie-bracket grid ({lie_res.total_points} pts): "
        f"singular={lie_res.singular_points}, min_sv={lie_res.min_singular_value:.4f}"
    )
    print(
        f"POS step (0.5, 0.5) [Kp=50, Kd=10]: "
        f"ts={pos_res.settling_time_s:.2f}s, overshoot={pos_res.max_overshoot_rad:.4e} rad, "
        f"ss_err={pos_res.steady_state_error_rad:.4e} rad"
    )
    print(
        f"CIRCLE [Kp=50, Kd=10]: amp_err={circ_hi.steady_amplitude_error_pct:.3f}%, "
        f"phase_err={circ_hi.steady_phase_error_rad:.5f} rad"
    )
    print(
        f"CIRCLE [Kp=5, Kd=1]: peak_q1={circ_lo.peak_q1_rad:.4f} rad "
        f"at t={circ_lo.peak_q1_time_s:.2f}s, "
        f"amp_err={circ_lo.steady_amplitude_error_pct:.2f}%"
    )
    print(f"Cartesian IK circle max round-trip error = {cart_err:.4e} m")


if __name__ == "__main__":
    main()
