"""Numerical verification and closed-loop simulation for the 2-DOF robotic arm."""

from arm_control.controllability import (
    compute_pole_placement,
    evaluate_nonlinear_controllability_grid,
    kalman_controllability_rank,
    kalman_observability_rank,
    linearize_at_origin,
    nonlinear_lie_controllability,
    solve_lyapunov_certificate,
)
from arm_control.controllers import (
    ComputedTorqueController,
    cubic_polynomial_trajectory,
)
from arm_control.dynamics import (
    actuator_torque,
    coriolis_matrix,
    forward_dynamics,
    forward_kinematics,
    gravity_vector,
    inverse_kinematics,
    mass_matrix,
)
from arm_control.params import ArmParams, MotorParams
from arm_control.simulation import (
    simulate_cartesian_circle,
    simulate_joint_circle,
    simulate_pole_placement_regulation,
    simulate_position_regulation,
)

__all__ = [
    "ArmParams",
    "ComputedTorqueController",
    "MotorParams",
    "actuator_torque",
    "compute_pole_placement",
    "coriolis_matrix",
    "cubic_polynomial_trajectory",
    "evaluate_nonlinear_controllability_grid",
    "forward_dynamics",
    "forward_kinematics",
    "gravity_vector",
    "inverse_kinematics",
    "kalman_controllability_rank",
    "kalman_observability_rank",
    "linearize_at_origin",
    "mass_matrix",
    "nonlinear_lie_controllability",
    "simulate_cartesian_circle",
    "simulate_joint_circle",
    "simulate_pole_placement_regulation",
    "simulate_position_regulation",
    "solve_lyapunov_certificate",
]
