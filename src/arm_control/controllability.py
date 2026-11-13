"""Linear and nonlinear (Lie-bracket) controllability, pole placement, and Lyapunov tools."""

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.linalg import solve_continuous_lyapunov
from scipy.signal import place_poles

from arm_control.dynamics import coriolis_matrix, forward_dynamics, gravity_vector, mass_matrix
from arm_control.params import ArmParams, MotorParams


@dataclass(frozen=True)
class LieGridResult:
    """Summary of nonlinear Lie-bracket controllability over a workspace grid."""

    total_points: int
    singular_points: int
    min_singular_value: float
    all_full_rank: bool


def _state_rhs(
    x: NDArray[np.float64], u: NDArray[np.float64], arm: ArmParams, motor: MotorParams
) -> NDArray[np.float64]:
    q = x[:2]
    dq = x[2:]
    ddq = forward_dynamics(q, dq, u, arm, motor, saturate_voltage=False)
    return np.concatenate([dq, ddq])


def _numeric_jacobian(
    fn: Callable[[NDArray[np.float64]], NDArray[np.float64]],
    x: NDArray[np.float64],
    eps: float = 1e-6,
) -> NDArray[np.float64]:
    f0 = fn(x)
    jac = np.zeros((f0.size, x.size), dtype=np.float64)
    for idx in range(x.size):
        dx = np.zeros_like(x)
        dx[idx] = eps
        jac[:, idx] = (fn(x + dx) - fn(x - dx)) / (2.0 * eps)
    return jac


def linearize_at_origin(
    arm: ArmParams, motor: MotorParams, eps: float = 1e-6
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Linearize dx/dt = f(x, u) around equilibrium x_eq = 0, u_eq = 0."""
    x_eq = np.zeros(4, dtype=np.float64)
    u_eq = np.zeros(2, dtype=np.float64)
    a_mat = _numeric_jacobian(lambda x: _state_rhs(x, u_eq, arm, motor), x_eq, eps=eps)
    b_mat = _numeric_jacobian(lambda u: _state_rhs(x_eq, u, arm, motor), u_eq, eps=eps)
    return a_mat, b_mat


def kalman_controllability_rank(
    a_mat: NDArray[np.float64], b_mat: NDArray[np.float64], tol: float = 1e-9
) -> int:
    """Compute rank of the Kalman controllability matrix [B, AB, ..., A^(n-1)B]."""
    n = a_mat.shape[0]
    blocks = [b_mat]
    cur = b_mat
    for _ in range(1, n):
        cur = a_mat @ cur
        blocks.append(cur)
    ctrl_mat = np.hstack(blocks)
    return int(np.linalg.matrix_rank(ctrl_mat, tol=tol))


def kalman_observability_rank(
    a_mat: NDArray[np.float64], c_mat: NDArray[np.float64], tol: float = 1e-9
) -> int:
    """Compute rank of the Kalman observability matrix [C; CA; ...; CA^(n-1)]."""
    n = a_mat.shape[0]
    blocks = [c_mat]
    cur = c_mat
    for _ in range(1, n):
        cur = cur @ a_mat
        blocks.append(cur)
    obsv_mat = np.vstack(blocks)
    return int(np.linalg.matrix_rank(obsv_mat, tol=tol))


def compute_pole_placement(
    a_mat: NDArray[np.float64],
    b_mat: NDArray[np.float64],
    poles: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Compute state-feedback gain K such that eig(A - B*K) matches poles."""
    res = place_poles(a_mat, b_mat, poles)
    return np.asarray(res.gain_matrix, dtype=np.float64)


def solve_lyapunov_certificate(
    a_cl: NDArray[np.float64], q_mat: NDArray[np.float64] | None = None
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Solve A_cl^T P + P A_cl = -Q and return (P, eigvals(P))."""
    n = a_cl.shape[0]
    q_lyap = np.eye(n, dtype=np.float64) if q_mat is None else q_mat
    p_mat = solve_continuous_lyapunov(a_cl.T, -q_lyap)
    p_mat = 0.5 * (p_mat + p_mat.T)
    eigvals = np.linalg.eigvalsh(p_mat)
    return p_mat, eigvals


def _drift_vector_field(x: NDArray[np.float64], arm: ArmParams) -> NDArray[np.float64]:
    q = x[:2]
    dq = x[2:]
    m_mat = mass_matrix(q, arm)
    c_mat = coriolis_matrix(q, dq, arm)
    g_vec = gravity_vector(q, arm)
    qdd = np.linalg.solve(m_mat, -c_mat @ dq - g_vec)
    return np.concatenate([dq, qdd])


def _input_vector_field(
    x: NDArray[np.float64], arm: ArmParams, motor: MotorParams, channel: int
) -> NDArray[np.float64]:
    q = x[:2]
    m_mat = mass_matrix(q, arm)
    g_acc = np.linalg.solve(m_mat, motor.b_matrix)
    return np.array([0.0, 0.0, g_acc[0, channel], g_acc[1, channel]], dtype=np.float64)


def nonlinear_lie_controllability(
    x: NDArray[np.float64], arm: ArmParams, motor: MotorParams, eps: float = 1e-6
) -> tuple[int, NDArray[np.float64]]:
    """Evaluate rank and singular values of [g1, g2, [f, g1], [f, g2]] at state x."""

    def f_fn(state: NDArray[np.float64]) -> NDArray[np.float64]:
        return _drift_vector_field(state, arm)

    def g1_fn(state: NDArray[np.float64]) -> NDArray[np.float64]:
        return _input_vector_field(state, arm, motor, 0)

    def g2_fn(state: NDArray[np.float64]) -> NDArray[np.float64]:
        return _input_vector_field(state, arm, motor, 1)

    fx = f_fn(x)
    g1x = g1_fn(x)
    g2x = g2_fn(x)

    df = _numeric_jacobian(f_fn, x, eps=eps)
    dg1 = _numeric_jacobian(g1_fn, x, eps=eps)
    dg2 = _numeric_jacobian(g2_fn, x, eps=eps)

    lie_1 = dg1 @ fx - df @ g1x
    lie_2 = dg2 @ fx - df @ g2x
    dist = np.column_stack([g1x, g2x, lie_1, lie_2])
    svals = np.linalg.svd(dist, compute_uv=False)
    rank = int(np.sum(svals > 1e-6))
    return rank, svals


def evaluate_nonlinear_controllability_grid(
    arm: ArmParams, motor: MotorParams, n_grid: int = 120
) -> LieGridResult:
    """Evaluate Lie-bracket rank across an n_grid x n_grid mesh on [-pi, pi]^2."""
    grid = np.linspace(-np.pi, np.pi, n_grid)
    singular_count = 0
    min_sv = float("inf")
    for q1 in grid:
        for q2 in grid:
            x = np.array([q1, q2, 0.0, 0.0], dtype=np.float64)
            rank, svals = nonlinear_lie_controllability(x, arm, motor)
            min_sv = min(min_sv, float(svals[-1]))
            if rank < 4:
                singular_count += 1
    total = n_grid * n_grid
    return LieGridResult(
        total_points=total,
        singular_points=singular_count,
        min_singular_value=min_sv,
        all_full_rank=(singular_count == 0),
    )
