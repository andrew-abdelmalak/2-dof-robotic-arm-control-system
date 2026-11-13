"""Physical, motor, and controller parameters matching matlab/init_params.m."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class ArmParams:
    """Link and motor mechanical parameters from matlab/init_params.m."""

    l1: float = 0.200
    l2: float = 0.200
    a1: float = 51.62e-3
    a2: float = 5.04e-3
    m1_link: float = 5.14e-3
    m2_link: float = 5.44e-3
    i1_com: float = 80234.34e-9
    i2_com: float = 14850.27e-9
    m_m1: float = 47.45e-3
    m_m2: float = 47.45e-3
    i_m1_com: float = 7024.83e-9
    i_m2_com: float = 7024.83e-9
    g: float = 0.0
    kp1: float = 50.0
    kp2: float = 50.0
    kd1: float = 10.0
    kd2: float = 10.0

    @property
    def m1_total(self) -> float:
        """Total mass of link 1 and motor 1 [kg]."""
        return self.m1_link + self.m_m1

    @property
    def m2_total(self) -> float:
        """Total mass of link 2 and motor 2 [kg]."""
        return self.m2_link + self.m_m2

    @property
    def i1_total(self) -> float:
        """Total inertia around joint 1 including reflected motor 2 inertia [kg*m^2]."""
        return self.i1_com + self.i_m1_com + self.i_m2_com

    @property
    def i2_total(self) -> float:
        """Total inertia around joint 2 [kg*m^2]."""
        return self.i2_com + self.i_m2_com

    def to_matlab_vector(self) -> NDArray[np.float64]:
        """Return the 17x1 params vector in the exact order expected by Simulink."""
        return np.array(
            [
                self.l1,
                self.l2,
                self.a1,
                self.a2,
                self.m1_total,
                self.m2_total,
                self.i1_total,
                self.i2_total,
                self.g,
                self.kp1,
                self.kp2,
                self.kd1,
                self.kd2,
                self.m_m1,
                self.m_m2,
                self.i_m1_com,
                self.i_m2_com,
            ],
            dtype=np.float64,
        )


@dataclass(frozen=True)
class MotorParams:
    """Electromechanical DC motor parameters from matlab/init_params.m."""

    b1: float = 0.8
    b2: float = 0.8
    d1: float = 0.01
    d2: float = 0.01
    v_max: float = 12.0

    @property
    def b_matrix(self) -> NDArray[np.float64]:
        """Diagonal voltage-to-torque gain matrix B_mot [N*m/V]."""
        return np.diag([self.b1, self.b2])

    @property
    def d_matrix(self) -> NDArray[np.float64]:
        """Diagonal viscous/back-EMF damping matrix D_mot [N*m*s/rad]."""
        return np.diag([self.d1, self.d2])

    def to_matlab_vector(self) -> NDArray[np.float64]:
        """Return the 4x1 motor vector [b1, b2, d1, d2]."""
        return np.array([self.b1, self.b2, self.d1, self.d2], dtype=np.float64)
