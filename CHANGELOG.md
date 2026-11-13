# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Python verification and closed-loop ODE simulation package (`src/arm_control/`) with CLI (`python -m arm_control`) and 25 unit and integration tests (`tests/test_arm_control.py`).
- Tracked LaTeX paper source (`paper/Nonlinear_Control_2DOF_Arm.tex`, `paper/IEEEtran.cls`, and extracted figures) with automated compilation in CI.
- Community health files (`CITATION.cff`, `CONTRIBUTING.md`, `SECURITY.md`, `CODE_OF_CONDUCT.md`, `SUPPORT.md`, issue forms, PR template, and Dependabot configuration).
- Computed-torque control architecture SVG diagram (`docs/figures/control_architecture.svg`) and social preview image (`assets/social-preview.svg`, `assets/social-preview.png`).

### Changed

- Added `#if defined(ARDUINO_ARCH_ESP32)` compatibility shims to `end_effector_circle.ino` and `nonlinear_trajectory_alt.ino` so all four Arduino sketches compile for both AVR (`arduino:avr:mega`) and ESP32 targets.
- Made `matlab/check_controllability.m` and `matlab/check_nonlinear_controllability_lie.m` portable across MATLAB and GNU Octave without requiring commercial toolboxes, and removed double-added motor masses/inertias from `plant_rhs` and `dyn_terms`.
- Aligned the IEEE paper, `README.md`, and firmware documentation on author names, encoder quadrature counts (`1968` counts/rev), control loop rate (`Ts >= 2 ms`), and Simulink trajectory plots.
- Replaced the placeholder Ruff-only workflow with full CI (`ruff`, `pytest`, GNU Octave `.m` verification, `arduino-cli` builds, and LaTeX paper build) using SHA-pinned actions.

## [1.0.0] - 2026-03-18

### Added

- Euler-Lagrange 2-DOF manipulator dynamics, linear pole-placement/Lyapunov check (`matlab/check_controllability.m`), Lie-bracket controllability check (`matlab/check_nonlinear_controllability_lie.m`), and Simulink Stateflow model (`matlab/Advanced_trial.slx`).
- Four embedded C++ control sketches under `arduino/` (`joint_space`, `end_effector_circle`, `nonlinear_trajectory`, `nonlinear_trajectory_alt`).
- Compiled IEEE-style project report PDF and figures in `docs/`.

[Unreleased]: https://github.com/andrew-abdelmalak/2-dof-robotic-arm-control-system/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/andrew-abdelmalak/2-dof-robotic-arm-control-system/releases/tag/v1.0.0
