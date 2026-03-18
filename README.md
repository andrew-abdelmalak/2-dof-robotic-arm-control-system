# Non-Linear Control of a 2-DOF Robotic Arm

Complete design, simulation, and hardware implementation of a non-linear controlled two-degree-of-freedom (2-DOF) planar robotic arm. The project covers mathematical modeling using the Euler–Lagrange formulation, non-linear stability analysis via Lyapunov methods, feedback linearization control synthesis, MATLAB/Simulink simulation, and real-time embedded implementation on an Arduino microcontroller.

## Project Goals

- Model a planar 2-DOF manipulator using Euler–Lagrange dynamics.
- Design a feedback linearization controller with Lyapunov stability guarantees.
- Validate trajectory tracking in MATLAB/Simulink and on physical hardware.

## Repository Structure

```
.
|-- matlab/
|   |-- Advanced_trial.slx
|   |-- init_params.m
|   |-- check_controllability.m
|   `-- check_nonlinear_controllability_lie.m
`-- arduino/
    |-- end_effector_circle/
    |   `-- end_effector_circle.ino
    |-- joint_space/
    |   `-- joint_space.ino
    |-- nonlinear_trajectory/
    |   `-- nonlinear_trajectory.ino
    `-- nonlinear_trajectory_alt/
        `-- nonlinear_trajectory_alt.ino
```

## MATLAB Workflow

1. Open MATLAB in the `matlab/` directory.
2. Run `init_params.m` to initialize arm and motor parameters.
3. Execute:
   - `check_controllability.m`
   - `check_nonlinear_controllability_lie.m`
4. Open `Advanced_trial.slx` for simulation and controller testing.

## Arduino Workflow

1. Open one sketch folder under `arduino/` in Arduino IDE.
2. Select the board and COM port matching your hardware.
3. Upload one of the control sketches:
   - `end_effector_circle.ino`
   - `joint_space.ino`
   - `nonlinear_trajectory.ino`
   - `nonlinear_trajectory_alt.ino`

`nonlinear_trajectory_alt` is preserved as an alternate trajectory/controller variant.

## Notes

- This repository intentionally keeps multiple controller implementations to compare behavior across trajectory spaces and tuning choices.
- Update pin definitions and hardware constants in each sketch before deployment if your wiring differs.

## Authors

| Name | Affiliation |
|------|-------------|
| **Ahmed Mostafa** | Mechatronics Engineering, GUC |
| **Andrew Khalil** | Mechatronics Engineering, GUC |
| **Hazim Ashraf** | Mechatronics Engineering, GUC |
| **Mazen Amr** | Mechatronics Engineering, GUC |
| **Samir Sameh** | Mechatronics Engineering, GUC |
| **Youssef Youssry** | Mechatronics Engineering, GUC |

## Report

The full project report is available in [`docs/Nonlinear_Control_2DOF_Arm.pdf`](docs/Nonlinear_Control_2DOF_Arm.pdf).

## License

Released under the MIT License. See `LICENSE`.
