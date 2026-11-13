# Contributing to 2-dof-robotic-arm-control-system

Bug reports, simulation verification improvements, and documentation fixes are welcome. For larger changes, open an issue first so we can agree on the scope.

## Before you start

- Search [existing issues](https://github.com/andrew-abdelmalak/2-dof-robotic-arm-control-system/issues) to avoid duplicates.
- By participating, you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md).

## Set up your environment

Requirements: Python 3.11+ with `uv`, `arduino-cli` (`arduino:avr` core), GNU Octave 9+ (or MATLAB), and TeX Live (`latexmk`) if editing the paper.

```bash
git clone https://github.com/andrew-abdelmalak/2-dof-robotic-arm-control-system.git
cd 2-dof-robotic-arm-control-system
uv sync
```

## Run the checks

```bash
uv run ruff check .
uv run ruff format --check .
uv run pytest -v
uv run python -m arm_control
```

To verify the MATLAB scripts in GNU Octave and compile all four Arduino sketches:

```bash
octave --no-gui --eval "addpath('matlab'); check_controllability; check_nonlinear_controllability_lie"
for s in arduino/end_effector_circle arduino/joint_space arduino/nonlinear_trajectory arduino/nonlinear_trajectory_alt; do
  arduino-cli compile --fqbn arduino:avr:mega "$s"
done
```

CI runs these checks on every pull request. A pull request with failing checks cannot be merged.

## Make your change

1. Create a branch from `main`: `git switch -c fix/short-description`.
2. Keep each change focused on one problem.
3. Add or update tests in `tests/` for control, kinematics, or dynamics changes.
4. Keep tuned hardware constants in `arduino/` untouched unless hardware bench testing confirms a new calibration.

## Commit messages

Use short imperative subjects (for example, `fix(arduino): guard AVR pin mapping` or `test(dynamics): verify skew symmetry`).

## Open a pull request

1. Push your branch and open a pull request against `main`.
2. Fill in the [pull request template](pull_request_template.md), listing the checks you ran locally.
3. A maintainer will review changes before merging.

## Report a bug or request a feature

Use the [issue forms](https://github.com/andrew-abdelmalak/2-dof-robotic-arm-control-system/issues/new/choose). For security issues, follow [SECURITY.md](SECURITY.md) instead. Never report vulnerabilities in a public issue.

## License

By contributing, you agree that your contributions are licensed under the project's [MIT License](../LICENSE).
