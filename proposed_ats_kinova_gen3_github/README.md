# AT-S Impedance Control for Kinova Gen3

This repository provides a self-contained Python implementation of the proposed adaptive Takagi-Sugeno (AT-S) compensated impedance controller for a Kinova Gen3 7-DOF robotic arm.

The package is designed for real-robot experiments using the Kinova Kortex Python API. It includes the main AT-S impedance controller, the base variable-impedance torque-control framework, kinematics, fuzzy logic utilities, and a safe launcher that requires users to provide their own robot network settings.

## What Is Included

The main controller is:

```text
src/comparison_proposed_method_ats.py
```

The controller combines:

- Equivalent end-effector impedance control.
- AT-S fuzzy adaptive compensation.
- Force-aware attenuation using weighted external joint torque.
- Conservative model-based dynamics compensation.
- Kinova Gen3 low-level torque control through the Kortex API.
- Data logging and plotting for tracking error, external torque, control torque, and AT-S compensation.

## Repository Layout

```text
.
+-- README.md
+-- SAFETY.md
+-- requirements.txt
+-- setup_env.sh
+-- run_kinova_gen3.sh
+-- .env.example
+-- .gitignore
+-- scripts/
|   +-- check_environment.py
|   +-- run_proposed_ats.py
+-- src/
    +-- comparison_proposed_method_ats.py
    +-- ats_imcontrol_sin_cos_jo.py
    +-- Kinematic_fcn.py
    +-- DiscreteIntegrator.py
    +-- ts_fuzzy_output.py
    +-- fuzzy_membership_fcn.py
    +-- fuzzyoutput.py
    +-- control_main.py
    +-- utilities.py
```

## Hardware And Software Requirements

Hardware:

- Kinova Gen3 7-DOF arm.
- A Linux/Ubuntu computer connected to the robot network.
- A local operator with access to the emergency stop.

Software:

- Python 3.8 or newer.
- Official Kinova Kortex Python API installed and importable as `kortex_api`.
- Python packages listed in `requirements.txt`.

Install Python packages:

```bash
pip install -r requirements.txt
```

The Kinova Kortex API is normally installed from Kinova's official API package, not from PyPI. After installation, verify it with:

```bash
python3 -c "import kortex_api; print('kortex_api OK')"
```

## Configure Robot Connection

Copy the example environment file:

```bash
cp .env.example .env
```

Edit `.env` with your own Kinova robot settings:

```bash
KINOVA_IP=<YOUR_KINOVA_ROBOT_IP>
KINOVA_USERNAME=<YOUR_KINOVA_USERNAME>
KINOVA_PASSWORD=<YOUR_KINOVA_PASSWORD>
```

This repository intentionally does not contain a default robot IP address, username, or password. Do not commit private robot network information to GitHub.

## Check The Environment

Run:

```bash
python3 scripts/check_environment.py
```

The checker verifies:

- Required source files in `src/`.
- Required Python packages.
- `kortex_api` availability.
- Bundled Kinova connection helper `src/utilities.py`.

## Run The Controller

Recommended launcher:

```bash
./run_kinova_gen3.sh
```

Direct Python command:

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD>
```

If the robot is already in a safe initial pose, the initial move can be skipped:

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD> --skip-initial-move
```

After launching, the program waits for keyboard input:

```text
s = start low-level torque control
q = quit
```

No torque command is sent until `s` is entered.

## Output Data

The controller uses `src/` as the runtime working directory. Typical generated folders are:

```text
src/impedance_improved_data/
src/proposed_method_plots/
```

These outputs are ignored by `.gitignore` and should not be committed unless intentionally publishing experiment data.

## Safety Notes

This code sends low-level torque commands to a real robot. Before running:

- Read `SAFETY.md`.
- Keep the emergency stop within reach.
- Make sure the workspace is clear.
- Start with no-contact tests.
- Do not run the robot unattended.
- Do not press `s` unless the arm is stable and a local operator is ready.

If the robot accelerates unexpectedly, contacts the environment, or repeatedly reports servoing-mode errors, stop immediately and recover the robot using the Kinova Web App or the standard Kinova recovery workflow.

## Common Issues

### `ModuleNotFoundError: No module named 'kortex_api'`

Install the official Kinova Kortex Python API and make sure the Python environment used to run this repository can import it.

### `Missing Kinova connection settings`

Create `.env` from `.env.example` and fill in your own robot IP, username, and password.

### `WRONG_SERVOING_MODE`

Another process may be controlling the robot or the robot may not be in the expected low-level servoing mode. Stop other control programs, recover the robot, and restart this script from a clean terminal.

### Baseline database not found

The controller can run without a no-contact baseline database. If no baseline file is present, baseline scaling is disabled and the controller continues. You can also run with:

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD> --no-baseline
```

## Citation

If this code is used in academic work, please cite the associated paper after publication.

```bibtex
@article{yanwen_ats_impedance_kinova,
  title   = {Adaptive Takagi-Sugeno Impedance Control for Human-Robot Interaction},
  author  = {Yanwen et al.},
  journal = {To be updated},
  year    = {2026}
}
```
