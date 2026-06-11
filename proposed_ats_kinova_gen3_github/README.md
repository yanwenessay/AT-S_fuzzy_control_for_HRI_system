# Proposed AT-S Impedance Control on Kinova Gen3

This folder provides a lightweight GitHub-ready entry point for running the proposed adaptive Takagi-Sugeno (AT-S) compensated impedance controller on a Kinova Gen3 7-DOF robot.

The controller implementation is:

```text
../Response1/comparison_proposed_method_ats.py
```

It combines:

- Equivalent end-effector impedance control.
- AT-S fuzzy adaptive compensation.
- Force-aware attenuation based on weighted external joint torque.
- Kinova low-level torque control through the Kortex Python API.

## 1. Hardware And Software Requirements

Target platform:

- Kinova Gen3 7-DOF arm.
- Ubuntu/Linux host connected to the user's robot network.
- Python 3.8 or newer.
- Kinova Kortex Python API installed and importable as `kortex_api`.
- Kinova API example helper `utilities.py` available in the Python path.

Python packages:

```bash
pip install -r requirements.txt
```

The Kinova Kortex Python API is usually installed from Kinova's official Kortex API package, not from PyPI. After installing it, verify:

```bash
python3 -c "import kortex_api; print('kortex_api OK')"
```

## 2. Repository Layout Expected By This Launcher

This launcher assumes the following project layout:

```text
ATS_impedance_control/
+-- Response1/
|   +-- comparison_proposed_method_ats.py
+-- ats_imcontrol_sin_cos_jo.py
+-- Kinematic_fcn.py
+-- DiscreteIntegrator.py
+-- ts_fuzzy_output.py
+-- proposed_ats_kinova_gen3_github/
    +-- README.md
    +-- requirements.txt
    +-- setup_env.sh
    +-- run_kinova_gen3.sh
    +-- scripts/
        +-- check_environment.py
        +-- run_proposed_ats.py
```

If your `utilities.py` is not in the project root, copy it from the Kinova Kortex Python examples folder or add its folder to `PYTHONPATH`.

## 3. Configure Robot Connection

Copy the example environment file:

```bash
cp .env.example .env
```

Edit `.env` with your own robot network settings:

```bash
KINOVA_IP=<YOUR_KINOVA_ROBOT_IP>
KINOVA_USERNAME=<YOUR_KINOVA_USERNAME>
KINOVA_PASSWORD=<YOUR_KINOVA_PASSWORD>
```

Do not publish private robot IP addresses, usernames, or passwords in a public repository.

## 4. Environment Check

From this folder:

```bash
python3 scripts/check_environment.py
```

The script checks:

- Python version.
- Required Python packages.
- `kortex_api` import.
- Required local controller files.
- Kinova `utilities.py` availability.

## 5. Run On Kinova Gen3

Use the launcher after creating `.env`:

```bash
./run_kinova_gen3.sh
```

Equivalent direct command:

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD>
```

Optional: skip the initial joint move if the arm is already at a safe start pose:

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD> --skip-initial-move
```

The program will move to the initial pose first unless `--skip-initial-move` is used. After that, it waits for keyboard input:

```text
s = start torque control
q = quit
```

No torque command is sent until `s` is entered.

## 6. Data Output

The controller anchors its working directory beside `comparison_proposed_method_ats.py`. Typical generated folders are:

```text
Response1/impedance_improved_data/
Response1/proposed_method_plots/
```

The recorded data include joint tracking error, external torque, control torque, AT-S compensation, and related statistics.

## 7. Safety Notes

Before every real robot run:

- Keep the emergency stop within reach.
- Clear the workspace.
- Start with low-speed, no-contact tests.
- Verify that the robot is in a safe pose before pressing `s`.
- Do not run low-level torque control through remote SSH unless a local operator can stop the robot immediately.
- Confirm that Kinova Web App or another process is not holding an incompatible servoing mode.

See `SAFETY.md` for a more explicit checklist.

## 8. Common Problems

### `ModuleNotFoundError: No module named 'kortex_api'`

Install the Kinova Kortex Python API and activate the correct Python environment.

### `ModuleNotFoundError: No module named 'utilities'`

Make sure Kinova's `utilities.py` is in the project root or included in `PYTHONPATH`.

### `WRONG_SERVOING_MODE`

Another process may have changed the arm servoing mode. Stop other control scripts, recover the robot in Kinova Web App, then rerun the controller.

### Robot does not move after launching

This is expected. The script waits for `s` before torque control starts.

## 9. Citation Placeholder

If you use this code in academic work, please cite the associated paper after publication.

```bibtex
@article{yanwen_ats_impedance_kinova,
  title   = {Adaptive Takagi-Sugeno Impedance Control for Human-Robot Interaction},
  author  = {Yanwen et al.},
  journal = {To be updated},
  year    = {2026}
}
```

