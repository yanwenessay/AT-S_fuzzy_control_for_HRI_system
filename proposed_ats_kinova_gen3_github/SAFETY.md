# Safety Checklist

This controller uses Kinova low-level torque control. Treat every run as a real robot experiment, not as a normal Python demo.

## Before Running

- The robot workspace is clear.
- A trained operator is beside the robot.
- The emergency stop is reachable.
- The arm is not near joint limits.
- The end effector is not in contact with a person or fixture.
- No other process is controlling the robot.
- The Kinova Web App shows no active fault.

## Startup

- Run the environment check first.
- Let the script move to the initial pose unless you intentionally use `--skip-initial-move`.
- Do not press `s` until the robot is stable.
- Keep one hand near the emergency stop during the first seconds of torque mode.

## Stop Conditions

Stop immediately if:

- The arm accelerates unexpectedly.
- The arm hits a joint limit or fixture.
- The measured external torque grows abnormally.
- The terminal repeatedly reports servoing mode or communication errors.

## Recovery

After an emergency stop or fault:

- Do not immediately rerun the script.
- Recover the robot in Kinova Web App.
- Confirm servoing mode and faults are cleared.
- Restart the Python process from a clean terminal.

