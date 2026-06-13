# Safety Checklist

This controller uses Kinova low-level torque control. Treat every run as a real robot experiment.

Before running:

- Keep the emergency stop within reach.
- Clear the workspace.
- Use a trained local operator.
- Verify that the robot is not near joint limits.
- Confirm that no other process is controlling the robot.
- Do not press `s` until the robot is stable and the area is clear.

Stop immediately if the arm accelerates unexpectedly, contacts the environment, reports repeated servoing errors, or shows abnormal torque behavior.
