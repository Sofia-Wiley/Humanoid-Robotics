# pyCub Gaze Tracking

A 2-DOF gaze controller for the iCub humanoid robot in
[pyCub](https://rustlluk.github.io/pyCub). Course assignment from CTU's
Humanoid Robotics module (Spring 2026).

A green ball gets random forces applied to it at random intervals; the
robot's neck (yaw + pitch) has to keep the ball centered in its gaze
across a 20-second trial.


### 1. Predict the ball
Reads the ball's current position and velocity, and projects forward by
2–8 simulation steps depending on ball speed (faster ball → larger
lookahead). This compensates for the small lag between sensing and
actuation; without it the controller visibly trails fast-moving targets.

### 2. Compute the desired direction
The vector from the head to the predicted ball position is normalized in
world frame, then transformed into the chest frame using a precomputed
rotation-matrix transpose (the chest's orientation is captured once at
init).

### 3. Solve the 2-DOF IK
Given the desired direction in chest frame, the target neck joint angles
fall out of two `atan2` calls — one for pitch, one for yaw. Each is
clamped to the iCub's joint limits before being commanded.

### 4. Command the joints
`neck_yaw` and `neck_pitch` are sent the target angles with a high
velocity, which makes the neck snap to target each step rather than
ramping in.

### 5. Visualize
A debug overlay renders two lines from the head: green for the current
gaze direction, red for the true line to the ball. Useful for seeing
tracking error in real time.

### 6. Score
Each step, the angular error between gaze and true ball direction is
computed. After the trial, mean and max error are printed against a
rubric-style scoring threshold.

## Stress test

The trial is set up to be hard: every 0.5–2.0 s a random external force
is applied to the ball in a random horizontal direction, with magnitude
sampled from one of three tiers (soft / medium / hard). The controller
has to handle sudden direction changes and high speeds without losing
the target.

## Course
Humanoid Robotics, Czech Technical University in Prague,
Electrical Engineering Exchange Program, Spring 2026.