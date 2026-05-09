# pyCub Grasping

Two-task grasping pipeline for the iCub humanoid robot in
[pyCub](https://rustlluk.github.io/pyCub). Course assignment from CTU's
Humanoid Robotics module (Spring 2026).

The robot has to find a green ball on a table and grasp it across nine
trials (three table positions × three repetitions per position).

### Task 1 — Find the ball
Implemented in `find_the_ball`. The robot's left-eye RGB image is
color-thresholded with `cv2.inRange` against a green range, and the
centroid of the resulting mask gives the ball's `(u, v)` pixel coordinates.

### Task 2 — Grasp
Implemented in `grasp`. From the pixel center, the routine:

1. **Recovers 3D ball position.** The 2D pixel is unprojected to a 3D
   surface point using the simulator's depth image, then offset along the
   pupil-to-ball ray by the ball's radius (≈ 25 mm) to get the center.
2. **Aligns gaze.** The eyes tilt and pan to point at the ball using a
   simple atan2-based pointing controller, clamped to joint limits.
3. **Selects the hand.** Right hand if the ball's y-coordinate is on the
   right side of the body, left otherwise.
4. **Cartesian approach.** Five Cartesian waypoints take the end-effector
   from out-to-the-side, in over the ball, then down to contact height.
   Hand orientations are captured quaternions from working test runs;
   intermediate orientations are computed with **quaternion SLERP** so
   the wrist rotation between waypoints is smooth.
5. **Staged finger closure.** Proximal joints flex first, then the thumb
   is positioned, then all fingers pre-curl to cage the ball, then the
   distal joints close fully (thumb slightly faster than the other
   fingers).
6. **Lift.** Final Cartesian move 12 cm straight up at the same
   orientation.

## Files

- `hw2.py` — full grasping pipeline.
- `hw2.yaml` — pyCub configuration for this task.

## Running

Install pyCub per the project docs, then run from the repo root:

```python
python hw2.py
```

By default the script runs nine trials and prints per-trial and
aggregated scores. Set `FAKE_VISION = False` in `__main__` to use live
camera images instead of the saved data.

## Course
Humanoid Robotics, Czech Technical University in Prague,
Electrical Engineering Exchange Program, Spring 2026.
