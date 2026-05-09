import os
import sys
import pybullet as p
import numpy as np
from math import sqrt, atan2, degrees, acos
  
try:
    from icub_pybullet.pycub import pyCub
except:
 
    sys.path.insert(0, os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
    from icub_pybullet.pycub import pyCub
  
import open3d as o3d
import open3d.visualization.rendering as rendering
import random
import time
  
  
# clips val between lo and hi, used everywhere for joint limits
def clamp(value, lo, hi):
    return max(lo, min(hi, value))
  
  
# link 96 = head
HEAD_LINK = 96
  
  
def get_poses(client):
    # get ball head pos/ori each step
    ball_pos, _ = client.getBasePositionAndOrientation(
        client.free_objects[0]
    )
 
    head_state = client.getLinkState(
        client.robot, HEAD_LINK,
        computeLinkVelocity=0,
        computeForwardKinematics=1
    )
    return ball_pos, head_state[0], head_state[1]
  
  
def get_head_view_direction(head_ori):
    # quaternion convert to rotation matrix, then grab look direction
    # dot product w/ ball dir gives 1.0 when tracking is correct
    R = np.array(p.getMatrixFromQuaternion(head_ori)).reshape(3, 3)
    view_dir = R @ np.array([0.0, 0.0, 1.0])
    return view_dir / np.linalg.norm(view_dir)
  
  
def compute_gaze_error_degrees(view_dir, head_pos, ball_pos):
    # angle between where head is looking vs where ball is
    to_ball = np.array(ball_pos) - np.array(head_pos)
    dist = np.linalg.norm(to_ball)
    if dist < 1e-6:
        return 0.0
    to_ball_unit = to_ball / dist
    dot = clamp(float(np.dot(view_dir, to_ball_unit)), -1.0, 1.0)
    return degrees(acos(dot))
  
  
def draw_debug_lines(client, mat_line, head_pos, ball_pos, view_dir):
    # green = gaze ray (where head is looking)
    # red   = line to ball (where it should be looking)
    try:
        vis  = client.visualizer.vis
        head = np.array(head_pos)
        ball = np.array(ball_pos)
        gaze_end = head + view_dir * 1.5
  
        gaze_line = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector([head.tolist(), gaze_end.tolist()]),
            lines=o3d.utility.Vector2iVector([[0, 1]])
        )
        gaze_line.colors = o3d.utility.Vector3dVector([[0, 1, 0]])
        try:
            vis.remove_geometry("gaze_ray")
        except Exception:
            pass
        vis.add_geometry("gaze_ray", geometry=gaze_line, material=mat_line)
  
        ball_line = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector([head.tolist(), ball.tolist()]),
            lines=o3d.utility.Vector2iVector([[0, 1]])
        )
        ball_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]])
        try:
            vis.remove_geometry("ball_line")
        except Exception:
            pass
        vis.add_geometry("ball_line", geometry=ball_line, material=mat_line)
    except Exception:
        pass
  
  
def gaze(client):
    # get current positions
    ball_pos, head_pos, _ = get_poses(client)
  
    # one-time init:chest frame so we don't recompute every step
    if not hasattr(client, "_gaze_init"):
        chest_id = None
        for link in client.links:
            if link.name == "chest":
                chest_id = link.robot_link_id
                break
        chest_state = client.getLinkState(
            client.robot, chest_id,
            computeForwardKinematics=1
        )
        R_chest = np.array(
            p.getMatrixFromQuaternion(chest_state[1])
        ).reshape(3, 3)
        client._R_chest_T  = R_chest.T   # transpose = inverse for rotation matrix
        client._sim_step   = client.config.simulation_step
        client._gaze_init  = True
  
    # lead the ball a bit based on how fast its moving
    # capped at 8 steps for no overshoot
    ball_vel, _ = client.getBaseVelocity(client.free_objects[0])
    ball_speed   = np.linalg.norm(ball_vel)
    lookahead_steps = clamp(2.0 + ball_speed * 3.0, 2.0, 8.0)
    lookahead       = client._sim_step * lookahead_steps
    ball_predict    = np.array(ball_pos) + np.array(ball_vel) * lookahead
  
    # direction from head to predicted ball
    d_world = ball_predict - np.array(head_pos)
    norm = np.linalg.norm(d_world)
    if norm < 1e-6:
        return  # ball is at head, skip
    d_world /= norm
  
    # transform into chest frame for IK angles
    d = client._R_chest_T @ d_world
  
    # 2-DOF IK: atan2 handles quadrants, clamp keeps joints in range
    target_pitch = clamp(
        atan2(d[1], d[2]),
        -0.698131700798, 0.383972435439
    )
    target_yaw = clamp(
        atan2(d[0], sqrt(d[1]**2 + d[2]**2)),
        -0.872664625997, 0.872664625997
    )
  
    # velocity=1000 is basically instant, joints snap to target each step
    client.move_position(
        ["neck_yaw", "neck_pitch"],
        [target_yaw, target_pitch],
        wait=False,
        velocity=1000.0,
    )
  
  
if __name__ == "__main__":
    client = pyCub(config="Gaze.yaml")
  
    # line rendering setup for debug viz
    mat_line            = rendering.MaterialRecord()
    mat_line.shader     = "unlitLine"
    mat_line.line_width = 10
  
    # head down a bit + arms up so they're out of the way
    client.move_position("neck_pitch", -0.54, wait=False)
    client.move_position(
        ["l_shoulder_pitch", "l_shoulder_roll",
         "r_shoulder_pitch", "r_shoulder_roll"],
        [-1.5, 1.5, -1.5, 1.5],
        wait=False
    )
    while not client.motion_done():
        client.update_simulation(None)
  
    error_history  = []
    last_kick_time = time.time()
    kick_interval  = random.uniform(0.8, 1.5)
  
    t          = time.time()
    last_steps = None
  
    # runs for 20 sec
    while client.is_alive() and time.time() - t < 20:
        now = time.time()
  
        # kick ball at random intervals w/ random direction + 3 strength tiers
        if now - last_kick_time >= kick_interval:
            fx     = random.uniform(-1, 1)
            fy     = random.uniform(-1, 1)
            norm_f = sqrt(fx*fx + fy*fy) or 1.0
            K = random.choice([
                random.uniform(2, 5),    # soft
                random.uniform(6, 10),   # medium
                random.uniform(10, 15),  # hard
            ])
            client.applyExternalForce(
                client.free_objects[0], -1,
                [K * fx / norm_f, K * fy / norm_f, 0],
                [0, 0, 0],
                client.WORLD_FRAME
            )
            last_kick_time = now
            kick_interval  = random.uniform(0.5, 2.0)
  
        # only update gaze once per sim step, not every wall clock tick
        if last_steps is None or last_steps != client.steps_done:
            gaze(client)
  
            ball_pos, head_pos, head_ori = get_poses(client)
            view_dir  = get_head_view_direction(head_ori)
            error_deg = compute_gaze_error_degrees(view_dir, head_pos, ball_pos)
            error_history.append(error_deg)
  
            draw_debug_lines(client, mat_line, head_pos, ball_pos, view_dir)
  
            last_steps = client.steps_done
  
        client.update_simulation(None)
  
    # print results using the same thresholds as the rubric
    if error_history:
        mean_e = float(np.mean(error_history))
        max_e  = float(np.max(error_history))
  
        if mean_e < 0.55:
            mean_pct = 0
        elif mean_e < 1.0:
            mean_pct = 50
        elif mean_e < 5.0:
            mean_pct = 75
        else:
            mean_pct = 100
  
        if max_e < 1.25:
            max_pct = 0
        elif max_e < 5.0:
            max_pct = 25
        elif max_e < 10.0:
            max_pct = 50
        else:
            max_pct = 100
  
        print("\n=== Gaze tracking results ===")
        print(f"  Mean error : {mean_e:.3f} deg  -> {mean_pct}% of points lost")
        print(f"  Max error  : {max_e:.3f} deg  -> {max_pct}% of points lost")
        print(f"  Total lost : {mean_pct + max_pct}% (lower is better)")