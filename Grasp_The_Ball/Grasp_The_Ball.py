from __future__ import annotations
import os
import sys
import math
from typing import Optional
import cv2
import numpy as np
 
from icub_pybullet.pycub import EndEffector, pyCub
from icub_pybullet.utils import Pose
 
import open3d.visualization.rendering as rendering
import open3d as o3d
 
 
class Grasper:
    def __init__(self, client: pyCub, fake_vision: bool = False, idx: int = 0,
                 parent_path=os.path.realpath(os.path.dirname(__file__))):
        self.client = client
        self.fake_vision = fake_vision
        if self.fake_vision:
            self.rgb = cv2.imread(os.path.join(parent_path, "data", f"rgb_{idx}.png"))
            self.td_points = np.load(os.path.join(parent_path, "data", f"td_points_{idx}.npy"), allow_pickle=False)
        self.eye = "l_eye"
        for link in self.client.links:
            if link.name == f"{self.eye}_pupil":
                self.eye_link_id = link.robot_link_id
        self.mat = rendering.MaterialRecord()
        self.mat.shader = "defaultLit"
 
    # ---- provided helpers (unchanged from template) ----
 
    def get_rgb(self) -> np.ndarray:
        if self.fake_vision:
            return self.rgb
        return self.client.get_camera_images(self.eye)[self.eye]
 
    def get_depth(self) -> np.ndarray:
        if not self.fake_vision:
            return self.client.get_camera_depth_images(self.eye)[self.eye]
        raise NotImplementedError("Depth image is not available in fake vision mode")
 
    def get_3d_point(self, u: int, v: int, d: float = -1) -> np.ndarray:
        if not self.fake_vision:
            ew = self.client.visualizer.eye_windows[self.eye]
            return ew.unproject(v, u, d)
        return self.td_points[u, v]
 
    def move_fingers(self, closure: Optional[float] = 1.0, hand: Optional[str] = "right",
                     timeout: Optional[float] = 10):
        hand = "r" if hand == "right" else "l"
        joints = [
            f"{hand}_hand_thumb_2_joint", f"{hand}_hand_thumb_3_joint",
            f"{hand}_hand_index_2_joint", f"{hand}_hand_index_3_joint",
            f"{hand}_hand_middle_2_joint", f"{hand}_hand_middle_3_joint",
            f"{hand}_hand_ring_2_joint", f"{hand}_hand_ring_3_joint",
            f"{hand}_hand_little_2_joint", f"{hand}_hand_little_3_joint",
        ]
        for joint in joints:
            jh = self.get_joint_handle(joint)
            if jh is None:
                continue
            self.client.move_position(
                joint,
                jh.lower_limit + closure * (jh.upper_limit - jh.lower_limit),
                wait=False, check_collision=False, timeout=timeout, velocity=5,
            )
        while not self.client.motion_done(check_collision=False):
            self.client.update_simulation()
 
    def get_pupil_vectors(self, point: np.ndarray) -> tuple:
        head_state = self.client.getLinkState(
            self.client.robot, self.eye_link_id,
            computeLinkVelocity=0, computeForwardKinematics=0)
        head_pos, head_ori = head_state[0], head_state[1]
        pupil_ball_direction = np.array(point) - head_pos
        pupil_ball_direction /= np.linalg.norm(pupil_ball_direction)
        R_l = np.eye(4)
        R_l[:3, :3] = np.reshape(self.client.getMatrixFromQuaternion(head_ori), (3, 3))
        pupil_direction = np.matmul(R_l, [0, 0, 1, 1])[:3]
        pupil_direction /= np.linalg.norm(pupil_direction)
        return pupil_ball_direction, pupil_direction
 
    def get_joint_handle(self, joint_name: str):
        for joint in self.client.joints:
            if joint.name == joint_name:
                return joint
        return None
 
    @staticmethod
    def quaternion_swap(q, to: Optional[str] = "wxyz") -> list:
        if to == "wxyz":
            return [q[3], q[0], q[1], q[2]]
        elif to == "xyzw":
            return [q[1], q[2], q[3], q[0]]
 
    def set_ee(self, link_name: str):
        self.client.end_effector = EndEffector(link_name, self.client)
 
    # ---- small utility ----
 
    def safe_move_position(self, joint_name, target, velocity=2, timeout=5):
        jh = self.get_joint_handle(joint_name)
        if jh is None:
            return
        lo, hi = sorted([jh.lower_limit, jh.upper_limit])
        clamped = max(lo + 0.001, min(hi - 0.001, target))
        try:
            self.client.move_position(joint_name, clamped, wait=True,
                                      check_collision=False, timeout=timeout, velocity=velocity)
        except Exception:
            pass
 
    # ---- TASK 1: find the ball ----
 
    def find_the_ball(self) -> tuple:
        img = self.get_rgb()
        lower = np.array([0, 150, 0], dtype=np.uint8)
        upper = np.array([120, 255, 120], dtype=np.uint8)
        mask = cv2.inRange(img, lower, upper)
        ys, xs = np.where(mask > 0)
        u = int(np.mean(xs))
        v = int(np.mean(ys))
        return (u, v)
 
    # ---- TASK 2: grasp ----
 
    def grasp(self, center: tuple | list | np.ndarray) -> int:
        u, v = center
 
        # 1) Get 3D ball position (surface point + radius along viewing ray)
        surface_point = np.array(self.get_3d_point(v, u))
        ray, _ = self.get_pupil_vectors(surface_point)
        ball = surface_point + 0.025 * ray
        bx, by, bz = float(ball[0]), float(ball[1]), float(ball[2])
 
        # 2) Look at the ball (eyes only, once is enough per spec)
        pball_dir, _ = self.get_pupil_vectors(ball)
        tilt = float(np.arctan2(pball_dir[2], np.sqrt(pball_dir[0]**2 + pball_dir[1]**2)))
        pan = float(np.arctan2(pball_dir[1], -pball_dir[0]))
        self.client.move_position("eyes_tilt", np.clip(tilt, -0.52, 0.52),
                                  wait=False, check_collision=False, timeout=5, velocity=2)
        self.client.move_position("l_eye_pan_joint", np.clip(pan, -0.52, 0.52),
                                  wait=False, check_collision=False, timeout=5, velocity=2)
        self.client.move_position("r_eye_pan_joint", np.clip(pan, -0.52, 0.52),
                                  wait=False, check_collision=False, timeout=5, velocity=2)
        try:
            while not self.client.motion_done(check_collision=False):
                self.client.update_simulation()
        except Exception:
            for _ in range(50):
                self.client.update_simulation()
 
        # 3) Select hand based on ball position
        if by >= -0.05:
            self.set_ee("r_hand")
            hand_str, h = "right", "r"
        else:
            self.set_ee("l_hand")
            hand_str, h = "left", "l"
 
        # 4) Three-phase approach: out to side → lower forearm → sweep over ball
        if h == "r":
            # Quaternions captured from working right-side run (xyzw format)
            q1_r = [-0.004, 0.153, -0.487, 0.860]
            q2_r = [0.029, 0.141, -0.235, 0.961]
            pd_r = [0.492, -0.305, -0.041, 0.815]
            # SLERP step 3 = halfway between step 2 and palm-down
            q2_wxyz = self.quaternion_swap(q2_r, "wxyz")
            pd_wxyz = self.quaternion_swap(pd_r, "wxyz")
            dot = sum(a * b for a, b in zip(q2_wxyz, pd_wxyz))
            if dot < 0:
                pd_wxyz = [-x for x in pd_wxyz]
                dot = -dot
            t = 0.5
            s1 = math.sin((1 - t) * math.acos(min(dot, 1.0))) / math.sin(math.acos(min(dot, 1.0)))
            s2 = math.sin(t * math.acos(min(dot, 1.0))) / math.sin(math.acos(min(dot, 1.0)))
            q3_wxyz = [s1 * a + s2 * b for a, b in zip(q2_wxyz, pd_wxyz)]
            q3_r = self.quaternion_swap(q3_wxyz, "xyzw")
            step_quats = [q1_r, q2_r, q3_r, pd_r, pd_r]
        else:
            # Quaternions captured from working left-side run (xyzw format)
            q_l2 = [-0.012009, 0.045669, 0.998296, 0.034271]
            q_l5 = [-0.322629, 0.471894, 0.819217, -0.045922]
            # SLERP between step 2 and step 4 for a smooth step 3
            q2w = self.quaternion_swap(q_l2, "wxyz")
            q5w = self.quaternion_swap(q_l5, "wxyz")
            dot = sum(a * b for a, b in zip(q2w, q5w))
            if dot < 0:
                q5w = [-x for x in q5w]
                dot = -dot
            theta = math.acos(min(dot, 1.0))
            s1 = math.sin(0.5 * theta) / math.sin(theta)
            s2 = math.sin(0.5 * theta) / math.sin(theta)
            q3w = [s1 * a + s2 * b for a, b in zip(q2w, q5w)]
            q_l3 = self.quaternion_swap(q3w, "xyzw")
            step_quats = [
                [0.040008, -0.076927, 0.866608, -0.491399],
                [0.013403, -0.018798, 0.966150, -0.256947],
                q_l3,
                [-0.311571, 0.489286, 0.810662, -0.079679],
                q_l5,
            ]
 
        # Phase 1: Sweep from side to over ball, staying high
        if h == "r":
            waypoints = [
                (0.20, 0.15, 0.15),        # out to side, high
                (0.18, 0.075, 0.15),       # coming in, high
                (0.15, 0, 0.15),           # directly over ball, still high
            ]
            waypoints += [
                (0.10, 0.002, 0.095),      # lower
                (0.08, 0.002, 0.055),      # contact height
            ]
        else:
            waypoints = [
                (0.20, -0.15, 0.15),       # out to side, high
                (0.18, -0.075, 0.15),      # coming in, high
                (0.15, 0, 0.15),           # directly over ball, still high
            ]
            waypoints += [
                (0.10, -0.002, 0.095),     # lower
                (0.08, -0.002, 0.055),     # contact height
            ]
 
        for i, (wx, wy, wz) in enumerate(waypoints):
            self.client.move_cartesian(
                Pose([bx + wx, by + wy, bz + wz], step_quats[i]),
                wait=True, check_collision=False, timeout=8)
 
 
 
        # 7) Point proximal joints down, then curl and close
        self.move_fingers(0.0, hand=hand_str)
 
        # Flex proximal (_1) joints downward
        # Curl thumb joints slightly before positioning
        for tj in [1, 2, 3]:
            tjname = f"{h}_hand_thumb_{tj}_joint"
            tjh = self.get_joint_handle(tjname)
            if tjh is not None:
                target = tjh.lower_limit + 0.2 * (tjh.upper_limit - tjh.lower_limit)
                self.client.move_position(tjname, target,
                                          wait=False, check_collision=False, timeout=5, velocity=3)
        while not self.client.motion_done(check_collision=False):
            self.client.update_simulation()
 
        # Position thumb first, wait for it to finish
        thumb_0 = f"{h}_hand_thumb_0_joint"
        jh0 = self.get_joint_handle(thumb_0)
        if jh0 is not None:
            target = jh0.lower_limit + 0.90 * (jh0.upper_limit - jh0.lower_limit)
            self.safe_move_position(thumb_0, target, velocity=3)
 
        # Pre-curl all fingers to cage the ball: _1=60%, _2=50%, _3=40%
        finger_names = ["thumb", "index", "middle", "ring", "little"]
        curl_levels = {1: 0.35, 2: 0.3, 3: 0.25}
        for fname in finger_names:
            for joint_idx, closure in curl_levels.items():
                jname = f"{h}_hand_{fname}_{joint_idx}_joint"
                jh = self.get_joint_handle(jname)
                if jh is not None:
                    target = jh.lower_limit + closure * (jh.upper_limit - jh.lower_limit)
                    self.client.move_position(jname, target,
                                              wait=False, check_collision=False, timeout=5, velocity=5)
 
        while not self.client.motion_done(check_collision=False):
            self.client.update_simulation()
 
        # Full close — thumb faster than other fingers
        close_joints = [
            f"{h}_hand_thumb_2_joint", f"{h}_hand_thumb_3_joint",
            f"{h}_hand_index_2_joint", f"{h}_hand_index_3_joint",
            f"{h}_hand_middle_2_joint", f"{h}_hand_middle_3_joint",
            f"{h}_hand_ring_2_joint", f"{h}_hand_ring_3_joint",
            f"{h}_hand_little_2_joint", f"{h}_hand_little_3_joint",
        ]
        for jname in close_joints:
            jh = self.get_joint_handle(jname)
            if jh is None:
                continue
            vel = 6 if "thumb" in jname else 5
            self.client.move_position(jname, jh.upper_limit,
                                      wait=False, check_collision=False, timeout=10, velocity=vel)
        while not self.client.motion_done(check_collision=False):
            self.client.update_simulation()
 
        for _ in range(30):
            self.client.update_simulation()
 
        # Re-fire for tighter grip
        self.move_fingers(1.0, hand=hand_str)
 
        # 10) Lift with same orientation
        self.client.move_cartesian(
            Pose([bx, by, bz + 0.12], None),
            wait=True, check_collision=False, timeout=8)
 
        return 0
 
 
def main(pos, idx, fake_vision=True) -> tuple[int, float, float]:
    client = pyCub(config="Grasp_The_Ball.yaml")
    client.resetBasePositionAndOrientation(client.other_objects[1][0], pos, [0, 0, 0, 1])
    grasper = Grasper(client, fake_vision, int(idx.split("_")[-2]))
 
    while client.steps_done < 10:
        client.update_simulation()
 
    try:
        center = grasper.find_the_ball()
        grasper.grasp(center)
    except Exception as e:
        print(e)
        return 0, 0, 0
 
    c = client.getClosestPoints(client.other_objects[1][0], client.other_objects[2][0], np.Inf)
    min_dist = np.Inf
    for _ in c:
        d = _[client.contactPoints["DISTANCE"]]
        if d < min_dist:
            min_dist = d
 
    c = client.getClosestPoints(
        client.other_objects[1][0], client.robot, np.Inf, -1, client.end_effector.link_id)
    min_dist_to_hand = np.Inf
    for _ in c:
        d = _[client.contactPoints["DISTANCE"]]
        if d < min_dist_to_hand:
            min_dist_to_hand = d
 
    min_dist = np.abs(np.round(min_dist, 3))
    min_dist_to_hand = np.abs(np.round(min_dist_to_hand, 3))
 
    score = 5 if min_dist > 0.05 > min_dist_to_hand else 0
    return score, min_dist * 100, min_dist_to_hand * 100
 
 
if __name__ == "__main__":
    FAKE_VISION = True
    REPS = 3
    positions = [[-0.35, 0.175, -0.1], [-0.35, 0, -0.1], [-0.35, -0.175, -0.1]]
    score = 0
    total_score = 0
    for position_id in range(len(positions)):
        pose_score = 0
        for rep in range(REPS):
            idx = f"{position_id}_{rep}"
            position = positions[position_id]
            score_temp, min_dist, min_dist_to_hand = main(position, idx, FAKE_VISION)
            pose_score += score_temp
            print(f"Test {idx} score {score_temp} dist_table {min_dist:.3f}cm dist_hand {min_dist_to_hand:.3f}cm")
        score += min(pose_score, 5)
        total_score += pose_score
        print(f"Pose {position_id} score {min(pose_score, 5)}")
    if total_score == REPS * 3 * 5:
        score = 17
        print(f"All 9 passed! Score: {score}")
    else:
        print(f"Total score: {score}")