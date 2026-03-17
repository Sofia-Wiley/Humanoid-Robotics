import os
import sys
try:
    from icub_pybullet.pycub import pyCub
except:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
    from icub_pybullet.pycub import pyCub

import time


def get_poses(client):
    # get ball position and orientation
    ball_pos, ball_ori = client.getBasePositionAndOrientation(client.free_objects[0])

    # Get head link position and orientation
    head_state = client.getLinkState(client.robot, 96, computeLinkVelocity=0, computeForwardKinematics=0)
    head_pos, head_ori = head_state[0], head_state[1]

    return ball_pos, ball_ori, head_pos, head_ori


def gaze(client):
    # TODO
    pass

if __name__ == "__main__":
    client = pyCub(config="hw2.yaml")
    # look down and move arms from the view
    client.move_position("neck_pitch", -0.54, wait=False)
    client.move_position(["l_shoulder_pitch", "l_shoulder_roll", "r_shoulder_pitch", "r_shoulder_roll"],
                         [-1.5, 1.5, -1.5, 1.5], wait=False)
    while not client.motion_done():
        client.update_simulation(None)

    # apply force to the ball so it moves
    K = -5
    client.applyExternalForce(client.free_objects[0], -1, [K, 0, 0], [0, 0, 0], client.WORLD_FRAME)

    time_step = client.config.simulation_step
    t = time.time()
    last_steps = None
    while client.is_alive() and time.time()-t < 10:
        if last_steps is None or last_steps != client.steps_done:
            gaze(client)
            last_steps = client.steps_done

        client.update_simulation(time_step)


