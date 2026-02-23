"""
Template for HRO HW1
"""
import os
import sys
try:
    from icub_pybullet.pycub import pyCub
except:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
    from icub_pybullet.pycub import pyCub

import time
import numpy as np

def push_the_ball(client):
    # WRITE YOUR CODE HERE
    return 0


def evaluate(client):
    c = client.getClosestPoints(client.other_objects[1][0], client.other_objects[2][0], np.Inf)
    min_dist = np.Inf
    for _ in c:
        d = _[client.contactPoints["DISTANCE"]]
        if d < min_dist:
            min_dist = d
    min_dist = np.round(min_dist, 3)
    score = np.round(np.min([min_dist*2, 5]), 2)
    client.logger.info(f"You moved the ball {min_dist}m away from the table. Your score is {score}.")


if __name__ == "__main__":
    # load the robot with correct world/config
    client = pyCub(config="hw1.yaml")

    push_the_ball(client)

    start_step = client.steps_done
    while client.is_alive():
        client.update_simulation()
        if int((client.steps_done - start_step) / (1/client.config.simulation_step)) >= 1:
            break

    evaluate(client)