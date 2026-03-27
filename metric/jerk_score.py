import numpy as np
import sys
import os
def jerk_score(npy,jerk_thread=2):
    trajectory_numpy = np.load(npy)
    if len(trajectory_numpy)==1:
        return 0
    trajectory = Trajectory2D.from_numpy(0.1, trajectory_numpy)
    trajectory.update(velocity=True,acceleration=True,jerk=True,yaw_rate=True)
    jerks = trajectory.jerks
    jerk_score = (abs(jerks) > jerk_thread).sum() / len(jerks)
    return jerk_score
