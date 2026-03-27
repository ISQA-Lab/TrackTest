import os
import json
from math import pi
import numpy as np

def angle_score(npy,theta_thresh=0.1):
    trajectory_numpy = np.load(npy)
    assert len(trajectory_numpy) > 0
    if len(trajectory_numpy) == 1:
        return (0,0,0)
    pre = trajectory_numpy[0][2]
    cnt = 0
    angle_sum = 0   
    for i in range(1,len(trajectory_numpy)):
        diff = abs(trajectory_numpy[i][2]-pre)
        diff = min(diff,2*pi-diff)
        angle_sum += diff
        if diff >= theta_thresh:
            cnt += 1
        pre = trajectory_numpy[i][2]
    return cnt/(len(trajectory_numpy)-1)
        
        