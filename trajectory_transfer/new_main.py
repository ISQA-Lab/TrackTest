import numpy as np
import matplotlib.pyplot as plt
import pickle
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from my_utils import (load_background_npcs, load_map, load_inserted_npcs,
                      consturct_npc_track_by_mpc_result, load_ego_car, \
                      get_ego_trajectory, check_trajectory_intersect, get_start_end_frame)


def get_map_dict(data_num):
    resolution = 0.1
    file_path = f"{os.path.dirname(os.path.abspath(__file__))}/kitti_map/{data_num:04d}_{resolution}.png"

    map_params = {
        0: {
            "origin": [-80, -10],
            "waypoint": (
                [(-10.2, 5.6), (-15.7, 15.2), (-20.0, 24.2),
                 (-30.5, 29.2), (-40.7, 29.4), (-50.3, 28.8), (-62, 31), (-70, 33)],
                [(-10.2, 5.6), (-11.0, 6.5), (-18, 20), (-19.1, 23.2), (-18, 30.5), (-15.5, 33)]
            ),
            "default_speed_level": 0
        },
        1: {
            "origin": [-80, 0],
        },
        2: {
            "origin": [-10, -80],
            "waypoint": (
                [(10.5, -11.5), (20.7, -22.2), (30.7, -32.6), (40.8, -42.3), (50.2, -50.5), (60.0, -58.1),
                 (70.9, -66.1), (80.2, -72.5)],  
            ),
            "oppo_lane_wp": (
                [(65, -75), (40, -55), (0, -15)],
                [(43, -57), (32, -50), (11.7, -31.3), (0, -15)],
                [(54, -67), (40, -55), (32, -50), (11.7, -31.3), (0, -15)],
                [(40, -55), (20, -40), (32, -50), (11.7, -31.3), (0, -15)],
            ),
            "default_speed_level": 2
        },
        3:{
            "origin": [10, -60]
        },
        4:{
            "origin": [1,-4]
        },
        5: {
            "origin": [0, 0],
            "waypoint": (
                [(50, 48), (100, 92), (145, 129), (199, 168), (260, 210)],
            ),
            "oppo_lane_wp": (
                [(265, 212), (229, 184), (176, 146), (100, 87), (47, 39)],
                [(241, 193), (229, 184), (176, 146), (100, 87), (47, 39)],
                [(176, 146), (100, 87), (47, 39)],
                [(148, 123), (100, 87), (47, 39)],
                [(100, 87), (47, 39)],
                [(64, 52), (47, 39)],
                [(53, 44), (47, 39)],
            ),
            "default_speed_level": 2
        },
        8:{
            "origin":[-555, -170]
        },
        13:{
            "origin": [-20, -200]
        },
        7: {
            "origin": [0, -185],
            "waypoint": (
                [(13, -23), (13, -50), (14.5, -75), (14.5, -125), (14, -150),
                 (15, -169), (17.8, -175), (25, -180),
                 (64, -176), (64, -178), (74, -161),
                 (74, -120), (74, -100), (74, -75), (73, -50), (72.5, -25), (71.7, -6.4),
                 (73, 6), (81, 8.7), (96, 12), (113, 16), (126, 20), (135, 16)],
            ),
            "default_speed_level": 1,
            "segment": [
                (40, 230),
                (230, 360),
                (360, 604),
                
            ]
        },
        10: {
            "origin": (-385, 0),
            "waypoint": (
                [(-57.9, 25.4), (-106.5, 50.6), (-154.0, 75.6), (-199.5, 100.6), (-246.3, 125.3), (-305.0, 150.4)],
            ),
            "default_speed_level": 1,
            "oppo_lane_wp": (
                [(-304, 156), (-246, 131), (-199, 107), (-155, 82), (-100, 55), (-51, 31), (-29, 23)],
                [(-246, 131), (-199, 107), (-155, 82), (-100, 55), (-51, 31), (-29, 23)],
                [(-199, 107), (-155, 82), (-100, 55), (-51, 31), (-29, 23)],
                [(-155, 82), (-100, 55), (-51, 31), (-29, 23)],
                [(-130, 72), (-51, 31), (-29, 23)],
                [(-120, 65), (-51, 31), (-29, 23)],
                [(-100, 55), (-51, 31), (-29, 23)],
                [(-75, 42), (-51, 31), (-29, 23)]
            ),
        },
        11: {
            "origin": (-100, 0),
            "waypoint": (
                [(-13.5, 25.2), (-26.5, 50.2), (-39.3, 75.7), (-52.1, 100.7), (-64.7, 125.1), (-77.3, 150.3),
                 (-89.3, 175.5)],
            ),
            "default_speed_level": 2,
            "oppo_lane_wp": (
                [(-75.9, 164.4), (-51.1, 118.2), (-23.2, 62.9), (-5.8, 25.9), (-1.5, 19.1)],
                [(-60.1, 133.5), (-51.1, 118.2), (-23.2, 62.9), (-5.8, 25.9), (-1.5, 19.1)],
                [(-51.1, 118.2), (-23.2, 62.9), (-5.8, 25.9), (-1.5, 19.1)],
                [(-33.7, 81.8), (-23.2, 62.9), (-5.8, 25.9), (-1.5, 19.1)],
                [(-16.3, 48.6), (-5.8, 25.9), (-1.5, 19.1)],
            ),
        },
        14: {  
            "origin": (-40, 0),
            "waypoint": [
                [(0.4, 6.0), (0.3, 10.3), (-2.8, 15.0), (-5.8, 16.6), (-10.5, 17.5), (-15.4, 17.9), (-20.1, 18.1),
                 (-25.1, 18.3), (-30.6, 18.5), (-35.5, 18.7), (-38.7, 18.7)],
            ],
            "default_speed_level": 2,
            "oppo_lane_wp": (
                [(-35.5, 21), (-30.6, 20.8), (-25.1, 20.6), (-20.1, 20.4), (-15.4, 20.2), (-10.5, 20.2), (-5.8, 20.2),
                 (0, 20.2), (1.5, 20.2)],
                [(-15.4, 20.2), (-10.5, 20.2), (-5.8, 20.2), (0, 20.2), (1.5, 20.2)],
                
                [(-5.8, 20.2), (0, 20.2), (1.5, 20.2)],
                [(-30.6, 20.8), (-25.1, 20.6), (-20.1, 20.4), (-15.4, 20.2), (-10.5, 19.8), (-5.8, 18.7),
                 (-2.8, 17), (1.5, 10), (2, 8.5)],
                [(-20.1, 20.4), (-10.5, 19.8), (-5.8, 18.7), (-2.8, 17), (1.5, 10), (2, 8.5)],
                
                
            ),
            "segment": [(0, 105)]  
        },
        18: {
            "origin": (0, -5),
            "waypoint": (
                [(36.5, 25.7), (69.2, 50.9), (97.7, 75.2), (98.6, 76.0), (124.9, 100.3), (125.4, 100.7), (150.2, 125.3),
                 (150.5, 125.7), (150.8, 126.0), (173.6, 150.5), (174.1, 151.0)],

            ),
            "default_speed_level": 2,
            "oppo_lane_wp": (
                [(182.3, 150.4), (173.1, 139.8), (158.4, 125.6), (139.1, 107.2), (111.5, 80.5), (84.9, 56.2),
                 (66.9, 42.4), (49.5, 29.1), (29.3, 14.0), (15.0, 3.8)],
                [(158.4, 125.6), (139.1, 107.2), (111.5, 80.5), (84.9, 56.2), (66.9, 42.4), (49.5, 29.1),
                 (29.3, 14.0), (15.0, 3.8)],
                [(139.1, 107.2), (111.5, 80.5), (84.9, 56.5), (66.9, 42.4), (49.5, 29.1), (29.3, 14.0),
                 (15.0, 3.8)],
                [(111.5, 80.5), (84.9, 56.5), (66.9, 42.4), (49.5, 29.1), (29.3, 14.0), (15.0, 3.8)],
                [(84.9, 56.5), (66.9, 42.4), (49.5, 29.1), (29.3, 14.0), (15.0, 3.8)],
                [(66.9, 42.4), (49.5, 29.1), (29.3, 14.0), (15.0, 3.8)],
                [(56.8, 34.1), (49.5, 29.1), (29.3, 14.0), (15.0, 3.8)],
                [(49.5, 29.1), (29.3, 14.0), (15.0, 3.8)],
                [(37.5, 19.9), (29.3, 14.0), (15.0, 3.8)],
                [(29.3, 14.0), (15.0, 3.8)],
            ),
        },
    }

    map_dict = map_params[data_num]
    map_dict["file_path"] = file_path
    map_dict["resolution"] = resolution

    return map_dict


def init_data_path():
	  
    
    ego_path = "./ego_trajectory.npy"
    npc_path = "" 
    return ego_path, npc_path
    
def get_car_dict(use_safe=False):
    safety_width_margin = 0.4
    safety_length_margin = 1
    if not use_safe:
        safety_width_margin = 0
        safety_length_margin = 0
    car_dict = {
        "length": 4 + safety_length_margin,  
        "width": 1.3 + safety_width_margin,  
        "ts": 0.1,
        
    }
    return car_dict


def transfer(id,npy,name,start_frame,savedir,car_id=1):
    map_dict = get_map_dict(id)
    
    my_map = load_map(map_dict)
    ego_path, npc_path = init_data_path() 
    
    car_dict = get_car_dict()                  
    
    
    
    
    
    
    
    
    
    
    
    
    
    pos = np.load(npy)
    
    pos_global = my_map.m2w_strict( pos[0],  pos[1])
    
    
    
    center_arr = list(zip(pos_global[1],pos_global[0]))
    center_arr = np.array(center_arr)
    
    
    
    
    
    yaw_arr = []
    for i in range(len(center_arr) - 1):
        dif_ahead = center_arr[i + 1] - center_arr[i]
        
        dist_ahead = np.linalg.norm(dif_ahead, 2)
        
        psi = np.arctan2(dif_ahead[1], dif_ahead[0])  
        yaw_arr.append(psi)
    yaw_arr.append(yaw_arr[-1])
    yaw_arr = np.array(yaw_arr).reshape(-1, 1)
    res_arr = np.concatenate([center_arr, yaw_arr], axis=1)
    
    
    np.save(f"{savedir}/{car_id}_{name}_2_{start_frame}_{start_frame+len(res_arr)}_-1_1.npy", res_arr)
    print(f"save to 1_{name}_2_{start_frame}_{start_frame+len(res_arr)}_-1_1.npy")
    return os.path.join(savedir,f"{car_id}_{name}_2_{start_frame}_{start_frame+len(res_arr)}_-1_1.npy")
    
    
    
    
    
    
    
    
    
    

    
    
    
    
    
    
    
    
    
    
    
    
    

if __name__ == '__main__':
    ego_path="./5_ego_trajectory.npy"
    npc_path = "./background_npc_trajectory.npy"
    map_dict = get_map_dict(5)
    print(map_dict)
    my_map = load_map(map_dict)
    ego_trajectory = get_ego_trajectory(ego_path)  
    e1 = ego_trajectory[:, 1]
    e2 = ego_trajectory[:, 0]
    e = list(zip(*my_map.w2m_strict(e1, e2)))
    pickle.dump(e, open("5_ego_trajectory.pkl", "wb"))
    
    

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
        
        