import os
import json


def calculate_distance(p1,p2):
    sum = 0
    t = 2
    for x1,x2 in zip(p1,p2):
        sum += (x1-x2)**2
    return sum**(1/t)


def distance_score(directory, start_frame,end_frame,dis1,dis2):
    total_num = 0
    num_gt_dis1 = 0
    num_gt_dis2 = 0
    num_lt_dis1 = 0
    num_between_dis1_dis2 = 0

    for i in range(start_frame,end_frame+1):
        frame = json.load(open(os.path.join(directory, f"scene_{i}.json")))
        vehicles = frame["vehicles"]
        for v in vehicles:
            total_num += 1
            dis = calculate_distance(v["location"],[0,0,0])

            if dis > dis1:
                num_gt_dis1 += 1
            if dis > dis2:
                num_gt_dis2 += 1

            if dis < dis1:
                num_lt_dis1 += 1
            elif dis < dis2:
                num_between_dis1_dis2 += 1

    if total_num == 0:
        return {
            "score": 0.0,
            "total_num": 0,
            "num_gt_dis1": 0,
            "num_gt_dis2": 0,
            "num_lt_dis1": 0,
            "num_between_dis1_dis2": 0,
        }

    score = (num_lt_dis1 + num_gt_dis2) / total_num
    return {
        "score": score,
        "total_num": total_num,
        "num_gt_dis1": num_gt_dis1,
        "num_gt_dis2": num_gt_dis2,
        "num_lt_dis1": num_lt_dis1,
        "num_between_dis1_dis2": num_between_dis1_dis2,
    }
