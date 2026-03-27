import os
import random
import time
import pickle
import shutil
from z3 import Solver, Real, If, Or, sat, set_option

from utils.pathgenerate import pkl2npy
from utils.util import get_value_of_z3


def _sample_roadinfo(roadinfo, seconds_between_points):
    sampled = []
    lengths = []
    for road in roadinfo:
        sampled_road = []
        for car in road:
            sampled_car = car[::seconds_between_points]
            sampled_road.append(sampled_car)
            lengths.append(len(sampled_car))
        sampled.append(sampled_road)
    point_count = min(lengths) if lengths else 0
    return sampled, point_count


def _trim_to_point_count(roadinfo, point_count):
    trimmed = []
    for road in roadinfo:
        trimmed_road = []
        for car in road:
            trimmed_road.append(car[:point_count])
        trimmed.append(trimmed_road)
    return trimmed


def _forward_bounds(ego_value, sign, min_offset, max_offset):
    if sign > 0:
        return ego_value + min_offset, ego_value + max_offset
    return ego_value - max_offset, ego_value - min_offset


def generate_followline(
    uid,
    save_dir,
    seq,
    frame_start,
    roadinfo,
    roads,
    road_ref_lines,
    ego_idx,
    npcs,
    script_dir,
    getderivative,
    curve_mode="y_of_x",
    forward_axis="x",
    forward_sign=1,
    speed_limit=25,
    seconds_range=(4, 5),
    save_distance_range=(35, 60),
    follow_offset_range=(45, 90),
):
    seconds_between_points = random.randint(seconds_range[0], seconds_range[1])
    print("seconds_between_points:", seconds_between_points)

    sampled_roadinfo, point_count = _sample_roadinfo(roadinfo, seconds_between_points)
    if point_count <= 2:
        print("point_count too small")
        return -1
    sampled_roadinfo = _trim_to_point_count(sampled_roadinfo, point_count)
    print("point_count:", point_count)

    candidate_lanes = [ego_idx]
    if ego_idx - 1 >= 0:
        candidate_lanes.append(ego_idx - 1)
    if ego_idx + 1 < len(roads):
        candidate_lanes.append(ego_idx + 1)

    lane_count = random.randint(1, min(2, len(candidate_lanes)))
    target_lanes = sorted(random.sample(candidate_lanes, lane_count))
    print("target_lanes:", target_lanes)

    axis_idx = 0 if forward_axis == "x" else 1
    ego_key_points = sampled_roadinfo[ego_idx][0]

    generated = []
    trajectory_dir = os.path.join(save_dir, "final")
    os.makedirs(trajectory_dir, exist_ok=True)

    for lane_idx in target_lanes:
        s = Solver()
        s.reset()

        waypoints = [
            (
                Real(f"x_{lane_idx}_{i}"),
                Real(f"y_{lane_idx}_{i}"),
                Real(f"x_speed_{lane_idx}_{i}"),
                Real(f"y_speed_{lane_idx}_{i}"),
                Real(f"x_acc_{lane_idx}_{i}"),
                Real(f"y_acc_{lane_idx}_{i}"),
            )
            for i in range(point_count)
        ]

        set_option(rational_to_decimal=True)
        set_option(precision=10)

        init_road = roads[lane_idx]
        ref_line = road_ref_lines[lane_idx]
        lane_cars = sampled_roadinfo[lane_idx]
        save_distance = random.randint(save_distance_range[0], save_distance_range[1])
        min_offset, max_offset = follow_offset_range

        x0, y0, vx0, vy0, ax0, ay0 = waypoints[0]
        s.add(init_road(x0, y0))

        if curve_mode == "y_of_x":
            der0 = getderivative(x0, ref_line)
            s.add(vx0 != 0)
            s.add(vy0 / vx0 == der0[1] / der0[0])
        else:
            der0 = getderivative(y0, ref_line)
            s.add(vy0 != 0)
            s.add(vx0 / vy0 == der0[1] / der0[0])

        ego_pos_0 = ego_key_points[0][axis_idx]
        lower_bound, upper_bound = _forward_bounds(ego_pos_0, forward_sign, min_offset, max_offset)
        if axis_idx == 0:
            s.add(x0 >= lower_bound, x0 <= upper_bound)
        else:
            s.add(y0 >= lower_bound, y0 <= upper_bound)

        start = time.time()

        for i in range(1, point_count):
            x1, y1, vx1, vy1, ax1, ay1 = waypoints[i - 1]
            x2, y2, vx2, vy2, ax2, ay2 = waypoints[i]

            s.add(x2 == x1 + vx1 * seconds_between_points + ax1 / 2 * seconds_between_points ** 2)
            s.add(y2 == y1 + vy1 * seconds_between_points + ay1 / 2 * seconds_between_points ** 2)
            s.add(vx2 == vx1 + ax1 * seconds_between_points)
            s.add(vy2 == vy1 + ay1 * seconds_between_points)

            if forward_axis == "x":
                if forward_sign > 0:
                    s.add(x2 > x1, vx2 > 0)
                else:
                    s.add(x2 < x1, vx2 < 0)
            else:
                if forward_sign > 0:
                    s.add(y2 > y1, vy2 > 0)
                else:
                    s.add(y2 < y1, vy2 < 0)

            s.add(If(vx2 > 0, vx2, -vx2) + If(vy2 > 0, vy2, -vy2) < speed_limit)
            s.add(init_road(x2, y2))

            if curve_mode == "y_of_x":
                der = getderivative(x2, ref_line)
                s.add(vx2 != 0)
                s.add(vy2 / vx2 == der[1] / der[0])
            else:
                der = getderivative(y2, ref_line)
                s.add(vy2 != 0)
                s.add(vx2 / vy2 == der[1] / der[0])

            ego_pos_i = ego_key_points[i][axis_idx]
            lb, ub = _forward_bounds(ego_pos_i, forward_sign, min_offset, max_offset)
            cur_pos = x2 if axis_idx == 0 else y2
            s.add(cur_pos >= lb, cur_pos <= ub)

            for car in lane_cars:
                car_pos = car[i][axis_idx]
                s.add(Or(cur_pos > car_pos + save_distance, cur_pos < car_pos - save_distance))

        set_option("timeout", 2000)

        path = []
        if s.check() != sat:
            end = time.time()
            print("Time:", end - start)
            print(f"lane {lane_idx} no solution")
            return -1

        m = s.model()
        for waypoint in waypoints[:-1]:
            x, y, vx, vy, ax, ay = map(lambda var: get_value_of_z3(m, var), waypoint)
            path.append((x, y, vx, vy))
            for j in range(1, seconds_between_points):
                small_x = x + vx * j + 0.5 * ax * j ** 2
                small_y = y + vy * j + 0.5 * ay * j ** 2
                small_vx = vx + ax * j
                small_vy = vy + ay * j
                path.append((small_x, small_y, small_vx, small_vy))
        path.append(list(map(lambda var: get_value_of_z3(m, var), waypoints[-1][:-2])))

        end = time.time()
        print(f"lane {lane_idx} solved, time:", end - start)

        name = f"{seq}_followline_{uid}_{lane_idx}"
        pkl_path = os.path.join(save_dir, "generates", f"{name}.pkl")
        npy_path = os.path.join(save_dir, "npy", f"{name}.npy")
        os.makedirs(os.path.dirname(pkl_path), exist_ok=True)
        os.makedirs(os.path.dirname(npy_path), exist_ok=True)
        pickle.dump(path, open(pkl_path, "wb"))
        pkl2npy(pkl_path, npy_path)

        from trajectory_transfer.new_main import transfer

        transfer(seq, npy_path, "".join(name.split("_")), frame_start, trajectory_dir, 1)
        generated.append(name)

    for npc in npcs:
        npc_full_path = os.path.join(script_dir, "..", npc)
        if os.path.exists(npc_full_path):
            shutil.copy(npc_full_path, trajectory_dir)

    print("generated followline:", generated)
    return 0