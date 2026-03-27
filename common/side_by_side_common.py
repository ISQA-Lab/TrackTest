import math
import os
import random
import time
import pickle
import shutil
from z3 import Solver, Real, If, sat, set_option

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


def _get_derivative(getderivative, axis_value, ref_line):
    try:
        return getderivative(axis_value, ref_line)
    except TypeError:
        return getderivative(axis_value)


def _add_heading_constraint(solver, vx, vy, derivative, curve_mode):
    if curve_mode == "y_of_x":
        solver.add(vx != 0)
        solver.add(vy / vx == derivative[1] / derivative[0])
    else:
        solver.add(vy != 0)
        solver.add(vx / vy == derivative[1] / derivative[0])


def _add_order_constraint(solver, position, lane_cars, insert_point, step_idx, distance, forward_sign):
    if len(lane_cars) == 0:
        return

    if insert_point == 0:
        ref = lane_cars[0][step_idx]
        bound = ref + distance if forward_sign > 0 else ref - distance
        solver.add(position < bound if forward_sign > 0 else position > bound)
        return

    if insert_point >= len(lane_cars):
        ref = lane_cars[-1][step_idx]
        bound = ref + distance if forward_sign > 0 else ref - distance
        solver.add(position > bound if forward_sign > 0 else position < bound)
        return

    prev_ref = lane_cars[insert_point - 1][step_idx]
    next_ref = lane_cars[insert_point][step_idx]

    prev_bound = prev_ref + distance if forward_sign > 0 else prev_ref - distance
    next_bound = next_ref + distance if forward_sign > 0 else next_ref - distance

    if forward_sign > 0:
        solver.add(position > prev_bound)
        solver.add(position < next_bound)
    else:
        solver.add(position < prev_bound)
        solver.add(position > next_bound)


def _resolve_npc_path(script_dir, npc):
    candidates = [npc]
    if script_dir:
        candidates.append(os.path.join(script_dir, npc))
        candidates.append(os.path.join(script_dir, "..", npc))
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return None


def _relation_band_constraints(solver, value, ego_value, rel_sign, center_offset, tolerance):
    if rel_sign > 0:
        solver.add(value > ego_value + center_offset - tolerance)
        solver.add(value < ego_value + center_offset + tolerance)
    else:
        solver.add(value < ego_value - center_offset + tolerance)
        solver.add(value > ego_value - center_offset - tolerance)


def generate_side_by_side(
    uid,
    save_dir,
    seq,
    frame_start,
    roadinfo,
    roads,
    ego_idx,
    script_dir,
    getderivative,
    npcs,
    curve_mode="y_of_x",
    forward_axis="x",
    forward_sign=1,
    relation_axis="x",
    relation_sign=None,
    init_road_idx=None,
    init_road_mode="adjacent_random",
    init_point=0,
    speed_limit=50,
    seconds_range=(3, 5),
    save_distance_range=(30, 50),
    timeout_ms=1000,
    path_name_template="{seq}_side_by_side_{uid}",
    ref_line=None,
    extra_axis_constraints=None,
    offset_min=30,
    offset_max=50,
    oscillation_amplitude=15,
    oscillation_frequency=1.5,
    post_sin_axis=None,
    post_sin_amplitude=0.0,
    post_sin_frequency=1.0,
    post_sin_random_phase=False,
):
    seconds_between_points = random.randint(seconds_range[0], seconds_range[1])
    print("seconds_between_points:", seconds_between_points)

    sampled_roadinfo, point_count = _sample_roadinfo(roadinfo, seconds_between_points)
    if point_count <= 2:
        print("point_count too small")
        return -1
    sampled_roadinfo = _trim_to_point_count(sampled_roadinfo, point_count)
    print("point_count:", point_count)

    if init_road_mode == "adjacent_random":
        choices = []
        if ego_idx - 1 >= 0:
            choices.append(ego_idx - 1)
        if ego_idx + 1 < len(roads):
            choices.append(ego_idx + 1)
        if not choices:
            print("no adjacent lane available")
            return -1
        init_road_idx = random.choice(choices)
    elif init_road_idx is None:
        print("init_road_idx is required when init_road_mode is fixed")
        return -1

    if relation_sign is None:
        relation_sign = forward_sign

    init_road = roads[init_road_idx]
    init_road_info = sampled_roadinfo[init_road_idx]
    ego_path = sampled_roadinfo[ego_idx][0]

    axis_idx = 0 if forward_axis == "x" else 1
    relation_axis_idx = 0 if relation_axis == "x" else 1

    save_distance = random.randint(save_distance_range[0], save_distance_range[1])
    base_offset = (offset_min + offset_max) / 2.0
    tolerance = offset_max - offset_min

    s = Solver()
    s.reset()
    waypoints = [
        (
            Real(f"x{i}"),
            Real(f"y{i}"),
            Real(f"x_speed{i}"),
            Real(f"y_speed{i}"),
            Real(f"x_acc{i}"),
            Real(f"y_acc{i}"),
        )
        for i in range(point_count)
    ]

    set_option(rational_to_decimal=True)
    set_option(precision=10)

    x0, y0, vx0, vy0, _, _ = waypoints[0]
    s.add(init_road(x0, y0))

    axis0 = x0 if curve_mode == "y_of_x" else y0
    der0 = _get_derivative(getderivative, axis0, ref_line)
    _add_heading_constraint(s, vx0, vy0, der0, curve_mode)

    init_pos = x0 if axis_idx == 0 else y0
    _add_order_constraint(s, init_pos, init_road_info, init_point, 0, save_distance, forward_sign)

    ego_rel0 = ego_path[0][relation_axis_idx]
    rel_value0 = x0 if relation_axis_idx == 0 else y0
    _relation_band_constraints(s, rel_value0, ego_rel0, relation_sign, base_offset, tolerance)

    start = time.time()

    for i in range(1, point_count):
        x1, y1, vx1, vy1, ax1, ay1 = waypoints[i - 1]
        x2, y2, vx2, vy2, ax2, ay2 = waypoints[i]

        s.add(x2 == x1 + vx1 * seconds_between_points + ax1 / 2 * seconds_between_points ** 2)
        s.add(y2 == y1 + vy1 * seconds_between_points + ay1 / 2 * seconds_between_points ** 2)
        s.add(vx2 == vx1 + ax1 * seconds_between_points)
        s.add(vy2 == vy1 + ay1 * seconds_between_points)

        if forward_axis == "x":
            s.add(x2 > x1 if forward_sign > 0 else x2 < x1)
            s.add(vx2 > 0 if forward_sign > 0 else vx2 < 0)
        else:
            s.add(y2 > y1 if forward_sign > 0 else y2 < y1)
            s.add(vy2 > 0 if forward_sign > 0 else vy2 < 0)

        if extra_axis_constraints:
            for axis_name, axis_sign in extra_axis_constraints:
                if axis_name == "x":
                    s.add(x2 > x1 if axis_sign > 0 else x2 < x1)
                else:
                    s.add(y2 > y1 if axis_sign > 0 else y2 < y1)

        s.add(If(vx2 > 0, vx2, -vx2) + If(vy2 > 0, vy2, -vy2) < speed_limit)
        s.add(init_road(x2, y2))

        axis_val = x2 if curve_mode == "y_of_x" else y2
        der = _get_derivative(getderivative, axis_val, ref_line)
        _add_heading_constraint(s, vx2, vy2, der, curve_mode)

        cur_pos = x2 if axis_idx == 0 else y2
        _add_order_constraint(s, cur_pos, init_road_info, init_point, i, save_distance, forward_sign)

        ego_rel_i = ego_path[i][relation_axis_idx]
        rel_value_i = x2 if relation_axis_idx == 0 else y2
        oscillation_offset = oscillation_amplitude * math.sin(i * oscillation_frequency)
        effective_offset = base_offset + oscillation_offset
        _relation_band_constraints(s, rel_value_i, ego_rel_i, relation_sign, effective_offset, tolerance)

    set_option("timeout", timeout_ms)

    path = []
    if s.check() != sat:
        end = time.time()
        print("Time:", end - start)
        print("no solution")
        return -1

    phase = random.uniform(0, 2 * math.pi) if post_sin_random_phase else 0.0

    m = s.model()
    for idx, waypoint in enumerate(waypoints[:-1]):
        x, y, vx, vy, ax, ay = map(lambda var: get_value_of_z3(m, var), waypoint)

        if post_sin_axis is not None and post_sin_amplitude != 0:
            sin_offset = post_sin_amplitude * math.sin(post_sin_frequency * idx + phase)
            if post_sin_axis == "x":
                x = x + sin_offset
            else:
                y = y + sin_offset

        path.append((x, y, vx, vy))
        for j in range(1, seconds_between_points):
            small_x = x + vx * j + 0.5 * ax * j ** 2
            small_y = y + vy * j + 0.5 * ay * j ** 2
            small_vx = vx + ax * j
            small_vy = vy + ay * j

            if post_sin_axis is not None and post_sin_amplitude != 0:
                sub_idx = idx * seconds_between_points + j
                sin_offset_sub = post_sin_amplitude * math.sin(
                    post_sin_frequency * sub_idx / seconds_between_points + phase
                )
                if post_sin_axis == "x":
                    small_x = small_x + sin_offset_sub
                else:
                    small_y = small_y + sin_offset_sub

            path.append((small_x, small_y, small_vx, small_vy))

    last_vals = list(map(lambda var: get_value_of_z3(m, var), waypoints[-1][:-2]))
    if post_sin_axis is not None and post_sin_amplitude != 0:
        last_idx = len(waypoints) - 1
        sin_offset_last = post_sin_amplitude * math.sin(post_sin_frequency * last_idx + phase)
        if post_sin_axis == "x":
            last_vals[0] += sin_offset_last
        else:
            last_vals[1] += sin_offset_last

    path.append(last_vals)

    end = time.time()
    print("Time:", end - start)

    name = path_name_template.format(seq=seq, uid=uid)
    pkl_path = os.path.join(save_dir, "generates", f"{name}.pkl")
    npy_path = os.path.join(save_dir, "npy", f"{name}.npy")
    os.makedirs(os.path.dirname(pkl_path), exist_ok=True)
    os.makedirs(os.path.dirname(npy_path), exist_ok=True)
    pickle.dump(path, open(pkl_path, "wb"))
    pkl2npy(pkl_path, npy_path)

    from trajectory_transfer.new_main import transfer

    trajectory_dir = os.path.join(save_dir, "final")
    os.makedirs(trajectory_dir, exist_ok=True)
    transfer(seq, npy_path, "".join(name.split("_")), frame_start, trajectory_dir, 1)

    for npc in npcs:
        npc_full_path = _resolve_npc_path(script_dir, npc)
        if npc_full_path and os.path.exists(npc_full_path):
            shutil.copy(npc_full_path, trajectory_dir)

    return 0
