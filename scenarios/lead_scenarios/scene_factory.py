import os
import pickle


def _slice_tracks(ego_path, others, frame_start, frame_end):
    frame_len = frame_end - frame_start + 1
    ego = ego_path[frame_start:frame_start + frame_len]
    sliced_others = [item[:frame_len] for item in others]
    return ego, sliced_others


def _scene_1(script_dir):
    from z3 import And
    from curves.curve_1 import roadline_left, roadline_middle, roadline_right, getderivative, getvalue

    seq = 1
    frame_start, frame_end = 73, 100
    frame_len = frame_end - frame_start + 1

    ego = pickle.load(open(os.path.join(script_dir, "data/egos__0001_ego_trajectory.pkl"), "rb"))[:frame_len]
    back0 = pickle.load(open(os.path.join(script_dir, "data/npc_1_linux__0__generates__npc_0.pkl"), "rb"))[:frame_len]
    back1 = pickle.load(open(os.path.join(script_dir, "data/npc_1_linux__0__generates__npc_1.pkl"), "rb"))[:frame_len]

    on_left = lambda x, y: And(x > getvalue(y, roadline_left) + 5, x < getvalue(y, roadline_middle) - 5)
    on_right = lambda x, y: And(x > getvalue(y, roadline_middle) + 5, x < getvalue(y, roadline_right) - 5)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [on_left, on_right],
        "roadinfo": [[ego, back0], [back1]],
        "ego_idx": 0,
        "npcs": [
            "data/npc_1_linux__0__final__1_npc0_2_73_98_-1_1.npy",
            "data/npc_1_linux__0__final__1_npc1_2_73_98_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "x_of_y",
            "forward_axis": "y",
            "forward_sign": 1,
            "init_road_idx": 0,
            "init_point": 1,
            "speed_limit": 25,
            "seconds_range": (4, 5),
            "save_distance_range": (70, 100),
            "init_ref_line": roadline_middle,
            "keep_ref_line": roadline_middle,
            "path_name_template": "{seq}_lead_{uid}",
        },
    }


def _scene_2(script_dir):
    from z3 import And
    from curves.curve_2 import roadline1, roadline2, roadline3, getderivative, getvalue

    seq = 2
    frame_start = 0
    ego = pickle.load(open(os.path.join(script_dir, "data/egos__2_ego_trajectory.pkl"), "rb"))[0:41]
    back0 = pickle.load(open("data/with_npc__15__generates__npc_0.pkl", "rb"))[0:41]
    back1 = pickle.load(open("data/with_npc__15__generates__npc_1.pkl", "rb"))[0:41]

    ontop = lambda x, y: And(y > getvalue(x, roadline2) + 15, y <= getvalue(x, roadline1) - 10)
    onsecond = lambda x, y: And(y >= getvalue(x, roadline3), y < getvalue(x, roadline2) - 20)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [ontop, onsecond],
        "roadinfo": [[back0], [ego, back1]],
        "ego_idx": 1,
        "npcs": [
            "data/with_npc__15__final__1_npc0_2_0_41_-1_1.npy",
            "data/with_npc__15__final__1_npc1_2_0_41_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "y_of_x",
            "forward_axis": "x",
            "forward_sign": 1,
            "init_road_idx": 1,
            "init_point": 1,
            "speed_limit": 25,
            "seconds_range": (4, 8),
            "save_distance_range": (25, 30),
            "keep_ref_line": roadline2,
            "extra_axis_constraints": [("y", -1)],
            "path_name_template": "{seq}_lead_{uid}",
        },
    }


def _scene_3(script_dir):
    from curves.curve_3 import roadline3, getderivative, on_upper_lane_relaxed, on_lower_lane_relaxed

    seq = 3
    frame_start, frame_end = 110, 143
    ego_path = pickle.load(open(os.path.join(script_dir, "data/egos__0003_ego_trajectory.pkl"), "rb"))
    bg_path = pickle.load(open(os.path.join(script_dir, "data/egos__0003_background_npc_trajectory.pkl"), "rb"))
    back0 = pickle.load(open(os.path.join(script_dir, "data/npc_3_linux__0__generates__npc_0.pkl"), "rb"))
    back1 = pickle.load(open(os.path.join(script_dir, "data/npc_3_linux__0__generates__npc_1.pkl"), "rb"))
    ego, [bg_path, back0, back1] = _slice_tracks(ego_path, [bg_path, back0, back1], frame_start, frame_end)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [on_lower_lane_relaxed, on_upper_lane_relaxed],
        "roadinfo": [[ego, back0, bg_path], [back1]],
        "ego_idx": 0,
        "npcs": [
            "data/npc_3_linux__0__final__1_npc0_2_110_141_-1_1.npy",
            "data/npc_3_linux__0__final__1_npc1_2_110_141_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "y_of_x",
            "forward_axis": "x",
            "forward_sign": 1,
            "init_road_idx": 0,
            "init_point": 1,
            "speed_limit": 25,
            "seconds_range": (4, 5),
            "save_distance_range": (30, 50),
            "init_ref_line": roadline3,
            "keep_ref_line": roadline3,
            "path_name_template": "{seq}_lead_{uid}",
        },
    }


def _scene_4(script_dir):
    from curves.curve_4 import roadline3, getderivative, on_upper_lane_relaxed, on_lower_lane_relaxed

    seq = 4
    frame_start, frame_end = 75, 105
    ego_path = pickle.load(open(os.path.join(script_dir, "data/egos__0004_ego_trajectory.pkl"), "rb"))
    bg_path = pickle.load(open(os.path.join(script_dir, "data/egos__0004_background_npc_trajectory.pkl"), "rb"))
    back0 = pickle.load(open(os.path.join(script_dir, "data/npc_4_linux__0__generates__npc_0.pkl"), "rb"))
    back1 = pickle.load(open(os.path.join(script_dir, "data/npc_4_linux__0__generates__npc_1.pkl"), "rb"))
    ego, [bg_path, back0, back1] = _slice_tracks(ego_path, [bg_path, back0, back1], frame_start, frame_end)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [on_lower_lane_relaxed, on_upper_lane_relaxed],
        "roadinfo": [[ego, back0, bg_path], [back1]],
        "ego_idx": 0,
        "npcs": [
            "data/npc_4_linux__0__final__1_npc0_2_75_106_-1_1.npy",
            "data/npc_4_linux__0__final__1_npc1_2_75_106_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "y_of_x",
            "forward_axis": "x",
            "forward_sign": 1,
            "init_road_idx": 0,
            "init_point": 1,
            "speed_limit": 25,
            "seconds_range": (4, 5),
            "save_distance_range": (30, 50),
            "init_ref_line": roadline3,
            "keep_ref_line": roadline3,
            "path_name_template": "{seq}_lead_{uid}",
        },
    }


def _scene_5(script_dir):
    from z3 import And
    from curves.curve_5 import roadline1, roadline2, roadline4, getderivative, getvalue

    seq = 5
    frame_start = 0
    ego = pickle.load(open(os.path.join(script_dir, "data/egos__5_ego_trajectory.pkl"), "rb"))[0:50]
    bg = pickle.load(open(os.path.join(script_dir, "data/egos__5_background_npc_trajectory.pkl"), "rb"))[0:50]
    back0 = pickle.load(open("data/npc_5_linux__0__generates__npc_0.pkl", "rb"))[0:50]
    back1 = pickle.load(open("data/npc_5_linux__0__generates__npc_1.pkl", "rb"))[0:50]

    ontop = lambda x, y: And(y > getvalue(x, roadline2) + 15, y <= getvalue(x, roadline1) - 10)
    onsecond = lambda x, y: And(y >= getvalue(x, roadline4), y < getvalue(x, roadline2) - 20)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [ontop, onsecond],
        "roadinfo": [[ego, back0, bg], [back1]],
        "ego_idx": 0,
        "npcs": [
            "data/npc_5_linux__0__final__1_npc0_2_0_50_-1_1.npy",
            "data/npc_5_linux__0__final__1_npc1_2_0_50_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "y_of_x",
            "forward_axis": "x",
            "forward_sign": 1,
            "init_road_idx": 0,
            "init_point": 1,
            "speed_limit": 35,
            "seconds_range": (4, 8),
            "save_distance_range": (50, 65),
            "keep_ref_line": roadline2,
            "path_name_template": "{seq}_lead_{uid}",
            "enforce_forward_position": False,
        },
    }


def _scene_8(script_dir):
    from curves.curve_8 import roadline2, getderivative, on_upper_lane_relaxed, on_middle_lane_relaxed, on_lower_lane_relaxed

    seq = 8
    frame_start, frame_end = 50, 80
    ego_path = pickle.load(open(os.path.join(script_dir, "data/egos__0008_ego_trajectory.pkl"), "rb"))
    back0 = pickle.load(open(os.path.join(script_dir, "data/npc_8_linux__0__generates__npc_0.pkl"), "rb"))
    back1 = pickle.load(open(os.path.join(script_dir, "data/npc_8_linux__0__generates__npc_1.pkl"), "rb"))
    ego, [back0, back1] = _slice_tracks(ego_path, [back0, back1], frame_start, frame_end)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [on_upper_lane_relaxed, on_middle_lane_relaxed, on_lower_lane_relaxed],
        "roadinfo": [[ego, back0], [back1], []],
        "ego_idx": 0,
        "npcs": [
            "data/npc_8_linux__0__final__1_npc0_2_50_79_-1_1.npy",
            "data/npc_8_linux__0__final__1_npc1_2_50_79_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "y_of_x",
            "forward_axis": "x",
            "forward_sign": -1,
            "init_road_idx": 0,
            "init_point": 1,
            "speed_limit": 80,
            "seconds_range": (4, 5),
            "save_distance_range": (100, 120),
            "init_ref_line": roadline2,
            "keep_ref_line": roadline2,
            "path_name_template": "{seq}_lead_{uid}",
        },
    }


def _scene_10(script_dir):
    from z3 import And
    from curves.curve_10 import roadline1, roadline2, roadline4, getderivative, getvalue, lane_margin_relaxed

    seq = 10
    frame_start, frame_end = 90, 125
    ego_path = pickle.load(open(os.path.join(script_dir, "data/egos__0010_ego_trajectory.pkl"), "rb"))
    back0 = pickle.load(open(os.path.join(script_dir, "data/npc_10_linux__0__generates__npc_0.pkl"), "rb"))
    back1 = pickle.load(open(os.path.join(script_dir, "data/npc_10_linux__0__generates__npc_1.pkl"), "rb"))
    ego, [back0, back1] = _slice_tracks(ego_path, [back0, back1], frame_start, frame_end)

    on_upper = lambda x, y: And(y > getvalue(x, roadline2) + lane_margin_relaxed, y < getvalue(x, roadline1) - lane_margin_relaxed)
    on_lower = lambda x, y: And(y > getvalue(x, roadline4) + lane_margin_relaxed, y < getvalue(x, roadline2) - lane_margin_relaxed)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [on_lower, on_upper],
        "roadinfo": [[ego, back1], [back0]],
        "ego_idx": 0,
        "npcs": [
            "data/npc_10_linux__0__final__1_npc0_2_90_121_-1_1.npy",
            "data/npc_10_linux__0__final__1_npc1_2_90_121_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "y_of_x",
            "forward_axis": "x",
            "forward_sign": -1,
            "init_road_idx": 0,
            "init_point": 1,
            "speed_limit": 25,
            "seconds_range": (4, 5),
            "save_distance_range": (30, 50),
            "init_ref_line": roadline2,
            "keep_ref_line": roadline2,
            "path_name_template": "{seq}_lead_{uid}",
        },
    }


def _scene_13(script_dir):
    from curves.curve_13 import roadline2, getderivative, on_left_lane_relaxed, on_right_lane_relaxed

    seq = 13
    frame_start, frame_end = 15, 45
    ego_path = pickle.load(open(os.path.join(script_dir, "data/egos__0013_ego_trajectory.pkl"), "rb"))
    back0 = pickle.load(open(os.path.join(script_dir, "data/npc_13_linux__0__generates__npc_0.pkl"), "rb"))
    back1 = pickle.load(open(os.path.join(script_dir, "data/npc_13_linux__0__generates__npc_1.pkl"), "rb"))
    ego, [back0, back1] = _slice_tracks(ego_path, [back0, back1], frame_start, frame_end)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [on_left_lane_relaxed, on_right_lane_relaxed],
        "roadinfo": [[ego, back0], [back1]],
        "ego_idx": 0,
        "npcs": [
            "data/npc_13_linux__0__final__1_npc0_2_15_46_-1_1.npy",
            "data/npc_13_linux__0__final__1_npc1_2_15_46_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "x_of_y",
            "forward_axis": "y",
            "forward_sign": -1,
            "init_road_idx": 0,
            "init_point": 1,
            "speed_limit": 25,
            "seconds_range": (4, 5),
            "save_distance_range": (50, 80),
            "init_ref_line": roadline2,
            "keep_ref_line": roadline2,
            "path_name_template": "{seq}_lead_{uid}",
        },
    }


def _scene_14(script_dir):
    from z3 import And
    from curves.curve_14 import roadline1, roadline2, roadline4, getderivative, getvalue

    seq = 14
    frame_start, frame_end = 80, 105
    ego_path = pickle.load(open(os.path.join(script_dir, "data/egos__14_ego_trajectory.pkl"), "rb"))
    back0 = pickle.load(open(os.path.join(script_dir, "data/npc_14_linux__0__generates__npc_0.pkl"), "rb"))
    back1 = pickle.load(open(os.path.join(script_dir, "data/npc_14_linux__0__generates__npc_1.pkl"), "rb"))
    ego, [back0, back1] = _slice_tracks(ego_path, [back0, back1], frame_start, frame_end)

    on_upper = lambda x, y: And(y > getvalue(x, roadline1) + 10, y < getvalue(x, roadline2) - 10)
    on_lower = lambda x, y: And(y > getvalue(x, roadline2) + 10, y < getvalue(x, roadline4) - 10)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [on_upper, on_lower],
        "roadinfo": [[ego, back0], [back1]],
        "ego_idx": 0,
        "npcs": [
            "data/npc_14_linux__0__final__1_npc0_2_80_106_-1_1.npy",
            "data/npc_14_linux__0__final__1_npc1_2_80_106_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "y_of_x",
            "forward_axis": "x",
            "forward_sign": -1,
            "init_road_idx": 0,
            "init_point": 1,
            "speed_limit": 25,
            "seconds_range": (4, 5),
            "save_distance_range": (70, 100),
            "init_ref_line": roadline2,
            "keep_ref_line": roadline2,
            "path_name_template": "{seq}_lead_{uid}",
        },
    }


def _scene_18(script_dir):
    from z3 import And
    from curves.curve_18 import roadline1, roadline2, roadline3, getderivative, getvalue

    seq = 18
    frame_start, frame_end = 0, 48
    ego_path = pickle.load(open(os.path.join(script_dir, "data/egos__18_ego_trajectory.pkl"), "rb"))
    back0 = pickle.load(open(os.path.join(script_dir, "data/npc_18_linux__0__generates__npc_0.pkl"), "rb"))
    back1 = pickle.load(open(os.path.join(script_dir, "data/npc_18_linux__0__generates__npc_1.pkl"), "rb"))
    ego, [back0, back1] = _slice_tracks(ego_path, [back0, back1], frame_start, frame_end)

    margin = 5
    on_left = lambda x, y: And(y > getvalue(x, roadline2) + margin, y < getvalue(x, roadline1) - margin)
    on_middle = lambda x, y: And(y > getvalue(x, roadline3) + margin, y < getvalue(x, roadline2) - margin)

    return {
        "seq": seq,
        "frame_start": frame_start,
        "roads": [on_left, on_middle],
        "roadinfo": [[ego, back0], [back1]],
        "ego_idx": 0,
        "npcs": [
            "data/npc_18_linux__0__final__1_npc0_2_0_49_-1_1.npy",
            "data/npc_18_linux__0__final__1_npc1_2_0_49_-1_1.npy",
        ],
        "getderivative": getderivative,
        "config": {
            "curve_mode": "y_of_x",
            "forward_axis": "x",
            "forward_sign": 1,
            "init_road_idx": 0,
            "init_point": 1,
            "speed_limit": 25,
            "seconds_range": (4, 5),
            "save_distance_range": (25, 45),
            "init_ref_line": roadline1,
            "keep_ref_line": roadline1,
            "path_name_template": "{seq}_lead_{uid}",
        },
    }


_SCENE_BUILDERS = {
    1: _scene_1,
    2: _scene_2,
    3: _scene_3,
    4: _scene_4,
    5: _scene_5,
    8: _scene_8,
    10: _scene_10,
    13: _scene_13,
    14: _scene_14,
    18: _scene_18,
}


def build_lead_scene(seq, script_dir):
    if seq not in _SCENE_BUILDERS:
        raise ValueError(f"Unsupported lead scene: {seq}")
    return _SCENE_BUILDERS[seq](script_dir)

