import argparse
import os

from common.followline_common import generate_followline
from scenarios.followline_scenarios import build_followline_scene


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def func(
    uid,
    save_dir,
    seq,
    roadinfo=None,
    ego_idx=None,
    npcs=None,
    roads=None,
    start_road_idx=None,
):
    scene = build_followline_scene(seq, SCRIPT_DIR)

    runtime_ego_idx = ego_idx if ego_idx is not None else scene["ego_idx"]
    if start_road_idx is not None:
        runtime_ego_idx = start_road_idx

    return generate_followline(
        uid=uid,
        save_dir=save_dir,
        seq=seq,
        frame_start=scene["frame_start"],
        roadinfo=roadinfo if roadinfo is not None else scene["roadinfo"],
        roads=roads if roads is not None else scene["roads"],
        road_ref_lines=scene["config"]["road_ref_lines"],
        ego_idx=runtime_ego_idx,
        npcs=npcs if npcs is not None else scene["npcs"],
        script_dir=SCRIPT_DIR,
        getderivative=scene["getderivative"],
        curve_mode=scene["config"]["curve_mode"],
        forward_axis=scene["config"]["forward_axis"],
        forward_sign=scene["config"]["forward_sign"],
        speed_limit=scene["config"]["speed_limit"],
        seconds_range=scene["config"]["seconds_range"],
        save_distance_range=scene["config"]["save_distance_range"],
        follow_offset_range=scene["config"]["follow_offset_range"],
    )


def _default_save_dir(seq):
    return f"with_npc/{seq}_followline"


def main():
    parser = argparse.ArgumentParser(description="Unified followline entry")
    parser.add_argument("--seq", type=int, required=True, choices=[1, 2, 3, 4, 5, 8, 10, 13, 14, 18])
    parser.add_argument("--uid", type=int, default=0)
    parser.add_argument("--save_dir", type=str, default=None, help="Output directory, default is with_npc/{seq}_followline")
    parser.add_argument("--retry", action="store_true", help="Retry until a satisfiable solution is found")
    parser.add_argument("--max_retries", type=int, default=None, help="Maximum retry count when --retry is enabled")
    parser.add_argument("--start_road_idx", type=int, default=None, help="Override start lane index")
    args = parser.parse_args()

    save_dir = args.save_dir if args.save_dir else _default_save_dir(args.seq)

    if args.retry:
        attempt = 0
        while True:
            ret = func(
                uid=args.uid,
                save_dir=save_dir,
                seq=args.seq,
                start_road_idx=args.start_road_idx,
            )
            if ret != -1:
                break
            attempt += 1
            if args.max_retries is not None and attempt >= args.max_retries:
                raise SystemExit(1)
            print("retrying")
    else:
        ret = func(
            uid=args.uid,
            save_dir=save_dir,
            seq=args.seq,
            start_road_idx=args.start_road_idx,
        )
        if ret == -1:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
