import argparse
import os

from common.cut_out_common import generate_cut_out
from scenarios.cut_out_scenarios import build_cut_out_scene


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
    scene = build_cut_out_scene(seq, SCRIPT_DIR)

    config = dict(scene["config"])
    if start_road_idx is not None:
        config["init_road_idx"] = start_road_idx
        if config.get("target_road_mode") == "adjacent_random":
            config["target_road_mode"] = "fixed"
            config["target_road_idx"] = max(0, min(len(scene["roads"]) - 1, start_road_idx + 1))

    return generate_cut_out(
        uid=uid,
        save_dir=save_dir,
        seq=seq,
        frame_start=scene["frame_start"],
        roadinfo=roadinfo if roadinfo is not None else scene["roadinfo"],
        roads=roads if roads is not None else scene["roads"],
        ego_idx=ego_idx if ego_idx is not None else scene["ego_idx"],
        script_dir=SCRIPT_DIR,
        getderivative=scene["getderivative"],
        npcs=npcs if npcs is not None else scene["npcs"],
        **config,
    )


def main():
    parser = argparse.ArgumentParser(description="Unified cut_out entry")
    parser.add_argument("--seq", type=int, required=True, choices=[1, 2, 3, 4, 5, 8, 10, 13, 14, 18])
    parser.add_argument("--uid", type=int, default=0)
    parser.add_argument("--save_dir", type=str, required=True)
    parser.add_argument("--retry", action="store_true", help="Retry until a satisfiable solution is found")
    parser.add_argument("--start_road_idx", type=int, default=None, help="Override start lane index")
    args = parser.parse_args()

    if args.retry:
        while func(
            uid=args.uid,
            save_dir=args.save_dir,
            seq=args.seq,
            start_road_idx=args.start_road_idx,
        ) == -1:
            print("retrying")
    else:
        ret = func(
            uid=args.uid,
            save_dir=args.save_dir,
            seq=args.seq,
            start_road_idx=args.start_road_idx,
        )
        if ret == -1:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
