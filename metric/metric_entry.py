import argparse
import json
from typing import Any, Dict, Optional

from metric.angle_score import angle_score
from metric.distance_score import distance_score
from metric.jerk_score import jerk_score
from metric.occlusion_score import occulsion_score
from metric.truncation_score import truncation_score


def _weighted_mean(values: Dict[str, float], weights: Dict[str, float]) -> float:
    used = {k: v for k, v in values.items() if isinstance(v, (int, float)) and k in weights}
    if not used:
        return 0.0
    total_w = sum(weights[k] for k in used)
    if total_w <= 0:
        return 0.0
    return sum(used[k] * weights[k] for k in used) / total_w


def score_single_trajectory(
    npy_path: str,
    theta_thresh: float = 0.1,
    jerk_thread: float = 2.0,
    distance_directory: Optional[str] = None,
    start_frame: Optional[int] = None,
    end_frame: Optional[int] = None,
    dis1: Optional[float] = None,
    dis2: Optional[float] = None,
    occlusion_folder: Optional[str] = None,
    label_folder: Optional[str] = None,
    occlusion_thresh: float = 0.5,
    truncation_thresh: Optional[float] = None,
    truncation_folder_path: Optional[str] = None,
    truncation_trajectory_path: Optional[str] = None,
    weights: Optional[Dict[str, float]] = None,
) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "angle_score": None,
        "jerk_score": None,
        "distance_score": None,
        "occlusion_score": None,
        "truncation_score": None,
        "distance_detail": None,
    }

    result["angle_score"] = angle_score(npy_path, theta_thresh=theta_thresh)
    result["jerk_score"] = jerk_score(npy_path, jerk_thread=jerk_thread)

    if (
        distance_directory is not None
        and start_frame is not None
        and end_frame is not None
        and dis1 is not None
        and dis2 is not None
    ):
        distance_ret = distance_score(
            distance_directory,
            start_frame,
            end_frame,
            dis1,
            dis2,
        )
        result["distance_detail"] = distance_ret
        result["distance_score"] = distance_ret.get("score", 0.0)

    if occlusion_folder is not None and label_folder is not None:
        result["occlusion_score"] = occulsion_score(
            occlusion_folder,
            label_folder,
            thresh=occlusion_thresh,
        )

    if truncation_thresh is not None and truncation_folder_path is not None:
        result["truncation_score"] = truncation_score(
            trajectory_path=truncation_trajectory_path or npy_path,
            thresh=truncation_thresh,
            folder_path=truncation_folder_path,
        )

    numeric_scores = {
        k: float(v)
        for k, v in result.items()
        if k.endswith("_score") and isinstance(v, (int, float))
    }

    default_weights = {
        "angle_score": 1.0,
        "jerk_score": 1.0,
        "distance_score": 1.0,
        "occlusion_score": 1.0,
        "truncation_score": 1.0,
    }
    merged_weights = dict(default_weights)
    if weights:
        merged_weights.update(weights)

    result["composite_score"] = _weighted_mean(numeric_scores, merged_weights)
    result["used_metrics"] = sorted(numeric_scores.keys())
    return result


__all__ = ["score_single_trajectory"]


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Unified metric entry for single trajectory scoring")
    parser.add_argument("--npy-path", required=True)
    parser.add_argument("--theta-thresh", type=float, default=0.1)
    parser.add_argument("--jerk-thread", type=float, default=2.0)

    parser.add_argument("--distance-directory", default=None)
    parser.add_argument("--start-frame", type=int, default=None)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--dis1", type=float, default=None)
    parser.add_argument("--dis2", type=float, default=None)

    parser.add_argument("--occlusion-folder", default=None)
    parser.add_argument("--label-folder", default=None)
    parser.add_argument("--occlusion-thresh", type=float, default=0.5)

    parser.add_argument("--truncation-thresh", type=float, default=None)
    parser.add_argument("--truncation-folder-path", default=None)
    parser.add_argument("--truncation-trajectory-path", default=None)

    parser.add_argument(
        "--weights",
        default=None,
        help='JSON string, e.g. {"angle_score":1,"jerk_score":1}',
    )
    return parser


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    weights = None
    if args.weights:
        try:
            weights = json.loads(args.weights)
        except json.JSONDecodeError as exc:
            raise ValueError(f"Invalid --weights JSON: {exc}") from exc

    result = score_single_trajectory(
        npy_path=args.npy_path,
        theta_thresh=args.theta_thresh,
        jerk_thread=args.jerk_thread,
        distance_directory=args.distance_directory,
        start_frame=args.start_frame,
        end_frame=args.end_frame,
        dis1=args.dis1,
        dis2=args.dis2,
        occlusion_folder=args.occlusion_folder,
        label_folder=args.label_folder,
        occlusion_thresh=args.occlusion_thresh,
        truncation_thresh=args.truncation_thresh,
        truncation_folder_path=args.truncation_folder_path,
        truncation_trajectory_path=args.truncation_trajectory_path,
        weights=weights,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
