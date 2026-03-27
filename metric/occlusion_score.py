from pathlib import Path

def _collect_txt_files(root_dir: Path):
    if not root_dir.exists():
        return []
    return sorted(root_dir.rglob("*.txt"))


def _read_valid_ids_from_label(label_file: Path) -> set:
    valid_ids = set()
    if not label_file.exists():
        return valid_ids

    with open(label_file, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 3:
                continue
            cls_type = parts[2]
            if cls_type.lower() == "dontcare":
                continue
            try:
                track_id = int(parts[1])
            except ValueError:
                continue
            valid_ids.add(track_id)
    return valid_ids


def occulsion_score(occlusion_folder, label_folder, thresh=0.5):
    object_hits = []

    occlusion_root = Path(occlusion_folder)
    label_root = Path(label_folder)

    for occ_file in _collect_txt_files(occlusion_root):
        rel_path = occ_file.relative_to(occlusion_root)
        label_file = label_root / rel_path

        if not label_file.exists():
            label_file = label_root / occ_file.name

        valid_ids = _read_valid_ids_from_label(label_file)
        if not valid_ids:
            continue

        with open(occ_file, "r") as f:
            for line in f:
                line = line.strip()
                if not line or ":" not in line:
                    continue
                parts = line.split(":", 1)
                try:
                    obj_id = int(parts[0].strip())
                    ratio = float(parts[1].strip())
                except ValueError:
                    continue
                if obj_id not in valid_ids:
                    continue
                object_hits.append(1 if ratio >= thresh else 0)

    if not object_hits:
        return 0.0
    return sum(object_hits) / len(object_hits)
