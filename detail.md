# TrackTest Scenario Generation (Anonymized)

This repository contains a trajectory/scenario generation pipeline for multi-vehicle traffic cases.
The document is intentionally written in an anonymized style for meeting/review usage:

- No personal names are included.
- No institution- or company-specific identifiers are introduced.
- Only technical structure, call flow, and usage are documented.

## 1. Project Purpose

The project generates synthetic ego/NPC trajectories for several scenario families:

- `cut_in`
- `cut_out`
- `followline`
- `lead`
- `linechange`

Each family has a dedicated top-level entry script (`with_npc_*.py`) that:

1. Builds a scene definition from `scenarios/*_scenarios/scene_factory.py` by `seq`.
2. Merges runtime overrides (such as lane/start index).
3. Calls the corresponding generator in `common/*_common.py`.
4. Produces generated outputs and returns a status code (`0` on success, `-1` on unsatisfied constraints).

---

## 2. Directory Structure (Detailed)

```text
TrackTest/
├─ with_npc_cut_in.py
├─ with_npc_cut_out.py
├─ with_npc_followline.py
├─ with_npc_lead.py
├─ with_npc_linechange.py
├─ common/
│  ├─ cut_in_common.py
│  ├─ cut_out_common.py
│  ├─ followline_common.py
│  ├─ lead_common.py
│  ├─ linechange_common.py
│  └─ side_by_side_common.py
├─ curves/
│  ├─ curve_1.py
│  ├─ curve_2.py
│  ├─ ...
│  └─ curve_18.py
├─ egos/
├─ scenarios/
│  ├─ cut_in_scenarios/scene_factory.py
│  ├─ cut_out_scenarios/scene_factory.py
│  ├─ followline_scenarios/scene_factory.py
│  ├─ lead_scenarios/scene_factory.py
│  ├─ linechange_scenarios/scene_factory.py
│  └─ side_by_side_scenarios/scene_factory.py
├─ trajectory_transfer/
│  ├─ map.py
│  ├─ my_utils.py
│  └─ new_main.py
└─ utils/
   ├─ pathgenerate.py
   └─ util.py
```

### 2.1 Top-level Entry Scripts

- `with_npc_cut_in.py`: unified runtime entry for cut-in generation.
- `with_npc_cut_out.py`: unified runtime entry for cut-out generation.
- `with_npc_followline.py`: unified runtime entry for lane-follow generation.
- `with_npc_lead.py`: unified runtime entry for lead-vehicle generation.
- `with_npc_linechange.py`: unified runtime entry for lane-change generation.

These are the recommended external interfaces for batch execution and scripting.

### 2.2 `common/` (Core Solvers/Generators)

This folder contains the core generation logic. It mainly uses kinematics + lane constraints + Z3 constraints to solve waypoint sequences.

- `generate_cut_in(...)`
- `generate_cut_out(...)`
- `generate_followline(...)`
- `generate_lead(...)`
- `generate_linechange(...)`

Typical responsibilities in these modules:

- Sample source trajectories with random temporal stride.
- Build dynamic constraints (position, speed, heading, lane validity, spacing).
- Solve feasible waypoints using Z3.
- Densify trajectory points and export to `.pkl`/`.npy`.
- Transfer trajectories to final output format via `trajectory_transfer.new_main.transfer`.

### 2.3 `scenarios/*_scenarios/scene_factory.py`

Each scene factory maps `seq` to concrete scenario payloads.

Each payload typically includes:

- `frame_start`
- `roads` (lane predicates)
- `roadinfo` (reference trajectories per lane)
- `ego_idx`
- `npcs` (paths of NPC trajectory artifacts to copy)
- `getderivative`
- `config` (scenario-specific generator arguments)

For the five documented families, the supported sequence set is:

`[1, 2, 3, 4, 5, 8, 10, 13, 14, 18]`

### 2.4 `curves/`

Contains map geometry and lane-related helpers per sequence (road lines, derivatives, lane membership predicates).

### 2.5 `trajectory_transfer/`

Post-processes generated trajectories into final output artifacts used by downstream consumers.

### 2.6 `utils/`

General utility code for conversion and value handling (`pkl2npy`, z3 value extraction, etc.).

---

## 3. Five Entry Functions: Invocation and Call Flow

All five top-level files expose `func(...)` as the programmatic entry function and `main()` as CLI entry.

Common behavior:

- Input `seq` selects scenario template from `scene_factory`.
- Optional runtime overrides can replace scene defaults (e.g., `roadinfo`, `ego_idx`, `npcs`, `roads`, `start_road_idx`).
- Return value:
  - `0` (or non-`-1`) = success
  - `-1` = no satisfiable solution for this random trial
- CLI `--retry` loops until success.

---

### 3.1 `with_npc_cut_in.func(...)`

**Signature**

```python
func(uid, save_dir, seq, roadinfo=None, ego_idx=None, npcs=None, roads=None, start_road_idx=None)
```

**Internal call chain**

1. `build_cut_in_scene(seq, SCRIPT_DIR)` loads baseline scene payload.
2. If `start_road_idx` is provided, it overrides `config["init_road_idx"]`.
3. Calls `generate_cut_in(...)` with scene fields + overridden values.

**CLI usage**

```bash
python with_npc_cut_in.py --seq 1 --uid 0 --save_dir outputs/cut_in
python with_npc_cut_in.py --seq 1 --uid 0 --save_dir outputs/cut_in --retry
python with_npc_cut_in.py --seq 18 --uid 0 --save_dir outputs/cut_in --start_road_idx 1
```

**Python usage**

```python
from with_npc_cut_in import func
ret = func(uid=0, save_dir="outputs/cut_in", seq=1, start_road_idx=1)
```

---

### 3.2 `with_npc_cut_out.func(...)`

**Signature**

```python
func(uid, save_dir, seq, roadinfo=None, ego_idx=None, npcs=None, roads=None, start_road_idx=None)
```

**Internal call chain**

1. `build_cut_out_scene(seq, SCRIPT_DIR)` loads cut-out scene payload.
2. If `start_road_idx` is provided:
   - override `config["init_road_idx"]`
   - when original `target_road_mode == "adjacent_random"`, convert to fixed target lane (`target_road_mode="fixed"`, `target_road_idx=start_road_idx+1` clamped to valid range)
3. Calls `generate_cut_out(...)`.

**CLI usage**

```bash
python with_npc_cut_out.py --seq 2 --uid 0 --save_dir outputs/cut_out
python with_npc_cut_out.py --seq 2 --uid 0 --save_dir outputs/cut_out --retry
python with_npc_cut_out.py --seq 2 --uid 0 --save_dir outputs/cut_out --start_road_idx 0
```

**Python usage**

```python
from with_npc_cut_out import func
ret = func(uid=0, save_dir="outputs/cut_out", seq=2, start_road_idx=0)
```

---

### 3.3 `with_npc_followline.func(...)`

**Signature**

```python
func(uid, save_dir, seq, roadinfo=None, ego_idx=None, npcs=None, roads=None, start_road_idx=None)
```

**Internal call chain**

1. `build_followline_scene(seq, SCRIPT_DIR)` loads followline payload.
2. Runtime ego lane index:
   - default: `scene["ego_idx"]`
   - if `ego_idx` provided: use it
   - if `start_road_idx` provided: it overrides the runtime ego lane index
3. Calls `generate_followline(...)` with `road_ref_lines`, follow offsets, and speed/range config from scene.

**CLI usage**

```bash
python with_npc_followline.py --seq 3
python with_npc_followline.py --seq 3 --uid 5 --save_dir outputs/followline --retry
python with_npc_followline.py --seq 3 --uid 5 --save_dir outputs/followline --retry --max_retries 20 --start_road_idx 1
```

Notes:

- If `--save_dir` is omitted, default path is `with_npc/{seq}_followline`.
- `--max_retries` limits retry loops when `--retry` is enabled.

**Python usage**

```python
from with_npc_followline import func
ret = func(uid=5, save_dir="outputs/followline", seq=3, start_road_idx=1)
```

---

### 3.4 `with_npc_lead.func(...)`

**Signature**

```python
func(uid, save_dir, seq, roadinfo=None, ego_idx=None, npcs=None, roads=None, start_road_idx=None)
```

**Internal call chain**

1. `build_lead_scene(seq, SCRIPT_DIR)` loads lead scene payload.
2. If `start_road_idx` is provided, it overrides `config["init_road_idx"]`.
3. Calls `generate_lead(...)` with lane-keeping constraints.

**CLI usage**

```bash
python with_npc_lead.py --seq 4 --uid 0 --save_dir outputs/lead
python with_npc_lead.py --seq 4 --uid 0 --save_dir outputs/lead --retry --start_road_idx 0
```

**Python usage**

```python
from with_npc_lead import func
ret = func(uid=0, save_dir="outputs/lead", seq=4, start_road_idx=0)
```

---

### 3.5 `with_npc_linechange.func(...)`

**Signature**

```python
func(uid, save_dir, seq, roadinfo=None, ego_idx=None, npcs=None, roads=None, start_road_idx=None)
```

**Internal call chain**

1. `build_linechange_scene(seq, SCRIPT_DIR)` loads line-change payload.
2. If `start_road_idx` is provided:
   - override `config["init_road_idx"]`
   - if lane count is 2, auto-set `config["dst_road_idx"] = 1 - start_road_idx`
3. Calls `generate_linechange(...)`, which supports multiple planning modes (for example, toggling with/without transition points).

**CLI usage**

```bash
python with_npc_linechange.py --seq 10 --uid 1 --save_dir outputs/linechange
python with_npc_linechange.py --seq 10 --uid 1 --save_dir outputs/linechange --retry --start_road_idx 0
```

**Python usage**

```python
from with_npc_linechange import func
ret = func(uid=1, save_dir="outputs/linechange", seq=10, start_road_idx=0)
```

---

## 4. Command-Line Summary

Common arguments across the five entry scripts:

- `--seq` (required): scenario sequence ID
- `--uid` (optional, default `0`): unique suffix for output naming
- `--save_dir` (required in 4 scripts, optional in followline): output directory
- `--retry` (optional): loop until a satisfiable run is found
- `--start_road_idx` (optional): override initial/start lane behavior

Extra argument in followline:

- `--max_retries`: max retry count when `--retry` is set

---

## 5. Recommended Usage Pattern

1. Start with one `seq` and one scenario family.
2. Run once without `--retry` to validate path/data dependencies.
3. Enable `--retry` for production generation.
4. If lane initialization is unstable for your case, set `--start_road_idx` explicitly.
5. Keep outputs separated by family and sequence for reproducibility.

---

## 6. Minimal End-to-End Example

```bash
python with_npc_cut_in.py --seq 1 --uid 100 --save_dir outputs/cut_in --retry
python with_npc_cut_out.py --seq 1 --uid 100 --save_dir outputs/cut_out --retry
python with_npc_followline.py --seq 1 --uid 100 --save_dir outputs/followline --retry --max_retries 30
python with_npc_lead.py --seq 1 --uid 100 --save_dir outputs/lead --retry
python with_npc_linechange.py --seq 1 --uid 100 --save_dir outputs/linechange --retry
```

This produces one generated batch per scenario family under the specified output directories.
