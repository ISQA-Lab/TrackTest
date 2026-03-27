# TrackTest: Trajectory Generation and Prioritization for Testing MOT Systems

------

## Overview

TrackTest is an automated testing framework designed to evaluate **Multi-Object Tracking (MOT)** systems in autonomous driving. Existing evaluation approaches for MOT systems rely on manually annotated real-world datasets, which are costly to scale, behaviorally imbalanced, and insufficient in covering rare but safety-critical scenarios, while also lacking trajectory-level evaluation. To address these limitations, we propose **TrackTest**, an automated testing framework that generates physically feasible and behaviorally diverse trajectories from real-world seed data using constraint-based optimization, and prioritizes high-risk scenarios through trajectory-level difficulty metrics. Operating in an offline testing setting, TrackTest renders these scenarios into multi-modal data (e.g., images and LiDAR) for evaluation, enabling systematic exploration of challenging corner cases and effectively uncovering diverse tracking failures in state-of-the-art MOT systems.

------

## Installation

------

### Environment

- Python 3.7
- PyTorch 1.8

Experiments were conducted on:

- NVIDIA RTX 3050 GPU (4GB VRAM) 

### Dependencies

Run the following command to install the dependencies

```
pip install -r requirements.txt


```

### Install Multitest
Refer to this repository (https://github.com/MSFTest/MultiTest) to configure MultiTest as the data rendering engine.


### Install blender

MultiTest leverage blender, an open-source 3D computer graphics software, to build virtual camera sensor.

install blender>=3.3.1

------

## Dataset

Download KITTI datasets from this [link]([The KITTI Vision Benchmark Suite](https://www.cvlibs.net/datasets/kitti/))

------


### Model Configuration

| PointTrack | https://github.com/detectRecog/PointTrack         |
| ---------- | --------------------------------------------------- |
| CasTrack   | https://github.com/hailanyi/3D-Multi-Object-Tracker |
| JMODT      | https://github.com/Kemo-Huang/JMODT                 |
| MCTrack    | https://github.com/megvii-research/MCTrack          |





# Supporting Resources

We have uploaded high-definition GIF examples (`GIF.rar`) and the corresponding point cloud-image dataset (`KITTI_Tracking_demo.rar`) to Google Drive, which includes six types of constraint behaviors.





## **Experimental Data**

The all data required to reproduce the experiments is stored at::

https://drive.google.com/drive/folders/1SoeTrFn7AuWfXlLqX3ka8jv3sw30e-Ij?usp=drive_link

Including:

RQ1: High-definition GIF examples(RQ1_gif_example.rar) and corresponding point cloud-image data(RQ1_example_data.zip);

data.zip: Trajectories

seed.rar: Required experiment seed files



------







## Experimental Results

------



### RQ1: Trajectory Generation

Low-resolution GIF examples have been uploaded to GitHub, and high-resolution GIF examples along with the corresponding point cloud-image data have been uploaded to Google Drive.

To run the trajectory generation path,you should excute like this.

this loads the seq=1 scene from scene_factory.py, applies runtime overrides (for example --start_road_idx if provided), calls generate_cut_in(...) in cut_in_common.py to sample and solve feasible motion constraints, then exports trajectory artifacts (intermediate .pkl/.npy plus transferred final outputs), where a return code of 0 means success and -1 means the current trial is unsatisfied (use --retry to keep trying until success).



```
python with_npc_cut_in.py --seq 1 --uid 0 --save_dir outputs/cut_in

or keep retrying until a feasible solution is found:

python with_npc_cut_in.py --seq 1 --uid 0 --save_dir outputs/cut_in --retry
```

------



------



### RQ2: Trajectory Prioritization

To compute the priority score for one trajectory, run the unified metric entry with the trajectory file and optional metric inputs.
This call evaluates available sub-metrics (angle, jerk, distance, occlusion, truncation) and returns a merged composite_score, which can be used as the trajectory priority score (higher means higher priority).



```
python -m metric.metric_entry \
  --npy-path data/your_traj.npy \
  --theta-thresh 0.1*pi \
  --jerk-thread 2.0 \
  --distance-directory data/scene_json \
  --start-frame start_frame \
  --end-frame 100 \
  --dis1 5 \
  --dis2 50 \
  --occlusion-folder data/occlusion \
  --label-folder data/label \
  --occlusion-thresh 0.5 \
  --truncation-thresh 0.5 \
  --truncation-folder-path data/truncation \
  --truncation-trajectory-path data/your_traj.npy \
  --weights "{\"angle_score\":1,\"jerk_score\":1,\"distance_score\":1,\"occlusion_score\":1,\"truncation_score\":1}"
```

------



------



### RQ3: Ablation Study

To obtain the results for RQ3, first follow the same procedure used in RQ2 to compute trajectory scores. Then, rank all trajectories based on each individual metric score (e.g., distance, angle, jerk, occlusion, truncation) separately rather than using only a combined score. Finally, perform a correlation analysis between these metric-based rankings (or scores) and the corresponding test outcomes to quantify how strongly each metric is associated with actual performance and to identify which metric is most predictive.
