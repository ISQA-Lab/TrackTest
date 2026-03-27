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

All experimental data is stored at:

https://drive.google.com/drive/folders/1SoeTrFn7AuWfXlLqX3ka8jv3sw30e-Ij?usp=drive_link







## Experimental Results

------



### RQ1: Trajectory Generation

Low-resolution GIF examples have been uploaded to GitHub, and high-resolution GIF examples along with the corresponding point cloud-image data have been uploaded to Google Drive.



------



### RQ2: Trajectory Prioritization



------



### RQ3: Ablation Study



```

```

------

