# RoArm-M3 Pro Sim-to-Real Pick-and-Place (ROS2 + MoveIt)

This repository contains the full implementation and report for an Advanced Robotics project on:

- kinematic modeling of **RoArm-M3 Pro**
- simulation and motion planning in **ROS2 / MoveIt / RViz**
- custom ROS2 node for autonomous **pick-and-place**
- transfer from simulation to the real robot (**Sim-to-Real**)

The original report is in Persian and is included in `report/`.

![Trajectory Preview](assets/trajectory_preview.png)

## Project Highlights

- Built a ROS2 package (`roarm_p`) for cyclic pick-and-place execution.
- Implemented arch-based transfer motion to reduce singularity-related failures.
- Added automatic mode switching logic in launch:
  - real robot mode if `/dev/ttyUSB0` is available
  - simulation mode otherwise
- Validated workflow from RViz simulation to physical RoArm-M3 Pro execution.

## Repository Structure

```text
.
├── roarm_p/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── src/
│   │   └── simple_pick_place.cpp
│   ├── launch/
│   │   └── auto_pick.launch.py
│   ├── config/
│   │   └── path_plan.json
│   ├── scripts/
│   │   └── visualize_path.py
│   └── notebooks/
│       └── workspace-simulation-monte-carlo.ipynb
├── report/
│   ├── project-report.docx
│   └── project-report.pdf
└── assets/
    └── trajectory_preview.png
```

## Requirements

- Ubuntu 22.04 (recommended)
- ROS2 Humble (or compatible)
- MoveIt2 and dependencies
- `roarm_moveit` and related RoArm packages from Waveshare setup
- Python 3 with `matplotlib`, `numpy`

## Build and Run

1. Put `roarm_p` inside your ROS2 workspace (example path from the project report):

```bash
mkdir -p ~/roarm_ws/src/roarm_main
cp -r roarm_p ~/roarm_ws/src/roarm_main/
```

2. Build:

```bash
cd ~/roarm_ws
colcon build --packages-select roarm_p
source install/setup.bash
```

3. Launch:

```bash
ros2 launch roarm_p auto_pick.launch.py
```

## Trajectory Visualization

Generate or view trajectory from the waypoint JSON:

```bash
python3 roarm_p/scripts/visualize_path.py
```

Save a static image:

```bash
python3 roarm_p/scripts/visualize_path.py --save assets/trajectory_preview.png
```

## Notes on Sim-to-Real

- The launch file checks `/dev/ttyUSB0` to infer hardware availability.
- In real mode it uses real hardware timing and the RoArm MoveIt launch pipeline.
- In simulation mode it enables fake hardware and simulated time.

## Authors

- Seyed Mahdi Sarfarazi
- Amin Parsafar
- Mohammad Hossein Ebrahimi

## References

- Waveshare RoArm-M3 Wiki: https://www.waveshare.com/wiki/RoArm-M3
- Tutorial video referenced in report: https://www.youtube.com/watch?v=aGMcTiRKnYQ
