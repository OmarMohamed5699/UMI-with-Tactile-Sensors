# UMI + Tactile Sensing for Robotic Imitation Learning

Extending the Universal Manipulation Interface (UMI) with PapillArray tactile
sensors to enable synchronized visual-tactile data collection for contact-rich
imitation learning.

> Master's Project · Queensland University of Technology, 2025
> Author: Omar Mohamed · Supervised by Prof. Niko Suenderhauf

---

## 📽️ Demo

![Assembled UMI system with tactile sensors](assets/system.jpg)

---

## Problem & approach

Vision-only imitation learning systems cannot capture force, slip, or texture
information essential for delicate manipulation tasks. This project integrates
two PapillArray tactile sensors into the portable UMI gripper, adding touch
as a second modality alongside the GoPro Hero 9 (60 FPS, 155° fisheye lens).

**The core challenge:** visual data (60 FPS) and tactile data (up to 1000 Hz)
run on independent system clocks — unsynchronised data produces incorrect
state representations and degrades policy learning.

**Solution — three interconnected contributions:**

1. **Hardware integration** — custom 3D-printed mounting brackets (PLA+ body,
   TPU fingers) attach two PapillArray sensors to the UMI gripper fingertips,
   preserving portability and keeping sensors visible in the camera's field of view

2. **QR code-based synchronisation** — ROS2 timestamps are encoded into QR codes
   displayed at the start and end of each demonstration and filmed by the GoPro,
   establishing temporal anchor points without any specialised hardware

3. **ROS2 data pipeline** — a C++ node acquires raw tactile data at 1000 Hz;
   a Python node republishes at 60 Hz; a post-processing script interpolates
   tactile measurements to each video frame and appends them to the UMI Zarr
   dataset alongside camera observations and gripper trajectories

---

## Results

Evaluated across 24 demonstrations of a multi-phase manipulation task:
cup reorientation, grasping, and placement on a plate.

### Synchronisation accuracy

| Metric                        | Value      |
|-------------------------------|------------|
| Mean error                    | 12.43 ms   |
| Median error                  | 9.15 ms    |
| Standard deviation            | 10.23 ms   |
| Minimum error                 | 0.03 ms    |
| Maximum error                 | 41.5 ms    |
| 90th percentile               | 26.0 ms    |
| Demonstrations < 10 ms        | 13/24 (54.2%) |
| Demonstrations < 20 ms        | 19/24 (79.2%) |
| Demonstrations < 100 ms       | 24/24 (100%)  |
| QR detection success rate     | 85.7% (24/28) |

> Frame duration at 60 FPS = 16.67 ms.
> Mean error of 12.43 ms represents sub-frame synchronisation accuracy.
> All 24 demonstrations achieved the <100 ms success criterion.

![Synchronisation error across 24 demonstrations](assets/sync_error.png)

### Key findings

- 54.2% of demonstrations achieved sub-10 ms accuracy — near-perfect
  temporal alignment under optimal QR capture conditions
- Highest errors (Demo_1: 26.4 ms, Demo_10: 29.3 ms, Demo_22: 41.5 ms)
  correlated with motion blur or non-perpendicular QR capture angles
- Sub-20 ms accuracy exceeds requirements for human-like manipulation
  learning (human tactile reaction time ≈ 150–200 ms)

---

## ⚙️ Hardware

| Component              | Specification                              |
|------------------------|--------------------------------------------|
| Tactile sensors        | 2× PapillArray (Contactile), 3×3 pillar array, up to 1000 Hz |
| Tactile data fields    | 3D forces, 3D displacements, slip detection, contact state, global torque, friction estimate |
| Camera                 | GoPro Hero 9, 60 FPS, 155° fisheye lens    |
| Gripper                | Modified UMI gripper (PLA+ body, TPU fingers) |
| Gripper redesign tool  | SolidWorks                                 |
| Data format            | Zarr (UMI SLAM pipeline compatible)        |

---

## ⚙️ Installation
```bash
git clone https://github.com/YOUR_USERNAME/umi-tactile.git
cd umi-tactile

# ROS2 Humble required
conda create -n umi-tactile python=3.10
conda activate umi-tactile

pip install -r requirements.txt
```

> Requires ROS 2 Humble, PapillArray PTSDK (Contactile), and UMI pipeline.

---

## ▶️ Usage
```bash
# 1. Launch tactile sensor node (C++)
ros2 run tactile_driver sensor_node

# 2. Launch Python data organisation node
ros2 run tactile_driver data_node

# 3. Launch QR code timestamp generator
ros2 run sync qr_generator

# 4. Start ROS2 bag recording
ros2 bag record -o demo_session /tactile /qr_timestamps

# 5. Post-process: detect QR codes, synchronise, integrate into Zarr
python scripts/process_demos.py --demo_dir /path/to/demos
```

---

## 📁 Repo structure
```
umi-tactile/
├── tactile_driver/      # ROS2 C++ + Python nodes for PapillArray
├── sync/                # QR code generator + temporal transformation
├── scripts/             # Post-processing: QR detection, interpolation,
│                        # Zarr integration
├── assets/              # System photos and sync error plot
└── README.md
```

---

## 🙏 References & acknowledgements

- UMI: Chi et al., arXiv:2402.10329
- PapillArray sensor: Contactile Pty Ltd, PTS 2.0 Spec
- 3D-ViTac: Huang et al., CoRL 2024
- Touch in the wild: Zhu et al., RSS Workshop 2025

Supervised by Prof. Niko Suenderhauf · QUT Centre for Robotics
School of Electrical Engineering and Robotics, QUT, 2025


  year   = {2025}
}
```
