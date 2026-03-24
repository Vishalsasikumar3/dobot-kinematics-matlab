# Dobot Magician Lite — Forward & Inverse Kinematics + Workspace Analysis

**RAS 545 · Assignment 2 · Arizona State University · Fall 2025**

> MATLAB-based kinematic modeling of the Dobot Magician Lite: geometric FK/IK derivation, URDF simulation via rigidBodyTree, animated trajectory following, and spherical-sweep workspace analysis — validated against the physical robot.

---

## Overview

This assignment derives and validates the forward and inverse kinematics of the Dobot Magician Lite (3-DOF + wrist) using geometric methods and MATLAB's Robotics System Toolbox. A URDF model built from CAD data enables simulation, IK solving, and workspace reachability analysis. All results were cross-checked against the physical robot.

---

## Repository Structure
```
.
├── forwardinverse_kinematics.m     # FK/IK derivation, URDF simulation, animated trajectory
├── workspace_analysis.m            # Spherical joint sweep → 3D workspace plot
├── Matlab_Assignment2.zip          # Full MATLAB project (URDF, helper functions, CAD)
├── Assignment_2_ras-Report.pdf     # 4-page report (IEEE format)
└── README.md
```

---

## Kinematics

### Forward Kinematics

End-effector pose computed by chaining homogeneous transforms across all links:
```
⁰T₄ = ⁰T₁ · ¹T₂ · ²T₃ · ³T₄
```

Each revolute joint contributes a rotation about Z:
```
Rz(θ) = [cos θ  -sin θ  0]
         [sin θ   cos θ  0]
         [0       0      1]
```

**Robot parameters (mm):**

| Parameter | Value |
|---|---|
| `a1` — base height | 53.5 mm |
| `a2` — upper arm | 150 mm |
| `a3` — forearm | 150 mm |
| `z_offset` | 53.5 mm |
| `r_const` — wrist offset | 90 mm |

### Inverse Kinematics

Geometric closed-form solution for target position `P = (x, y, z)`:

| Joint | Formula |
|---|---|
| θ₁ (base) | `atan2(y, x)` |
| θ₃ (elbow) | `π − acos((a₂² + a₃² − D²) / 2a₂a₃)` |
| θ₂ (shoulder) | `atan2(h, r) − acos((D² + a₂² − a₃²) / 2a₂D)` |
| θ₄ (wrist) | `−(θ₂ + θ₃)` |

where `D = √(r² + h²)`, `r = √(x² + y²)`, `h = z − d₁`

---

## FK Verification Results

| q (deg) | Status |
|---|---|
| [0, 0, 0] | Verified |
| [90, 20, 30] | Verified |
| [−2, 25, 23] | Verified |
| [31, 52, 42] | Verified |
| [27, 52, 24] | Verified |

---

## IK Verification Results

| Waypoint | x (mm) | y (mm) | z (mm) | Status |
|---|---|---|---|---|
| 1 | 240 | 0 | 150 | OK |
| 2 | 0 | 272.5 | 68.54 | OK |
| 3 | 300 | 50 | 100 | OK |
| 4 | 280 | −195 | 15 | OK |

---

## Workspace Analysis

`workspace_analysis.m` performs a spherical joint sweep across all revolute joints and plots the 3D reachable workspace color-coded by distance from the robot base.
```
samplesTheta = 20   (0 → π)
samplesPhi   = 40   (0 → 2π)
Total points = 800
Color map    = turbo (blue = near, red = far)
```

---

## Setup & Usage

### Requirements
- MATLAB R2022b or later
- Robotics System Toolbox
- `Dobot_magLite.urdf` (included in `Matlab_Assignment2.zip`)
- Helper functions: `fk_dobot.m`, `ik_dobot.m` (included in zip)

### Run FK/IK simulation
```matlab
% Update URDF path on line 9 of forwardinverse_kinematics.m
urdfPath = 'path/to/modelrobot.urdf';

% Then run
forwardinverse_kinematics
```

### Run workspace analysis
```matlab
% Place Dobot_magLite.urdf in working directory, then:
workspace_analysis
```

---

## Lessons Learned

- Geometric IK is faster and more predictable than numerical solvers for low-DOF arms like the Dobot
- URDF fixed-transform offsets must be non-zero for `rigidBodyTree` FK to match geometric FK
- Spherical sampling covers joint space more uniformly than random sampling near joint limits
- Warm-starting IK with the previous solution dramatically reduces convergence failures across waypoints

---

## Course Info

- **Course:** RAS 545 — Robotic and Autonomous Systems
- **Instructor:** Prof. Mostafa Yourdkhani
- **University:** Arizona State University, Tempe AZ
- **Semester:** Fall 2025
- **Grade:** 15 / 15
