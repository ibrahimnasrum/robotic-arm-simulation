# 4-DOF ReBel Robotic Arm Simulation (MATLAB)
**Forward Kinematics (FK) • Inverse Kinematics (IK) • IK→FK Verification • Trajectory Planning • 3D Animation**

A MATLAB project that models a **4-DOF ReBel robotic arm** and simulates a **pick-and-place** motion using classical robotics fundamentals.  
Includes **DH-style transforms**, **FK/IK**, **verification of IK using FK**, **smooth joint trajectory interpolation**, and **3D visualization/animation**—structured as modular MATLAB scripts for clarity and reuse.

> ✅ Portfolio note: This project showcases end-to-end robotics workflow: modeling → solving → verifying → planning → visualizing.

---

## Highlights (Why this project is valuable)
- Built a full kinematics pipeline (**DH → FK → IK**) for a 4-DOF manipulator
- Verified IK solutions by re-running FK (**IK→FK consistency check**) to confirm end-effector reaches target coordinates
- Planned smooth motion using **PCHIP interpolation** and visualized both **θ vs time** and **3D end-effector paths**
- Produced a clean modular codebase (separate FK, IK, trajectory, and animation files)

---

## Quick Start (Recommended)
1. Open MATLAB and set this repository folder as your **Current Folder**
2. Run the main script:

```matlab
main_pick_and_place
```
### Expected Outputs

- Command Window: IK angles + FK verification result
- Figures:
  - θ vs time (Joint 1–4)
  - End-effector 3D path (Cartesian trajectory)
  - 3D robot animation (if enabled)

---

## Repository Structure (What each file does)

### Main Scripts
- main_pick_and_place.m
Main driver script. Runs the complete workflow:
1. define robot parameters & waypoints
2. compute IK for start/end points
3. verify IK using FK
4. generate smooth joint trajectories
5. plot θ vs time and end-effector path
6. animate robot in 3D
- robotic_arm_rebel4dof_main.m
Alternative main script / consolidated demo version (useful as backup).

---

### Core Kinematics
- fk_rebel4dof.m — Forward Kinematics
Computes the chained transformation matrix **T0_4** (base → end-effector) using DH-style transforms.
Also returns intermediate joint positions for plotting and animation.

- ik_rebel4dof.m — Inverse Kinematics
Solves joint angles **[θ1 θ2 θ3 θ4]** (degrees) for a target point (x, y, z).
Includes a reachability check (throws an error if target is out of workspace).

### Planning & Visualization

- plan_trajectory.m — Trajectory Planning
Generates a smooth joint trajectory using interpolation **(PCHIP)** between waypoints.
Output is used for θ-vs-time plots and animation playback.
- animate_rebel4dof.m — 3D Animation
Animates the robot by calling FK at each timestep and plotting link/joint positions.

### Validation / Testing

- verify_fk_example.m — Verification Utility
Confirms IK results by plugging solved angles into FK and checking the resulting end-effector pose.

--- 

## Documentation
- Full_Report_Robotics Project.pdf — Full report/documentation (derivation, results, plots)

---

## Robot Parameters (mm)

The simulation uses the following robot dimensions:
- d1 = 252 (base height)
- a2 = 237 (link length)
- a3 = 244 (link length)
- d4 = 142 (end-effector / gripper offset)

---

## Pick-and-Place Waypoints (mm)

Example tasks included in the main script:
- Task 1: WP1 [-200, 20, 370.5] → Rack2 [-180, 70, 550]
- Task 2: WP2 [-200, 50, 370.5] → Rack16 [180, 70, 450]

---

## Outputs (What you will see)

- IK solutions (deg) printed for each waypoint
- FK verification transformation matrix T0_4
- θ vs time plots for joints 1–4 (Task 1 vs Task 2)
- 3D end-effector trajectory plot
- 3D robot animation showing link motion
  
---

## Notes / Assumptions

- The IK solution uses a standard geometric approach and assumes a simplified wrist orientation constraint.
- You can extend this project by adding:
  - joint limits and multiple IK branches (elbow-up / elbow-down)
  - velocity/acceleration constraints
  - workspace boundary / collision checks
  - Robotics Toolbox integration for dynamics
---

## Group Members

1. IBRAHIM BIN NASRUM (2116467)
2. SHAREEN ARAWIE BIN HISHAM (2116943)
3. AHMAD HAFIZULLAH BIN IBRAHIM (2216185)
4. YOUSHA ABDULLAH (1929821)
5. MUHAMMAD SAAD ELDIN (G2321795)
