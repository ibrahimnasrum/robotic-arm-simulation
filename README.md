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

## Group Members

1. IBRAHIM BIN NASRUM (2116467)
2. SHAREEN ARAWIE BIN HISHAM (2116943)
3. AHMAD HAFIZULLAH BIN IBRAHIM (2216185)
4. YOUSHA ABDULLAH (1929821)
5. MUHAMMAD SAAD ELDIN (G2321795)
