# 4-DOF ReBel Robotic Arm Simulation (MATLAB)
**Forward Kinematics (FK) • Inverse Kinematics (IK) • Trajectory Planning • 3D Animation • IK→FK Verification**

This project simulates a 4-DOF robotic arm in MATLAB and demonstrates the full workflow for a pick-and-place style motion:
- Solve joint angles using **Inverse Kinematics (IK)**
- Validate solutions using **Forward Kinematics (FK)**
- Generate smooth motion using **trajectory interpolation**
- Visualize motion using **3D animation**

---

## Files Overview

## Core Kinematics (The “Robotics Brain”)

### `fk_rebel4dof.m` — Forward Kinematics
**Purpose:** Computes the end-effector pose from joint angles.  
**Input:**  
- `theta = [θ1 θ2 θ3 θ4]` (degrees)  
- robot parameters `(d1, a2, a3, d4)`  

**Output:**  
- `T0_4` = 4×4 transformation matrix (base → end-effector)  
- (optional) intermediate joint positions for plotting/animation  

**Used in:**  
- 3D animation  
- IK verification  
- End-effector path plotting  

---

### `ik_rebel4dof.m` — Inverse Kinematics
**Purpose:** Solves joint angles needed to reach a target coordinate `(x, y, z)`.  
**Input:**  
- target `(x, y, z)`  
- desired orientation  
- robot parameters `(d1, a2, a3, d4)`  

**Output:**  
- `[θ1 θ2 θ3 θ4]` in degrees  

**Notes:** Includes a reachability check (throws an error if the target is out of reach).  
**Used in:**  
- `main_pick_and_place.m` for each waypoint  

---

## Trajectory Planning (Motion Smoothness)

### `plan_trajectory.m` — Trajectory Planning / Interpolation
**Purpose:** Generates a smooth joint trajectory (θ vs time) between waypoints.  
**How:** Uses interpolation such as `pchip` to avoid sharp jumps and produce smooth motion.  
**Input:**  
- joint waypoint angles  
- time settings  

**Output:**  
- `theta_trajectory` (N×4) for joints 1–4 across time steps  

**Used in:**  
- plotting θ vs time  
- 3D animation  

---

## Visualization (Show the Result)

### `animate_rebel4dof.m` — 3D Animation
**Purpose:** Animates the robot links in 3D over time.  
**How (per timestep):**
1. takes joint angles from `theta_trajectory`  
2. calls `fk_rebel4dof.m` to compute joint positions  
3. plots links/joints using `plot3`  

**Output:**  
- 3D animation showing pick-and-place motion  

---

## Verification / Debugging

### `verify_fk_example.m` — IK→FK Verification
**Purpose:** Confirms the IK angles are correct by running FK using the IK output.  
**How:**
1. uses IK results (θ values)  
2. runs FK and checks if end-effector position matches the target waypoint  

**Output:**  
- `T0_4` matrix  
- end-effector XYZ close to the target  

---

## How to Run
1. Open MATLAB and set this repository as your current folder
2. Run the main script:
```matlab
main_pick_and_place
