# robotic-arm-simulation



\# 4-DOF ReBel Robotic Arm Simulation (MATLAB) — FK, IK, Trajectory \& 3D Pick-and-Place



This project models and simulates a \*\*4-DOF ReBel robotic arm\*\* for a \*\*pick-and-place\*\* task using MATLAB.  

It covers the full workflow from \*\*Denavit–Hartenberg (DH) modeling\*\* and \*\*Forward Kinematics (FK)\*\* to \*\*Inverse Kinematics (IK)\*\*, \*\*trajectory planning\*\*, and \*\*3D visualization\*\* of the arm executing movements between workstations and rack locations.



> Focus: practical robotics fundamentals (kinematics + trajectory + simulation) with clear verification and visualization.



---



\## Key Features

\- \*\*DH Parameter Modeling\*\* and transformation matrices for a 4-DOF manipulator.:contentReference\[oaicite:2]{index=2}:contentReference\[oaicite:3]{index=3}

\- \*\*Forward Kinematics (FK)\*\* to compute end-effector pose using chained transforms.:contentReference\[oaicite:4]{index=4}

\- \*\*Inverse Kinematics (IK)\*\* to compute joint angles for target (x, y, z) positions with reachability checks.:contentReference\[oaicite:5]{index=5}:contentReference\[oaicite:6]{index=6}

\- \*\*Verification step\*\*: validates IK solutions by re-running FK to confirm the resulting end-effector pose.:contentReference\[oaicite:7]{index=7}:contentReference\[oaicite:8]{index=8}

\- \*\*Trajectory planning\*\* (θ vs time) using interpolation (e.g., `pchip`) for smooth joint motion across waypoints.:contentReference\[oaicite:9]{index=9}

\- \*\*3D robot visualization \& animation\*\*, including link/joint plotting and end-effector trajectory display.:contentReference\[oaicite:10]{index=10}:contentReference\[oaicite:11]{index=11}



---



\## Robot Parameters (from DH model)

\- `d1 = 252 mm` (base height):contentReference\[oaicite:12]{index=12}

\- `a2 = 237 mm` (link length):contentReference\[oaicite:13]{index=13}

\- `a3 = 244 mm` (link length):contentReference\[oaicite:14]{index=14}

\- `d4` (gripper offset used in simulation):contentReference\[oaicite:15]{index=15}



---



\## Pick-and-Place Scenario

Targets include \*\*two pickup points\*\* and \*\*rack placement points\*\* (rack grid context in the report).:contentReference\[oaicite:16]{index=16}



Example target coordinates (mm):

\- `WP1 = \[-200,  20, 370.5]` (Pickup 1):contentReference\[oaicite:17]{index=17}

\- `WP2 = \[-200,  50, 370.5]` (Pickup 2):contentReference\[oaicite:18]{index=18}

\- `Rack2 = \[-180, 70, 550]` (Place 1):contentReference\[oaicite:19]{index=19}

\- `Rack16 = \[180, 70, 450]` (Place 2):contentReference\[oaicite:20]{index=20}



Sample IK results (degrees) included in the report:

\- WP2: θ1=165.96°, θ2=-31.96°, θ3=120.77°, θ4=-254.77°:contentReference\[oaicite:21]{index=21}

\- Rack2: θ1=158.75°, θ2=13.87°, θ3=84.84°, θ4=-257.46°:contentReference\[oaicite:22]{index=22}

\- Rack16: θ1=21.25°, θ2=-10.38°, θ3=109.81°, θ4=-120.68°:contentReference\[oaicite:23]{index=23}



---

