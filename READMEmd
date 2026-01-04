# PID Ultrasonic Motor Control Simulation (Python + MATLAB)

This repository contains a simulation of a closed-loop position control system:
**Ultrasonic distance (cm) → Target encoder counts → PID controller → Motor command → Simulated encoder position**.

## Files
- `pid_ultrasonic_motor_sim.py` — Python simulation + plots
- `pid_ultrasonic_motor_sim.m` — MATLAB simulation + plots
- `full_report_robotics_project.pdf` — Full report/documentation

## Method (Summary)
1. Measure distance (0–10 cm)
2. Map distance to target position (0–1200 encoder counts)
3. Compute PID control signal: `u = kp*e + kd*dedt + ki*∫e`
4. Convert `u` to motor power (0–255) and direction
5. Update encoder position (simulation plant model)
6. Plot distance, target vs position, and control signal

## How to Run
### Python
```bash
python pid_ultrasonic_motor_sim.py
