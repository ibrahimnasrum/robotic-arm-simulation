"""
Ultrasonic Distance -> Target Position -> PID -> Motor Command (Simulation)

This is a Python simulation equivalent of the Arduino slide logic:
1) distance measurement (here: simulated list OR optionally read from serial)
2) map distance (0–10 cm) -> target position (0–1200 counts)
3) PID control: u = kp*e + kd*dedt + ki*integral
4) motor power = abs(u) limited to 255, direction from sign(u)
5) encoder position tracking (simulated plant)
6) prints values like Arduino Serial Plotter style + plots

Author: (your name)
"""

import time
import math
import numpy as np
import matplotlib.pyplot as plt


# Helpers (Arduino-like)

def constrain(x, lo, hi):
    return max(lo, min(hi, x))

def map_range(x, in_min, in_max, out_min, out_max):
    # Arduino-like map, but float-safe
    if in_max == in_min:
        return out_min
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# PID Controller

class PID:
    def __init__(self, kp=0.5, kd=0.017, ki=0.0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.integral = 0.0
        self.prev_e = 0.0

    def step(self, e, dt):
        dedt = (e - self.prev_e) / dt if dt > 1e-9 else 0.0
        self.integral += e * dt
        u = self.kp * e + self.kd * dedt + self.ki * self.integral
        self.prev_e = e
        return u



# Simulation settings

TARGET_MIN, TARGET_MAX = 0, 1200
DT = 0.01                # 10 ms loop (like Arduino delay(10))
STEPS = 1200             # 12 seconds total (STEPS * DT)
MAX_COUNTS_PER_SEC = 400 # "motor strength" in encoder counts/sec (tune this)

pid = PID(kp=0.5, kd=0.017, ki=0.0)

# Example distance sequence (cm): simulate hand moving closer then farther
t = np.arange(STEPS) * DT
distance_cm = 5 + 4*np.sin(2*np.pi*0.1*t)   # ranges roughly 1..9 cm
distance_cm = np.clip(distance_cm, 0, 10)


# State variables (encoder pos)

pos = 0.0

# Logs
log_target = []
log_pos = []
log_dist = []
log_u = []
log_pwr = []
log_dir = []


# Main loop (simulation)

for k in range(STEPS):
    dist = float(distance_cm[k])
    dist = constrain(dist, 0.0, 10.0)

    # 2) distance -> target mapping
    target = map_range(dist, 0.0, 10.0, TARGET_MIN, TARGET_MAX)
    target = constrain(target, TARGET_MIN, TARGET_MAX)

    # 3) PID
    e = pos - target
    u = pid.step(e, DT)

    # 4) Motor control (power + direction)
    pwr = abs(u)
    if pwr > 255:
        pwr = 255
    direction = -1 if u < 0 else 1

    # 5) "Plant" update (simulate encoder counts change)
    # convert pwr (0..255) to speed (counts/sec)
    speed = (pwr / 255.0) * MAX_COUNTS_PER_SEC
    pos += direction * speed * DT

    # Serial-plotter-like output (optional)
    # print(f"Target:{int(target)}\tPosition:{int(pos)}\tDistance:{dist:.2f}")

    # logs
    log_target.append(target)
    log_pos.append(pos)
    log_dist.append(dist)
    log_u.append(u)
    log_pwr.append(pwr)
    log_dir.append(direction)


# Plot results

plt.figure()
plt.plot(t, log_dist)
plt.xlabel("Time (s)")
plt.ylabel("Distance (cm)")
plt.title("Distance input")
plt.grid(True)

plt.figure()
plt.plot(t, log_target, label="Target (counts)")
plt.plot(t, log_pos, label="Position (counts)")
plt.xlabel("Time (s)")
plt.ylabel("Encoder counts")
plt.title("Target vs Position")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(t, log_u, label="u (PID output)")
plt.plot(t, log_pwr, label="pwr (0..255)")
plt.xlabel("Time (s)")
plt.title("Control signal")
plt.legend()
plt.grid(True)

plt.show()
