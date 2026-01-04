% Ultrasonic Distance -> Target Position -> PID -> Motor Command (Simulation)
% MATLAB simulation equivalent of Arduino slide logic.

clear; clc; close all;


% Settings

TARGET_MIN = 0;
TARGET_MAX = 1200;

dt = 0.01;           % 10 ms loop
steps = 1200;        % 12 seconds
t = (0:steps-1) * dt;

MAX_COUNTS_PER_SEC = 400;  % motor strength (tune)

% PID constants (from slides)
kp = 0.5;
kd = 0.017;
ki = 0.0;

eintegral = 0;
ePrev = 0;

% Example distance (cm): simulated wave 1..9 cm, clipped to 0..10
distance_cm = 5 + 4*sin(2*pi*0.1*t);
distance_cm = min(max(distance_cm, 0), 10);


% State

pos = 0;

% Logs
target_log = zeros(steps,1);
pos_log = zeros(steps,1);
dist_log = zeros(steps,1);
u_log = zeros(steps,1);
pwr_log = zeros(steps,1);
dir_log = zeros(steps,1);


% Helper functions (local)

constrain = @(x, lo, hi) min(max(x, lo), hi);
map_range = @(x, in_min, in_max, out_min, out_max) ...
    (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


% Main loop

for k = 1:steps
    dist = distance_cm(k);
    dist = constrain(dist, 0, 10);

    % 2) distance -> target mapping
    target = map_range(dist, 0, 10, TARGET_MIN, TARGET_MAX);
    target = constrain(target, TARGET_MIN, TARGET_MAX);

    % 3) PID control
    e = pos - target;
    dedt = (e - ePrev) / dt;
    eintegral = eintegral + e * dt;
    u = kp*e + kd*dedt + ki*eintegral;
    ePrev = e;

    % 4) motor power + direction
    pwr = abs(u);
    if pwr > 255
        pwr = 255;
    end

    if u < 0
        dir = -1;
    else
        dir = 1;
    end

    % 5) simulated encoder update
    speed = (pwr / 255) * MAX_COUNTS_PER_SEC;   % counts/sec
    pos = pos + dir * speed * dt;

    % logs
    target_log(k) = target;
    pos_log(k) = pos;
    dist_log(k) = dist;
    u_log(k) = u;
    pwr_log(k) = pwr;
    dir_log(k) = dir;

    % Serial-plotter-like print (optional)
    % fprintf("Target:%d\tPosition:%d\tDistance:%.2f\n", round(target), round(pos), dist);
end


% Plots

figure; plot(t, dist_log); grid on;
xlabel('Time (s)'); ylabel('Distance (cm)');
title('Distance input');

figure; plot(t, target_log, t, pos_log); grid on;
xlabel('Time (s)'); ylabel('Encoder counts');
title('Target vs Position');
legend('Target', 'Position');

figure; plot(t, u_log); hold on; plot(t, pwr_log); grid on;
xlabel('Time (s)'); title('Control signal');
legend('u (PID)', 'pwr (0..255)');
