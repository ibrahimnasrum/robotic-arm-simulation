%% 4-DOF ReBel Robotic Arm Simulation (MATLAB) - Single File
% FK + IK + Verification + Trajectory Planning + Theta vs Time + 3D Animation
% Author: <Your Name>

clc; clear; close all;

%% =========================
%  Robot Parameters (mm)
% ==========================
d1 = 252;    % base height
a2 = 237;    % link 2 length
a3 = 244;    % link 3 length
d4 = 142;    % gripper offset (end-effector)

desired_orientation_deg = 0; % set 0 if you want "horizontal"

%% =========================
%  Targets / Waypoints (mm)
% ==========================
WP1    = [-200,  20, 370.5];   % Pickup 1
WP2    = [-200,  50, 370.5];   % Pickup 2
Rack2  = [-180,  70, 550];     % Place 1
Rack16 = [ 180,  70, 450];     % Place 2

% Task 1: WP1 -> Rack2
% Task 2: WP2 -> Rack16
tasks(1).name = "Task 1 (WP1 -> Rack2)";
tasks(1).start = WP1;
tasks(1).goal  = Rack2;

tasks(2).name = "Task 2 (WP2 -> Rack16)";
tasks(2).start = WP2;
tasks(2).goal  = Rack16;

%% =========================
%  IK Results (Degrees)
% ==========================
fprintf("=== Inverse Kinematics Results (Degrees) ===\n");
for i = 1:numel(tasks)
    th_start = IK_ReBel4DOF(tasks(i).start(1), tasks(i).start(2), tasks(i).start(3), ...
                           desired_orientation_deg, d1, a2, a3, d4);

    th_goal  = IK_ReBel4DOF(tasks(i).goal(1), tasks(i).goal(2), tasks(i).goal(3), ...
                           desired_orientation_deg, d1, a2, a3, d4);

    tasks(i).th_start = th_start;
    tasks(i).th_goal  = th_goal;

    fprintf("%s\n", tasks(i).name);
    fprintf("  Start: θ1=%.2f°, θ2=%.2f°, θ3=%.2f°, θ4=%.2f°\n", th_start(1), th_start(2), th_start(3), th_start(4));
    fprintf("  Goal : θ1=%.2f°, θ2=%.2f°, θ3=%.2f°, θ4=%.2f°\n\n", th_goal(1), th_goal(2), th_goal(3), th_goal(4));
end

%% =========================
%  Verify IK using FK (Example: Task 2 Goal)
% ==========================
fprintf("=== Verify IK using FK (Example: Task 2 Goal) ===\n");
[T0_4, ~] = FK_ReBel4DOF(tasks(2).th_goal, d1, a2, a3, d4);
disp(T0_4);
fprintf("End-effector position (mm): X=%.3f, Y=%.3f, Z=%.3f\n\n", T0_4(1,4), T0_4(2,4), T0_4(3,4));

%% =========================
%  Trajectory Planning (pchip)
% ==========================
t_total  = 6;     % seconds
n_points = 100;
time_steps = linspace(0, t_total, n_points);

for i = 1:numel(tasks)
    joint_waypoints = [tasks(i).th_start; tasks(i).th_goal];   % [2 x 4]
    tasks(i).traj = planTrajectoryPCHIP(joint_waypoints, time_steps);
end

%% =========================
%  Plot Theta vs Time (Task 1 vs Task 2)
% ==========================
figure("Name","Theta vs Time (All Joints)");
for j = 1:4
    subplot(4,1,j);
    plot(time_steps, tasks(1).traj(:,j), "LineWidth", 1.5); hold on;
    plot(time_steps, tasks(2).traj(:,j), "--", "LineWidth", 1.5);
    grid on;
    xlabel("Time (s)");
    ylabel(sprintf("\\theta_%d (deg)", j));
    title(sprintf("Joint %d Trajectory", j));
    legend("Task 1","Task 2","Location","best");
end

%% =========================
%  End-effector Cartesian Trajectory (Optional Plot)
% ==========================
P_task1 = zeros(n_points,3);
P_task2 = zeros(n_points,3);

for k = 1:n_points
    T1 = FK_ReBel4DOF(tasks(1).traj(k,:), d1, a2, a3, d4);
    T2 = FK_ReBel4DOF(tasks(2).traj(k,:), d1, a2, a3, d4);
    P_task1(k,:) = T1(1:3,4)';
    P_task2(k,:) = T2(1:3,4)';
end

figure("Name","End-Effector Cartesian Trajectory");
plot3(P_task1(:,1), P_task1(:,2), P_task1(:,3), "LineWidth", 2); hold on;
plot3(P_task2(:,1), P_task2(:,2), P_task2(:,3), "--", "LineWidth", 2);
scatter3(tasks(1).start(1), tasks(1).start(2), tasks(1).start(3), 80, "filled");
scatter3(tasks(1).goal(1),  tasks(1).goal(2),  tasks(1).goal(3),  80, "filled");
scatter3(tasks(2).start(1), tasks(2).start(2), tasks(2).start(3), 80, "filled");
scatter3(tasks(2).goal(1),  tasks(2).goal(2),  tasks(2).goal(3),  80, "filled");
grid on; axis equal; view(3);
xlabel("X (mm)"); ylabel("Y (mm)"); zlabel("Z (mm)");
title("End-Effector Path (Task 1 & Task 2)");
legend("Task 1","Task 2","T1 Start","T1 Goal","T2 Start","T2 Goal","Location","best");

%% =========================
%  3D Animation (Optional)
%  Tukar TRUE/FALSE
% ==========================
DO_ANIMATE = true;

if DO_ANIMATE
    animateReBel4DOF(tasks(1).traj, d1, a2, a3, d4, "3D Animation - Task 1");
    animateReBel4DOF(tasks(2).traj, d1, a2, a3, d4, "3D Animation - Task 2");
end

disp("✅ Done. Simulation complete.");

%% ============================================================
%  Local Functions (in the same file)
%  MATLAB supports local functions in scripts (R2016b+)
% ============================================================

function theta_deg = IK_ReBel4DOF(x, y, z, desired_orientation_deg, d1, a2, a3, d4)
% Inverse kinematics (simplified 4-DOF):
% θ1 base yaw, θ2 shoulder, θ3 elbow, θ4 wrist orientation (approx)

    theta1 = atan2(y, x);  % rad

    r = sqrt(x^2 + y^2);
    s = (z - d1);

    % NOTE: d4 is kept for consistency, but this IK is simplified (no wrist-center offset model)

    D = (r^2 + s^2 - a2^2 - a3^2) / (2*a2*a3);
    if abs(D) > 1
        error("Target out of reach (|D| > 1). Check waypoint or link lengths.");
    end

    theta3 = atan2(sqrt(1 - D^2), D);  % elbow-up

    phi = atan2(s, r);
    psi = atan2(a3*sin(theta3), a2 + a3*cos(theta3));
    theta2 = phi - psi;

    desired_orientation = deg2rad(desired_orientation_deg);
    theta4 = desired_orientation - (theta1 + theta2 + theta3);

    theta_deg = [rad2deg(theta1), rad2deg(theta2), rad2deg(theta3), rad2deg(theta4)];
end

function [T0_4, P] = FK_ReBel4DOF(theta_deg, d1, a2, a3, d4)
% Forward kinematics based on your matrix form (degrees input)
% Returns transform and joint points for plotting

    t1 = theta_deg(1); t2 = theta_deg(2); t3 = theta_deg(3); t4 = theta_deg(4);

    T1_0 = [
        cosd(t1), 0,  sind(t1), 0;
        sind(t1), 0, -cosd(t1), 0;
        0,        1,  0,        d1;
        0,        0,  0,        1
    ];

    T2_1 = [
        cosd(t2), -sind(t2), 0, a2*cosd(t2);
        sind(t2),  cosd(t2), 0, a2*sind(t2);
        0,         0,        1, 0;
        0,         0,        0, 1
    ];

    T3_2 = [
        cosd(t3), -sind(t3), 0, a3*cosd(t3);
        sind(t3),  cosd(t3), 0, a3*sind(t3);
        0,         0,        1, 0;
        0,         0,        0, 1
    ];

    T4_3 = [
        cosd(t4), -sind(t4), 0, 0;
        sind(t4),  cosd(t4), 0, 0;
        0,         0,        1, d4;
        0,         0,        0, 1
    ];

    T0_1 = T1_0;
    T0_2 = T1_0 * T2_1;
    T0_3 = T0_2 * T3_2;
    T0_4 = T0_3 * T4_3;

    base = [0;0;0];
    P1 = T0_1(1:3,4);
    P2 = T0_2(1:3,4);
    P3 = T0_3(1:3,4);
    P4 = T0_4(1:3,4);

    P = [base, P1, P2, P3, P4];
end

function theta_traj = planTrajectoryPCHIP(joint_waypoints_deg, time_steps)
% Interpolate joint waypoints using pchip
    t_total = time_steps(end);
    N = size(joint_waypoints_deg,1);
    t_wp = linspace(0, t_total, N);

    theta_traj = zeros(length(time_steps), 4);
    for j = 1:4
        theta_traj(:,j) = interp1(t_wp, joint_waypoints_deg(:,j), time_steps, 'pchip');
    end
end

function animateReBel4DOF(theta_traj_deg, d1, a2, a3, d4, figTitle)
% Simple 3D animation using FK points
    figure("Name", figTitle);
    axis equal; grid on; view(3);
    xlabel("X (mm)"); ylabel("Y (mm)"); zlabel("Z (mm)");
    title(figTitle);
    axis([-500 500 -500 500 0 800]);

    for k = 1:size(theta_traj_deg,1)
        [~, P] = FK_ReBel4DOF(theta_traj_deg(k,:), d1, a2, a3, d4);

        clf; hold on; grid on; view(3);
        axis([-500 500 -500 500 0 800]);
        xlabel("X (mm)"); ylabel("Y (mm)"); zlabel("Z (mm)");
        title(figTitle);

        plot3(P(1,:), P(2,:), P(3,:), "-o", "LineWidth", 2, "MarkerSize", 5);
        drawnow;
    end
end
