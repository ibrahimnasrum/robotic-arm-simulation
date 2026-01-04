clc; clear; close all;

% =========================
% Robot parameters (mm)
% =========================
d1 = 252;
a2 = 237;
a3 = 244;
d4 = 142;   % gripper offset (mm)

% Desired wrist/orientation (deg) - keep 0 if you want "horizontal"
desired_orientation = 0;

% =========================
% Waypoints (mm)
% =========================
WP1   = [-200,  20, 370.5];   % Pickup 1
WP2   = [-200,  50, 370.5];   % Pickup 2
Rack2 = [-180,  70, 550];     % Place 1
Rack16= [ 180,  70, 450];     % Place 2

% Two tasks:
% Task 1: WP1 -> Rack2
% Task 2: WP2 -> Rack16
tasks = {
    "Task 1 (WP1 -> Rack2)", [WP1; Rack2];
    "Task 2 (WP2 -> Rack16)", [WP2; Rack16];
};

% =========================
% Compute IK for each target
% =========================
fprintf("=== Inverse Kinematics Results (deg) ===\n");
for i = 1:size(tasks,1)
    name = tasks{i,1};
    pts  = tasks{i,2};

    th_start = ik_rebel4dof(pts(1,1), pts(1,2), pts(1,3), desired_orientation, d1, a2, a3, d4);
    th_end   = ik_rebel4dof(pts(2,1), pts(2,2), pts(2,3), desired_orientation, d1, a2, a3, d4);

    fprintf("%s\n", name);
    fprintf("  Start: θ1=%.2f°, θ2=%.2f°, θ3=%.2f°, θ4=%.2f°\n", th_start(1), th_start(2), th_start(3), th_start(4));
    fprintf("  End  : θ1=%.2f°, θ2=%.2f°, θ3=%.2f°, θ4=%.2f°\n\n", th_end(1), th_end(2), th_end(3), th_end(4));
end

% =========================
% Trajectory planning (pchip)
% =========================
t_total  = 6;     % seconds
n_points = 100;

% Build joint waypoints for both tasks
th_task1_start = ik_rebel4dof(WP1(1),WP1(2),WP1(3),desired_orientation,d1,a2,a3,d4);
th_task1_end   = ik_rebel4dof(Rack2(1),Rack2(2),Rack2(3),desired_orientation,d1,a2,a3,d4);

th_task2_start = ik_rebel4dof(WP2(1),WP2(2),WP2(3),desired_orientation,d1,a2,a3,d4);
th_task2_end   = ik_rebel4dof(Rack16(1),Rack16(2),Rack16(3),desired_orientation,d1,a2,a3,d4);

time_steps = linspace(0, t_total, n_points);

traj1 = plan_trajectory([th_task1_start; th_task1_end], time_steps);
traj2 = plan_trajectory([th_task2_start; th_task2_end], time_steps);

% =========================
% Plot theta vs time
% =========================
figure('Name','Joint Trajectories (Theta vs Time)');
for j = 1:4
    subplot(4,1,j);
    plot(time_steps, traj1(:,j), 'LineWidth', 1.5); hold on;
    plot(time_steps, traj2(:,j), '--', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('\\theta_%d (deg)', j));
    title(sprintf('Joint %d Trajectory', j));
    legend('Task 1','Task 2','Location','best');
end

% =========================
% Animate (optional)
% =========================
animate_rebel4dof(traj1, d1, a2, a3, d4, "Task 1 Animation");
animate_rebel4dof(traj2, d1, a2, a3, d4, "Task 2 Animation");
