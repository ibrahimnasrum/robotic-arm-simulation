function theta_traj = plan_trajectory(joint_waypoints_deg, time_steps)
%PLAN_TRAJECTORY Interpolate joint waypoints using pchip
% joint_waypoints_deg: [N x 4] (deg)
% time_steps: 1 x M

    t_total = time_steps(end);
    N = size(joint_waypoints_deg,1);
    t_wp = linspace(0, t_total, N);

    theta_traj = zeros(length(time_steps), 4);
    for j = 1:4
        theta_traj(:,j) = interp1(t_wp, joint_waypoints_deg(:,j), time_steps, 'pchip');
    end
end
