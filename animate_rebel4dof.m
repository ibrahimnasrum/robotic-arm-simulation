function animate_rebel4dof(theta_traj_deg, d1, a2, a3, d4, figTitle)
%ANIMATE_REBEL4DOF Simple 3D animation using FK joint points

    figure('Name',figTitle);
    axis equal; grid on; view(3);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title(figTitle);
    axis([-500 500 -500 500 0 800]);

    for k = 1:size(theta_traj_deg,1)
        [~, P] = fk_rebel4dof(theta_traj_deg(k,:), d1, a2, a3, d4);

        clf; hold on; grid on; view(3);
        axis([-500 500 -500 500 0 800]);
        xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
        title(figTitle);

        plot3(P(1,:), P(2,:), P(3,:), '-o', 'LineWidth', 2, 'MarkerSize', 5);
        drawnow;
    end
end
