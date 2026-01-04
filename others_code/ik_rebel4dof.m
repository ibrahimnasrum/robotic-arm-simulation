function theta_deg = ik_rebel4dof(x, y, z, desired_orientation_deg, d1, a2, a3, d4)
%IK_REBEL4DOF Inverse kinematics for simplified 4-DOF arm (degrees output)
% Uses classic 2-link planar IK after projecting to r-z plane.
% θ1: base yaw
% θ2, θ3: shoulder/elbow
% θ4: wrist to satisfy desired overall orientation (simplified)

    % Base
    theta1 = atan2(y, x);  % rad

    % Project into r-z plane
    r = sqrt(x^2 + y^2);
    s = (z - d1);  % vertical from shoulder level

    % Effective wrist offset (optional). Here we keep simplified model.
    % If you want to include d4 properly, you need orientation-dependent wrist center.
    % For now: keep consistent with your original approach.

    % Law of cosines for elbow
    D = (r^2 + s^2 - a2^2 - a3^2) / (2*a2*a3);
    if abs(D) > 1
        error("Target out of reach (|D|>1). Check waypoint or link lengths.");
    end

    % Elbow-up solution
    theta3 = atan2(sqrt(1 - D^2), D);

    % Shoulder
    phi = atan2(s, r);
    psi = atan2(a3*sin(theta3), a2 + a3*cos(theta3));
    theta2 = phi - psi;

    % Wrist orientation (simplified)
    desired_orientation = deg2rad(desired_orientation_deg);
    theta4 = desired_orientation - (theta1 + theta2 + theta3);

    % Convert to degrees
    theta_deg = [rad2deg(theta1), rad2deg(theta2), rad2deg(theta3), rad2deg(theta4)];
end
