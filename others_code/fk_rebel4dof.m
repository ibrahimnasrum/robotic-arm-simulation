function [T0_4, P] = fk_rebel4dof(theta_deg, d1, a2, a3, d4)
%FK_REBEL4DOF Forward kinematics using your DH-style matrices (degrees input)
% Returns:
%   T0_4 : 4x4 end-effector transform
%   P    : 3x5 points [base, joint1, joint2, joint3, end] for plotting

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
