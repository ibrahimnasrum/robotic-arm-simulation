clc; clear;

d1=252; a2=237; a3=244; d4=142;

% Example: Rack16 angles from your output (deg)
theta = [21.25, -10.38, 109.81, -120.68];

[T0_4, ~] = fk_rebel4dof(theta, d1, a2, a3, d4);
disp(T0_4);

fprintf("End-effector position (mm): X=%.3f, Y=%.3f, Z=%.3f\n", ...
    T0_4(1,4), T0_4(2,4), T0_4(3,4));
