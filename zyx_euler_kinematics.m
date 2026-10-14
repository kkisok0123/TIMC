function [T, T_dot, eta_dot] = zyx_euler_kinematics(eta, omega)
% ZYX: eta=[roll;pitch;yaw], omega is BODY angular velocity (rad/s).
% Valid only away from pitch singularities; fail rather than silently clamp.
phi = eta(1); theta = eta(2);
if abs(cos(theta)) < 1e-6
    error('TIMC:EulerSingularity', 'ZYX pitch is too close to pi/2 + k*pi.');
end
s = sin(phi); c = cos(phi); u = tan(theta); v = 1/cos(theta);
T = [1, s*u, c*u; 0, c, -s; 0, s*v, c*v];
eta_dot = T * omega;
T_phi = [0, c*u, -s*u; 0, -s, -c; 0, c*v, -s*v];
T_theta = [0, s*v^2, c*v^2; 0, 0, 0; 0, s*v*u, c*v*u];
T_dot = T_phi * eta_dot(1) + T_theta * eta_dot(2);
end
