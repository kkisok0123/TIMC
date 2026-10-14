function [Des_pL, RL_d, Des_Att] = load_lifting_desireTrajectory(t, dt, varargin)
% Transport starts at t=0 (no takeoff/reset phase).
% Des_Att=[ZYX Euler angles, Euler rates, Euler accelerations].
% Analytic derivatives avoid startup zeros and finite-difference impulses.
% dt retained for interface compatibility; samples evaluated at actual t.
N = numel(t);
Des_pL = zeros(3,3,N); RL_d = zeros(3,3,N); Des_Att = zeros(3,3,N);
w = 2*pi/10;
vx = 1.5*w*cos(w*t); vy = -4.2*w/2*sin(w*t/2);
ax = -1.5*w^2*sin(w*t); ay = -4.2*(w/2)^2*cos(w*t/2);
jx = -1.5*w^3*cos(w*t); jy = 4.2*(w/2)^3*sin(w*t/2);
yaw = unwrap(atan2(vy, vx));
for k = 1:N
    speed2 = vx(k)^2 + vy(k)^2;
    if speed2 < 1e-12
        error('TIMC:StationaryHeading', 'Velocity heading is undefined at a stationary reference.');
    end
    num = vx(k)*ay(k) - vy(k)*ax(k);
    yaw_dot = num/speed2;
    yaw_ddot = (vx(k)*jy(k)-vy(k)*jx(k))/speed2 ...
        - num*2*(vx(k)*ax(k)+vy(k)*ay(k))/speed2^2;
    Des_pL(:,:,k) = [1.5*sin(w*t(k)), vx(k), ax(k); ...
        4.2*cos(w*t(k)/2), vy(k), ay(k); 0.5, 0, 0];
    Des_Att(:,:,k) = [0,0,0; 0,0,0; yaw(k),yaw_dot,yaw_ddot];
    RL_d(:,:,k) = eul2rotm_ZYX(Des_Att(:,1,k));
end
end
