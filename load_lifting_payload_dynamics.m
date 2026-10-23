function [X_dot] = load_lifting_payload_dynamics(x, F_in, M_in, t, with_disturbance)
% 货物动力学函数
% 输入参数：
%   x: 状态向量 (12x1) [roll, wx, pitch, wy, yaw, wz, x, vx, y, vy, z, vz]
%   F_in: 输入力 (3x1) - 世界坐标系
%   M_in: 输入力矩 (3x1) - 机体坐标系
%   t: 当前时间 (标量)
%   with_disturbance: 是否启用加速度扰动 (逻辑值，可选，默认true)
% 输出参数：
%   X_dot: 状态导数 (12x1)

global g m_L Jx_L Jy_L Jz_L
global payload_drag_cd
% Physical plant parameters only; controllers use controller_config.payload_model.
% payload_drag_cd: 未建模的二次气动阻力系数（N/(m/s)^2），控制器不知道它的存在。
if isempty(payload_drag_cd)
    payload_drag_cd = 0;
end

% 默认添加干扰
if nargin < 5
    with_disturbance = true;
end

X_dot = zeros(12,1);

% 计算干扰项
if with_disturbance
    d_R = 1 * sin(t) + 0.2*sin(2*t) + 0.3*sin(3*t) + 0.4*sin(4*t);  % 姿态干扰
    d_p = 3 * sin(t) + 0.2*sin(2*t) + 0.3*sin(3*t) + 0.4*sin(4*t);  % 位置干扰
else
    d_R = 0;
    d_p = 0;
end

% ZYX kinematics and rigid-body dynamics. d_R is BODY angular acceleration
% (rad/s^2), not a moment; equivalent disturbance moment is J*d_R.
J = diag([Jx_L, Jy_L, Jz_L]);
omega = x([2,4,6]);
[~, ~, eta_dot] = zyx_euler_kinematics(x([1,3,5]), omega);
X_dot([1,3,5]) = eta_dot;
X_dot([2,4,6]) = J \ (M_in - cross(omega, J * omega)) + d_R * ones(3,1);
% d_p below is world acceleration (m/s^2), equivalent force m_L*d_p.

% 位置动力学（线速度）
% 未建模的二次气动阻力：F_drag = -cd * |v| * v（世界系，与速度反平行）
v_world = x([8; 10; 12]);
a_drag = -payload_drag_cd * norm(v_world) * v_world / m_L;
X_dot(7) = x(8);
X_dot(8) = F_in(1)/m_L + d_p + a_drag(1);  % 添加位置干扰
X_dot(9) = x(10);
X_dot(10) = F_in(2)/m_L + d_p + a_drag(2);  % 添加位置干扰
X_dot(11) = x(12);
% 向上为正，重力向下，所以是-g
X_dot(12) = F_in(3)/m_L - g + d_p + a_drag(3);  % 添加位置干扰
end 

