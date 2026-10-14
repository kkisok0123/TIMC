function [Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, theta_rbf, fn, fn_att, h_rbf_all, eso_state] = load_lifting_payload_control_rbf(x_L, Des_Att, Des_pL, RL_d, theta_rbf_prev, dt_override, x_L_prev, with_disturbance, controller_config, eso_state_prev, u_prev)
%% 货物 RISE+RBF+ESO 组合控制器
% 说明：本控制器采用RISE控制作为基础控制器，RBF神经网络用于自适应补偿未知动态和干扰
% 同时集成了ESO（扩展状态观测器）来估计负载状态和总扰动
% 控制律：u = u_rise - RBF_compensation - ESO_disturbance_compensation
% 其中 u_rise 是RISE控制律，RBF_compensation 是RBF网络的自适应补偿项
% ESO_disturbance_compensation 是ESO估计的总扰动补偿项
% 
% 输入参数：
%   x_L: 货物状态向量 (12x1) - 当前时刻 [Att; Ome; Pos; Vel]
%   Des_Att: 期望姿态 (3x3): [Att_L_d, Ome_L_d, Ome_L_d_dot]
%   Des_pL: 期望位置 (3x3): [pL_d, pL_d_dot, pL_2d_dot]
%   RL_d: 期望旋转矩阵 (3x3)
%   theta_rbf_prev: 上一时刻的RBF参数 (30x1) - 6自由度 × 5个RBF节点
%   dt_override: 时间步长（可选；为空时使用主仿真传入的全局 dt）
%   x_L_prev: 货物状态向量 (12x1) - 上一时刻（可选，用于自适应律）
%   with_disturbance: 是否有干扰（保留兼容接口，实际参数从 controller_config 读取）
%   controller_config: 控制器配置结构体（必须，包含基础控制器、RBF、ESO和自适应律参数）
%   eso_state_prev: 上一时刻的ESO状态 (18x1) - 可选，用于ESO更新
%                   格式：[z1_pos(3); z2_pos(3); z3_pos(3); z1_att(3); z2_att(3); z3_att(3)]
%                   其中 z1估计位置/姿态，z2估计速度/角速度，z3估计总扰动
%   u_prev: 上一时刻的控制输入 (6x1) - 可选，用于ESO更新
%           格式：[Fd_prev(3); Md_prev(3)]，其中Fd_prev是上一时刻的力，Md_prev是上一时刻的力矩
% 
% 输出参数：
%   Fd: 期望力 (3x1) - 世界坐标系（RISE控制）
%   Md: 期望力矩 (3x1) - 机体坐标系（RISE控制）
%   e_RL: 旋转矩阵误差 (3x1)
%   e_Ome_L: 物体角速度误差 (3x1)
%   e_pL: 位置误差 (3x1)
%   e_vL: 速度误差 (3x1)
%   theta_rbf: 更新后的RBF参数 (30x1) - 通过自适应律更新
%   fn: 位置控制RBF网络输出 (3x1) - 可用于补偿（当前已注释）
%   fn_att: 姿态控制RBF网络输出 (3x1) - 可用于补偿（当前已注释）
%   h_rbf_all: RBF基函数值 (5x6) - 前3列位置控制，后3列姿态控制
%   eso_state: 当前时刻的ESO状态 (18x1) - [z1_pos; z2_pos; z3_pos; z1_att; z2_att; z3_att]

global dt
model = get_required_config_section(controller_config, {'payload_model'}, 'Payload nominal model');
m_L = model.mass; J_L = model.inertia; g = model.gravity; e3 = [0;0;1];
Jdiag = diag(J_L);
a1_L = (Jdiag(2)-Jdiag(3))/Jdiag(1);
a2_L = (Jdiag(3)-Jdiag(1))/Jdiag(2);
a3_L = (Jdiag(1)-Jdiag(2))/Jdiag(3);
b1_L = 1/Jdiag(1); b2_L = 1/Jdiag(2); b3_L = 1/Jdiag(3);
persistent rise_initialized_rbf I_pos_rbf I_att_rbf
rbf_config = get_required_config_section(controller_config, {'rbf'}, 'Payload RBF controller');
eso_config = get_required_config_section(controller_config, {'eso'}, 'Payload RBF controller');
adaptation_config = get_required_config_section(controller_config, {'rbf', 'adaptation'}, 'Payload RBF controller');
output_limits = get_required_config_section(controller_config, {'rbf', 'output_limits'}, 'Payload RBF controller');

c_rbf = rbf_config.c_rbf;
c_adapt_rbf = rbf_config.c_adapt_rbf;
b_rbf = rbf_config.b_rbf;
theta_max = rbf_config.theta_max;
theta_norm_max = rbf_config.theta_norm_max;
rbf_enable_compensation = logical(rbf_config.enable_compensation);
rbf_compensation_scale = rbf_config.compensation_scale;
if isfield(rbf_config, 'compensation_scale_pos') && ~isempty(rbf_config.compensation_scale_pos)
    rbf_compensation_scale_pos = rbf_config.compensation_scale_pos;
else
    rbf_compensation_scale_pos = rbf_compensation_scale;
end
if isfield(rbf_config, 'compensation_scale_att') && ~isempty(rbf_config.compensation_scale_att)
    rbf_compensation_scale_att = rbf_config.compensation_scale_att;
else
    rbf_compensation_scale_att = rbf_compensation_scale;
end
rbf_enable_adaptation = logical(rbf_config.enable_adaptation);
fn_max = output_limits.fn_max;
fn_att_max = output_limits.fn_att_max;

a_L = [a1_L; a2_L; a3_L];
b_L = [b1_L; b2_L; b3_L];

% 如果未提供 RBF 参数，使用统一配置中的初值
if nargin < 5 || isempty(theta_rbf_prev)
    theta_rbf_prev = rbf_config.theta_rbf_init;
end

if nargin < 6 || isempty(dt_override)
    if isempty(dt)
        error('Payload RBF controller requires dt from the simulation loop or the global simulation state.');
    end
    dt_local = dt;
else
    dt_local = dt_override;
end

%% 提取货物状态
PosL = [x_L(7); x_L(9);  x_L(11)];       % X Y Z
VelL = [x_L(8); x_L(10); x_L(12)];       % X_dot Y_dot Z_dot
AttL = [x_L(1); x_L(3);  x_L(5)];        % PHI（roll） THETA（pitch） PSI（yaw）
OmeL = [x_L(2); x_L(4);  x_L(6)];        % PHI_dot THETA_dot PSI_dot

% ESO状态初始化（在状态提取之后）
if nargin < 10 || isempty(eso_state_prev)
    % 如果未提供ESO状态，使用当前测量值初始化
    eso_state_prev = zeros(18, 1);
    % 使用当前状态初始化ESO
    eso_state_prev(1:3) = PosL;      % z1_pos: 估计位置
    eso_state_prev(4:6) = VelL;      % z2_pos: 估计速度
    eso_state_prev(7:9) = zeros(3,1); % z3_pos: 估计位置总扰动（初始为0）
    eso_state_prev(10:12) = AttL;    % z1_att: 估计姿态
    eso_state_prev(13:15) = OmeL;   % z2_att: 估计角速度
    eso_state_prev(16:18) = zeros(3,1); % z3_att: 估计姿态总扰动（初始为0）
end

% 上一时刻的控制输入初始化（用于ESO更新）
if nargin < 11 || isempty(u_prev)
    u_prev = zeros(6, 1);  % [Fd_prev; Md_prev]
end
Fd_prev = u_prev(1:3);  % 上一时刻的力
Md_prev = u_prev(4:6);  % 上一时刻的力矩

%% 提取期望值
AttL_d = Des_Att(:,1);
OmeL_d = Des_Att(:,2);
OmeL_d_dot = Des_Att(:,3);
% Des_Att contains Euler derivatives; convert for geometric rate errors.
[T, T_dot, eta_dot] = zyx_euler_kinematics(AttL, OmeL);
[Td, ~, ~] = zyx_euler_kinematics(AttL_d, zeros(3,1));
OmeL_d = Td \ Des_Att(:,2);
[~, Td_dot] = zyx_euler_kinematics(AttL_d, OmeL_d);
OmeL_d_dot = Td \ (Des_Att(:,3) - Td_dot * OmeL_d);

PosL_d = Des_pL(:,1);
PosL_d_dot = Des_pL(:,2);
PosL_d_2dot = Des_pL(:,3);

%% 计算当前旋转矩阵和误差
RL = eul2rotm_ZYX(AttL);  % 当前旋转矩阵

% 姿态旋转矩阵误差（用于自适应律）
e_RL = 1/2 * vee(RL_d' * RL - RL' * RL_d);
e_Ome_L = OmeL - RL' * RL_d * OmeL_d;

base_mode = lower(char(rbf_config.base_mode));

%% 从controller_config读取RISE控制参数
if strcmpi(base_mode, 'rise')
    rise_config = get_required_config_section(controller_config, {'payload_control', 'rise'}, 'Payload RBF controller');
    k_s_pos = rise_config.k_s_pos;
    k1_pos = rise_config.k1_pos;
    k2_pos = rise_config.k2_pos;
    Imax_pos = rise_config.Imax_pos;
    eps_s_pos = rise_config.eps_s_pos;
    k_s_att = rise_config.k_s_att;
    k1_att = rise_config.k1_att;
    k2_att = rise_config.k2_att;
    Imax_att = rise_config.Imax_att;
    eps_s_att = rise_config.eps_s_att;
end

if strcmpi(base_mode, 'rise')
    if isscalar(k_s_pos), k_s_pos = k_s_pos * ones(3,1); end
    if isscalar(k1_pos), k1_pos = k1_pos * ones(3,1); end
    if isscalar(k2_pos), k2_pos = k2_pos * ones(3,1); end
    if isscalar(k_s_att), k_s_att = k_s_att * ones(3,1); end
    if isscalar(k1_att), k1_att = k1_att * ones(3,1); end
    if isscalar(k2_att), k2_att = k2_att * ones(3,1); end
end

%% 计算位置和姿态误差
e_pL = PosL - PosL_d;  % 位置误差（用于RBF输入和自适应律）
e_vL = VelL - PosL_d_dot;  % 速度误差（用于自适应律）
e_Att = AttL - AttL_d;  % 姿态误差（用于RBF输入）
e_Att_dot = eta_dot - Des_Att(:,2);  % Euler derivative error, not body-rate error

%% 基础控制器（RISE / Backstepping）
s_pos = zeros(3, 1);
s_att = zeros(3, 1);
I_pos_rbf = zeros(3, 1);
I_att_rbf = zeros(3, 1);
Fd_nominal = zeros(3, 1);
Md_nominal = zeros(3, 1);

switch lower(base_mode)
    case 'backstepping'
        [Fd_nominal, Md_nominal, e_RL, e_Ome_L, e_pL, e_vL] = ...
            load_lifting_payload_control_backstepping(x_L, Des_Att, Des_pL, RL_d, controller_config);

    otherwise
        if isempty(rise_initialized_rbf)
            I_pos_rbf = zeros(3, 1);
            I_att_rbf = zeros(3, 1);
            rise_initialized_rbf = true;
        end

        sat_pos = @(s) tanh(s / eps_s_pos);
        sat_att = @(s) tanh(s / eps_s_att);

        s_pos = e_vL + diag(k_s_pos) * e_pL;
        for i = 1:3
            I_pos_rbf(i) = I_pos_rbf(i) + dt_local * sat_pos(s_pos(i));
            I_pos_rbf(i) = min(max(I_pos_rbf(i), -Imax_pos), Imax_pos);
        end

        s_att = e_Ome_L + diag(k_s_att) * e_RL;
        for i = 1:3
            I_att_rbf(i) = I_att_rbf(i) + dt_local * sat_att(s_att(i));
            I_att_rbf(i) = min(max(I_att_rbf(i), -Imax_att), Imax_att);
        end
end

%% ESO扩展状态观测器更新
% ESO用于估计负载状态和总扰动（包括未知动态和外部干扰）
% 注意：
%   - ESO会始终进行估计（所有方向），估计数据会保存用于分析和绘图
%   - 可以通过use_eso_compensation控制是否使用ESO补偿
%   - 当前设置：ESO补偿仅应用于Z方向（垂直方向），X和Y方向不使用ESO补偿
% 从controller_config读取ESO参数
beta_pos = eso_config.beta_pos;  % [beta1, beta2, beta3] for position (3x3)
b_pos = eso_config.b_pos;        % 控制输入系数 (3x1)
beta_att = eso_config.beta_att;  % [beta1, beta2, beta3] for attitude (3x3)
b_att = eso_config.b_att;        % 控制输入系数 (3x1)
use_eso_compensation = eso_config.use_compensation;  % 是否使用ESO补偿（true=使用补偿，false=仅估计）
eso_compensation_scale = eso_config.compensation_scale;
if isfield(eso_config, 'compensation_scale_pos') && ~isempty(eso_config.compensation_scale_pos)
    eso_compensation_scale_pos = eso_config.compensation_scale_pos;
else
    eso_compensation_scale_pos = eso_compensation_scale;
end
if isfield(eso_config, 'compensation_scale_att') && ~isempty(eso_config.compensation_scale_att)
    eso_compensation_scale_att = eso_config.compensation_scale_att;
else
    eso_compensation_scale_att = eso_compensation_scale;
end

% 提取上一时刻的ESO状态
z1_pos_prev = eso_state_prev(1:3);   % 上一时刻估计的位置
z2_pos_prev = eso_state_prev(4:6);   % 上一时刻估计的速度
z3_pos_prev = eso_state_prev(7:9);   % 上一时刻估计的位置总扰动
z1_att_prev = eso_state_prev(10:12); % 上一时刻估计的姿态
z2_att_prev = eso_state_prev(13:15); % 上一时刻估计的角速度
z3_att_prev = eso_state_prev(16:18); % 上一时刻估计的姿态总扰动

% 初始化当前ESO状态
eso_state = zeros(18, 1);
z1_pos = zeros(3, 1);
z2_pos = zeros(3, 1);
z3_pos = zeros(3, 1);
z1_att = zeros(3, 1);
z2_att = zeros(3, 1);
z3_att = zeros(3, 1);

% 位置ESO更新（3个独立的ESO，分别对应X、Y、Z方向）
% 对于位置系统：p_dot = v, v_dot = f/m + d，其中d是总扰动
% ESO设计：z1估计p，z2估计v，z3估计d
% 注意：ESO模型假设控制输入f是"纯控制力"（不包括重力补偿）
% 因此需要从Fd_prev中减去重力补偿项，让ESO正确估计包括重力在内的总扰动
for i = 1:3
    % 位置测量值
    y_pos = PosL(i);
    
    % ESO观测误差
    e_eso_pos = y_pos - z1_pos_prev(i);
    
    % ESO更新（使用欧拉法）
    % z1_dot = z2 + beta1*e
    % z2_dot = z3 + b*u + beta2*e
    % z3_dot = beta3*e
    % 使用上一时刻的控制输入（力）进行更新
    % 重要：从控制输入中减去重力补偿项，因为ESO模型假设控制输入不包括重力
    % 这样ESO可以将重力作为扰动的一部分进行估计
    if i == 3  % Z方向需要减去重力补偿
        Fd_prev_clean = Fd_prev(i) - m_L * g;  % 减去重力补偿项
    else
        Fd_prev_clean = Fd_prev(i);  % X和Y方向没有重力补偿
    end
    u_pos = Fd_prev_clean / m_L;  % 将力转换为加速度（使用"纯控制力"）
    
    z1_pos(i) = z1_pos_prev(i) + dt_local * (z2_pos_prev(i) + beta_pos(i,1) * e_eso_pos);
    z2_pos(i) = z2_pos_prev(i) + dt_local * (z3_pos_prev(i) + b_pos(i) * u_pos + beta_pos(i,2) * e_eso_pos);
    z3_pos(i) = z3_pos_prev(i) + dt_local * (beta_pos(i,3) * e_eso_pos);
end

% 姿态ESO更新（3个独立的ESO，分别对应phi、theta、psi）
% 对于姿态系统：att_dot = omega, omega_dot = f/J + d，其中d是总扰动
% ESO设计：z1估计att，z2估计omega，z3估计d
for i = 1:3
    % 姿态测量值
    y_att = AttL(i);
    
    % ESO观测误差
    e_eso_att = y_att - z1_att_prev(i);
    
    % ESO更新（使用欧拉法）
    % 使用上一时刻的控制输入（力矩）进行更新
    u_att = Md_prev(i);  % 上一时刻的力矩
    
    z1_att(i) = z1_att_prev(i) + dt_local * (z2_att_prev(i) + beta_att(i,1) * e_eso_att);
    z2_att(i) = z2_att_prev(i) + dt_local * (z3_att_prev(i) + b_att(i) * u_att + beta_att(i,2) * e_eso_att);
    z3_att(i) = z3_att_prev(i) + dt_local * (beta_att(i,3) * e_eso_att);
end

% 保存当前ESO状态
eso_state(1:3) = z1_pos;
eso_state(4:6) = z2_pos;
eso_state(7:9) = z3_pos;
eso_state(10:12) = z1_att;
eso_state(13:15) = z2_att;
eso_state(16:18) = z3_att;

% ESO估计的总扰动（用于补偿）
d_pos_eso = z3_pos;  % 位置总扰动估计
d_att_eso = z3_att;  % 姿态总扰动估计

%% RBF网络输入和自适应律参数
% 组合误差向量（用于RBF网络输入）
xi = [e_pL; e_Att];  % 6维：3个位置误差 + 3个姿态误差

% 自适应律中的系统矩阵参数（用于构建Lyapunov方程）
kp_pos_xy = adaptation_config.kp_pos_xy;  % X和Y方向位置比例增益
kv_pos_xy = adaptation_config.kv_pos_xy;  % X和Y方向速度微分增益
kp_pos_z = adaptation_config.kp_pos_z;    % Z方向位置比例增益
kv_pos_z = adaptation_config.kv_pos_z;    % Z方向速度微分增益
kp_att = adaptation_config.kp_att;        % 姿态比例增益
kv_att = adaptation_config.kv_att;        % 姿态微分增益

%% 位置控制（RISE控制 + RBF补偿）
% 计算位置控制RBF网络输出
fn = zeros(3, 1);  % 位置控制RBF网络输出
h_rbf_pos = zeros(5, 3);  % 位置控制的h_rbf (5个节点 x 3个方向)
for i = 1:3
    % 每个方向的RBF网络（索引：1-15对应位置控制）
    idx = (i-1)*5 + 1:i*5;
    theta_i = theta_rbf_prev(idx);
    theta_i = theta_i(:);  % 确保是列向量
    
    % 计算RBF基函数（添加数值稳定性保护）
    h_rbf = zeros(5, 1);
    for j = 1:5
        dist_sq = norm(xi - c_rbf(:, j))^2;
        exp_arg = -dist_sq/(2*b_rbf(j)*b_rbf(j));
        % 限制指数参数，防止exp溢出
        exp_arg = max(exp_arg, -50);  % exp(-50) ≈ 0，防止数值溢出
        h_rbf(j) = exp(exp_arg);
    end
    
    % 保存h_rbf（位置控制）
    h_rbf_pos(:, i) = h_rbf;
    
    % RBF网络输出（归一化基函数，提高数值稳定性）
    h_rbf_sum = sum(h_rbf);
    if h_rbf_sum > 1e-10
        h_rbf_norm = h_rbf / h_rbf_sum;  % 归一化
    else
        h_rbf_norm = h_rbf;  % 如果和太小，不归一化
    end
    
    % 限制theta_i的值，防止fn过大
    theta_i_clamped = max(min(theta_i, theta_max), -theta_max);
    fn(i) = dot(theta_i_clamped, h_rbf_norm);
    
    fn(i) = max(min(fn(i), fn_max), -fn_max);
end

% RISE控制力计算
if ~rbf_enable_compensation
    fn = zeros(3, 1);
end
fn = rbf_compensation_scale_pos * fn;

if use_eso_compensation
    Fd_eso_comp = eso_compensation_scale_pos * m_L * d_pos_eso;
else
    Fd_eso_comp = zeros(3, 1);
end
if strcmpi(base_mode, 'backstepping')
    Fd = Fd_nominal - m_L * fn - Fd_eso_comp;
else
    Fd_nominal = m_L * (PosL_d_2dot - diag(k1_pos) * s_pos - diag(k2_pos) * I_pos_rbf + g * e3);
    Fd = Fd_nominal - m_L * fn - Fd_eso_comp;
end

%% 姿态控制（RISE控制 + RBF补偿）
tmp = RL' * RL_d * OmeL_d;
tmp_hat = skew(tmp);
OmeL_cross = [OmeL(2)*OmeL(3); OmeL(1)*OmeL(3); OmeL(1)*OmeL(2)];  % 角速度交叉乘积项
coriolis_term = diag(a_L) * OmeL_cross;  % 科里奥利项
b_L_inv = diag(1./b_L);  % 惯性矩阵的逆

% 计算每个轴的RBF网络输出（用于自适应补偿，当前已注释掉补偿项）
fn_att = zeros(3, 1);  % 姿态控制RBF网络输出
h_rbf_att = zeros(5, 3);  % 姿态控制的RBF基函数值 (5个节点 x 3个方向)
for i = 1:3
    % 每个轴的RBF网络参数索引（索引：16-30对应姿态控制）
    % 位置控制：1-15 (3×5), 姿态控制：16-30 (3×5)
    idx = (i+2)*5 + 1:(i+3)*5;
    theta_i = theta_rbf_prev(idx);
    theta_i = theta_i(:);  % 确保是列向量
    
    % 计算RBF基函数（添加数值稳定性保护）
    h_rbf = zeros(5, 1);
    for j = 1:5
        dist_sq = norm(xi - c_rbf(:, j))^2;
        exp_arg = -dist_sq/(2*b_rbf(j)*b_rbf(j));
        % 限制指数参数，防止exp溢出
        exp_arg = max(exp_arg, -50);  % exp(-50) ≈ 0，防止数值溢出
        h_rbf(j) = exp(exp_arg);
    end
    
    % 保存h_rbf（姿态控制）
    h_rbf_att(:, i) = h_rbf;
    
    % RBF网络输出（归一化基函数，提高数值稳定性）
    h_rbf_sum = sum(h_rbf);
    if h_rbf_sum > 1e-10
        h_rbf_norm = h_rbf / h_rbf_sum;  % 归一化
    else
        h_rbf_norm = h_rbf;  % 如果和太小，不归一化
    end
    
    % 限制theta_i的值，防止fn_att过大
    theta_i_clamped = max(min(theta_i, theta_max), -theta_max);
    fn_att(i) = dot(theta_i_clamped, h_rbf_norm);
    
    fn_att(i) = max(min(fn_att(i), fn_att_max), -fn_att_max);
end

if ~rbf_enable_compensation
    fn_att = zeros(3, 1);
end
fn_att = rbf_compensation_scale_att * fn_att;

if use_eso_compensation
    Md_eso_comp = eso_compensation_scale_att * d_att_eso;
else
    Md_eso_comp = zeros(3, 1);
end
if strcmpi(base_mode, 'backstepping')
    % Compensation coordinates follow the Euler-acceleration nominal channel.
    Md = Md_nominal - J_L * (T \ (fn_att + Md_eso_comp));
else
    Md_nominal = b_L_inv * (J_L * RL' * RL_d * OmeL_d_dot + tmp_hat * J_L * tmp - diag(k1_att) * s_att - diag(k2_att) * I_att_rbf - coriolis_term);
    Md = Md_nominal - b_L_inv * fn_att - b_L_inv * Md_eso_comp;
end
%% RBF参数自适应更新律
% 使用Lyapunov稳定性理论设计自适应律，确保闭环系统稳定
theta_rbf = theta_rbf_prev;  % 默认保持上一步参数
gama = adaptation_config.gama;  % 自适应增益（控制参数更新速度）
gama_z = adaptation_config.gama_z;  % Z方向自适应增益
k1 = adaptation_config.k1;  % 位置修正项增益（防止参数发散，增强参数有界性）
k1_att = k1;
sigma = adaptation_config.sigma;  % 死区参数（当误差很小时减小自适应更新，避免参数抖动）
sigma_z = adaptation_config.sigma_z;  % Z方向死区参数
B_i = adaptation_config.B_i;
Q_i = adaptation_config.Q_i;
gama_att = gama;
sigma_att = sigma;
Q_att = Q_i;
if isfield(adaptation_config, 'gama_att') && ~isempty(adaptation_config.gama_att)
    gama_att = adaptation_config.gama_att;
end
if isfield(adaptation_config, 'sigma_att') && ~isempty(adaptation_config.sigma_att)
    sigma_att = adaptation_config.sigma_att;
end
if isfield(adaptation_config, 'k1_att') && ~isempty(adaptation_config.k1_att)
    k1_att = adaptation_config.k1_att;
end
if isfield(adaptation_config, 'Q_att') && ~isempty(adaptation_config.Q_att)
    Q_att = adaptation_config.Q_att;
end

if ~rbf_enable_adaptation
    h_rbf_all = [h_rbf_pos, h_rbf_att];
    return;
end

for i = 1:6
    idx = (i-1)*5 + 1:i*5;
    theta_i = theta_rbf_prev(idx);
    
    if i <= 3  % 位置控制（i=1,2,3 对应 X, Y, Z）
        % 构建误差状态向量（用于自适应律）
        xi_i = [e_pL(i); e_vL(i)];
        
        % 死区机制：当误差很小时，减小自适应更新（避免参数抖动）
        % Z方向使用更大的自适应增益以加快学习
        if i == 3  % Z方向
            error_norm = norm(xi_i);
            if error_norm < sigma_z
                gama_eff = gama_z * (error_norm / sigma_z);  % 线性减小
            else
                gama_eff = gama_z;
            end
        else  % X和Y方向
            error_norm = norm(xi_i);
            if error_norm < sigma
                gama_eff = gama * (error_norm / sigma);  % 线性减小
            else
                gama_eff = gama;
            end
        end
        
        % 构建系统矩阵（用于求解Lyapunov方程）
        if i == 3  % Z方向
            A_i = [0, 1; -kp_pos_z, -kv_pos_z];
        else  % X和Y方向
            A_i = [0, 1; -kp_pos_xy, -kv_pos_xy];
        end
        P_i = lyap(A_i', Q_i);  % 求解Lyapunov方程：A_i'*P + P*A_i = -Q_i
        
        % 计算RBF基函数（用于自适应律）
        h_rbf_adapt = zeros(5, 1);
        for j = 1:5
            h_rbf_adapt(j) = exp(-norm(xi_i - c_adapt_rbf(:, j))^2/(2*b_rbf(j)*b_rbf(j)));
        end
        
        % 自适应律：theta_dot = gamma * h_rbf * xi' * P * B - k1 * gamma * theta
        % 第一项：基于误差的自适应更新；第二项：修正项（防止参数发散）
        S = gama_eff*h_rbf_adapt*xi_i'*P_i*B_i - k1*gama_eff*theta_i;
        theta_rbf(idx) = theta_rbf_prev(idx) + dt_local * S;
        
    else  % 姿态控制 (i=4,5,6 对应 phi, theta, psi)
        % 构建误差状态向量（用于自适应律）
        att_idx = i - 3;  % 1, 2, 3 对应 phi, theta, psi
        xi_i = [e_Att(att_idx); e_Att_dot(att_idx)];
        
        % 死区机制
        error_norm = norm(xi_i);
        if error_norm < sigma_att
            gama_eff = gama_att * (error_norm / sigma_att);
        else
            gama_eff = gama_att;
        end
        
        % 构建系统矩阵（用于求解Lyapunov方程）
        A_i = [0, 1; -kp_att, -kv_att];
        P_i = lyap(A_i', Q_att);  % 求解Lyapunov方程
        
        % 计算RBF基函数（用于自适应律）
        h_rbf_adapt = zeros(5, 1);
        for j = 1:5
            h_rbf_adapt(j) = exp(-norm(xi_i - c_adapt_rbf(:, j))^2/(2*b_rbf(j)*b_rbf(j)));
        end
        
        % 自适应律
        S = gama_eff*h_rbf_adapt*xi_i'*P_i*B_i - k1_att*gama_eff*theta_i;
        theta_rbf(idx) = theta_rbf_prev(idx) + dt_local * S;
    end
    
    % 参数投影：限制每个参数的绝对值（防止参数过大）
    for k = 1:length(idx)
        if abs(theta_rbf(idx(k))) > theta_max
            theta_rbf(idx(k)) = sign(theta_rbf(idx(k))) * theta_max;
        end
    end
    
    % 参数投影：限制每个自由度参数向量的范数（防止参数向量过大）
    theta_norm = norm(theta_rbf(idx));
    if theta_norm > theta_norm_max
        theta_rbf(idx) = theta_rbf(idx) * (theta_norm_max / theta_norm);
    end
end

% 合并所有RBF基函数值：h_rbf_all (5 x 6)
% 前3列是位置控制（X, Y, Z），后3列是姿态控制（phi, theta, psi）
h_rbf_all = [h_rbf_pos, h_rbf_att];

end
