function [Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, theta_rbf, fn, fn_att, h_rbf_all, eso_state] = load_lifting_payload_control_rbf(x_L, Des_Att, Des_pL, RL_d, theta_rbf_prev, dt_override, x_L_prev, with_disturbance, controller_config, eso_state_prev, u_prev)
%% 负载层 Backstepping + RBF + ESO 复合控制器（返修版）
%
% 结构约定（与返修方案第 3 节一致）：
%   1) 名义控制统一由 load_lifting_payload_control_backstepping 给出，
%      复合控制器只在名义控制上叠加补偿项；
%   2) RBF 每个通道使用二维输入 [本通道误差; 本通道误差导数]，输出与自适应
%      更新共用同一套归一化基函数（6 通道 x 5 节点 = 30 个权重），
%      不再维护两套网络，也不使用误差相关的可变学习率；
%   3) ESO 以约化模型实际收到的合力/合力矩 Fin、Min 为输入，预测模型显式
%      包含重力、RBF 补偿与姿态欧拉角加速度映射 T(eta)*J0^{-1}(...)；
%   4) ESO 估计的是"RBF 补偿之后"的剩余项，避免与 RBF 重复计数；
%   5) 只保留一种理论上有依据的参数保护方式：闭球投影（范数投影）。
%
% 输入/输出接口与旧版保持一致，便于批处理与日志复用。
%
% 输入参数：
%   x_L: 货物状态 [roll,p,pitch,q,yaw,r,x,vx,y,vy,z,vz]
%   Des_Att: 3x3 [期望欧拉角, 期望欧拉角速率, 期望欧拉角加速度]
%   Des_pL: 3x3 [期望位置, 期望速度, 期望加速度]
%   RL_d: 期望旋转矩阵
%   theta_rbf_prev: 上一时刻 RBF 权重 (30x1)
%   dt_override: 时间步长（可选）
%   x_L_prev: 上一时刻状态（保留接口，当前不使用）
%   with_disturbance: 保留接口（实际配置从 controller_config 读取）
%   controller_config: 统一参数配置
%   eso_state_prev: 上一时刻 ESO 状态 (18x1)
%                   [z1_pos; z2_pos; z3_pos; z1_att; z2_att; z3_att]
%   u_prev: 上一执行区间实际施加的输入 (6x1) = [Fin_prev; Min_prev]

global dt

model = get_required_config_section(controller_config, {'payload_model'}, 'Payload nominal model');
m0 = model.mass;
J0 = model.inertia;
g0 = model.gravity;
e3 = [0; 0; 1];

rbf_config = get_required_config_section(controller_config, {'rbf'}, 'Payload RBF controller');
eso_config = get_required_config_section(controller_config, {'eso'}, 'Payload ESO');
adapt_config = get_required_config_section(controller_config, {'rbf', 'adaptation'}, 'Payload RBF adaptation');
bs_config = get_required_config_section(controller_config, {'payload_control', 'backstepping'}, 'Payload backstepping');

base_mode = lower(char(rbf_config.base_mode));
if ~strcmpi(base_mode, 'backstepping')
    error('TIMC:UnsupportedBaseMode', ...
        'Revision RBF controller supports base_mode=''backstepping'' only (got "%s").', base_mode);
end

if nargin < 6 || isempty(dt_override)
    if isempty(dt)
        error('TIMC:MissingDt', 'Payload RBF controller requires dt from the simulation loop or the global simulation state.');
    end
    dt_local = dt;
else
    dt_local = dt_override;
end

%% 状态提取
PosL = x_L([7, 9, 11]);
VelL = x_L([8, 10, 12]);
eta = x_L([1, 3, 5]);
omega = x_L([2, 4, 6]);
[T, T_dot, eta_dot] = zyx_euler_kinematics(eta, omega);

%% 名义反步控制（唯一的名义控制实现）
[Fd_nom, Md_nom, e_RL, e_Ome_L, e_pL, e_vL] = ...
    load_lifting_payload_control_backstepping(x_L, Des_Att, Des_pL, RL_d, controller_config);

e_Att = atan2(sin(eta - Des_Att(:,1)), cos(eta - Des_Att(:,1)));
e_Att_dot = eta_dot - Des_Att(:,2);

%% RBF：每通道二维输入，单套归一化基函数，供输出与更新共用
if nargin < 5 || isempty(theta_rbf_prev)
    theta_rbf_prev = rbf_config.theta_rbf_init(:);
end
theta_mat = reshape(theta_rbf_prev(:), 5, 6);

centers = rbf_config.c_rbf;          % 2 x 5（归一化单位）
widths = rbf_config.b_rbf(:)';       % 1 x 5（归一化单位）
scale_pos = rbf_config.center_scale_pos(:);
scale_att = rbf_config.center_scale_att(:);

% 每个通道的输入：[误差; 误差导数]，按各通道特征尺度归一化
xi = zeros(2, 6);
xi(:, 1:3) = [e_pL.'; e_vL.'];
xi(:, 4:6) = [e_Att.'; e_Att_dot.'];
scales = [repmat(scale_pos, 1, 3), repmat(scale_att, 1, 3)];

H = zeros(5, 6);
for ch = 1:6
    xs = xi(:, ch) ./ max(scales(:, ch), 1e-12);
    dist_sq = sum((centers - xs) .^ 2, 1);
    h = exp(-dist_sq ./ (2 * widths .^ 2));
    H(:, ch) = h / max(sum(h), 1e-12);   % 归一化基函数
end

rbf_raw = sum(H .* theta_mat, 1).';   % 6x1 = [3 位置; 3 姿态]
fn = rbf_raw(1:3);
fn_att = rbf_raw(4:6);
rbf_scale = rbf_config.compensation_scale;
if ~logical(rbf_config.enable_compensation)
    fn = zeros(3, 1);
    fn_att = zeros(3, 1);
end
fn_app = rbf_scale * fn;
fn_att_app = rbf_scale * fn_att;

%% ESO：输入为约化模型实际收到的 Fin、Min
if nargin < 10 || isempty(eso_state_prev)
    eso_state_prev = zeros(18, 1);
    eso_state_prev(1:3) = PosL;
    eso_state_prev(4:6) = VelL;
    eso_state_prev(10:12) = eta;
    eso_state_prev(13:15) = eta_dot;
end
if nargin < 11 || isempty(u_prev)
    u_prev = zeros(6, 1);
end
Fin_prev = u_prev(1:3);
Min_prev = u_prev(4:6);

beta_pos = eso_config.beta_pos;
beta_att = eso_config.beta_att;
z1p = eso_state_prev(1:3);
z2p = eso_state_prev(4:6);
z3p = eso_state_prev(7:9);
z1a = eso_state_prev(10:12);
z2a = eso_state_prev(13:15);
z3a = eso_state_prev(16:18);

% 平移通道：只除一次名义质量，重力显式归入已知项，预测模型包含 RBF 输出
ep_eso = PosL - z1p;
z1_pos = z1p + dt_local * (z2p + beta_pos(:,1) .* ep_eso);
z2_pos = z2p + dt_local * (Fin_prev / m0 - g0 * e3 + fn_app + z3p + beta_pos(:,2) .* ep_eso);
z3_pos = z3p + dt_local * (beta_pos(:,3) .* ep_eso);

% 姿态通道：eta 的二阶导 = T*J0^{-1}(M - omega x J0 omega) + T_dot*omega + 补偿
att_acc_nom = T * (J0 \ (Min_prev - cross(omega, J0 * omega))) + T_dot * omega;
ea_eso = eta - z1a;
z1_att = z1a + dt_local * (z2a + beta_att(:,1) .* ea_eso);
z2_att = z2a + dt_local * (att_acc_nom + fn_att_app + z3a + beta_att(:,2) .* ea_eso);
z3_att = z3a + dt_local * (beta_att(:,3) .* ea_eso);

eso_state = [z1_pos; z2_pos; z3_pos; z1_att; z2_att; z3_att];

%% 复合控制律：名义控制 - RBF 补偿 - ESO 剩余项补偿
eso_scale = eso_config.compensation_scale;
if logical(eso_config.use_compensation)
    Fd_eso = m0 * eso_scale * z3_pos;
    Md_eso = J0 * (T \ (eso_scale * z3_att));
else
    Fd_eso = zeros(3, 1);
    Md_eso = zeros(3, 1);
end

Fd = Fd_nom - m0 * fn_app - Fd_eso;
Md = Md_nom - J0 * (T \ fn_att_app) - Md_eso;

%% RBF 自适应更新：固定学习率 + 泄漏项 + 闭球投影
theta_rbf = theta_rbf_prev(:);
if logical(rbf_config.enable_adaptation)
    gamma_pos = adapt_config.gama;
    gamma_att = adapt_config.gama_att;
    sigma = adapt_config.sigma;
    theta_radius = rbf_config.theta_norm_max;

    % 反步滤波误差：与名义控制同一组增益
    z_pos = e_vL + bs_config.c3 * e_pL;
    z_att = e_Att_dot + bs_config.c1 * e_Att;
    z_ch = [z_pos; z_att];

    for ch = 1:6
        if ch <= 3
            gamma = gamma_pos;
        else
            gamma = gamma_att;
        end
        th = theta_mat(:, ch);
        th = th + dt_local * gamma * (H(:, ch) * z_ch(ch) - sigma * th);
        th_norm = norm(th);
        if th_norm > theta_radius
            th = th * (theta_radius / th_norm);
        end
        theta_mat(:, ch) = th;
    end
    theta_rbf = theta_mat(:);
end

h_rbf_all = H;

end
