function params = load_lifting_controller_params(controller_type_override, with_disturbance_override, uav_control_override, params_override)
%% 控制器参数配置文件
% 统一管理所有控制器的参数，包括：
%   - 负载控制器：由 default_controller_type 选择，switch 自动加载对应参数
%   - 无人机推力/力矩控制：默认由 default_uav_controller_type 联动选择，可分别覆盖
%   - load_lifting_allocation.m 中的参数（Kq_i, KsOmega_i, alpha_filter）
%   - load_lifting_UAV_control.m 中的参数（PD/反步/SMC/RISE）由无人机控制器类型加载
%   - RBF控制器参数（c_rbf, c_adapt_rbf, b_rbf, theta_rbf初始值等）
%
% 输入参数（可选）：
%   controller_type_override: 覆盖配置文件中的控制器类型
%   with_disturbance_override: 覆盖配置文件中的干扰状态
%   uav_control_override: 覆盖无人机推力/力矩控制器类型的结构体
%
% 输出参数：
%   params: 结构体，包含所有控制器参数
%       params.controller_type: 控制器类型 ('pd', 'backstepping', 'rbf', 'smc', 'rise')
%       params.with_disturbance: 是否包含干扰项 (true/false)
%       params.allocation.Kq_i: 绳索方向控制增益 (Quadrotor_Num x 1)
%       params.allocation.KsOmega_i: 绳索角速度控制增益 (Quadrotor_Num x 1)
%       params.allocation.alpha_filter: qi_d平滑滤波系数 (标量)
%       params.uav_control.c1: 无人机姿态反步第一层增益 (3 x 3 x Quadrotor_Num)
%       params.uav_control.c2: 无人机姿态反步第二层增益 (3 x 3 x Quadrotor_Num)
%       params.uav_control.uav_F_type: 无人机推力控制器类型 ('pd'|'backstepping'|'smc'|'rise')
%       params.uav_control.uav_M_type: 无人机力矩控制器类型 ('pd'|'backstepping'|'smc'|'rise')
%       params.uav_control.uav_F_pd: PD 参数（uav_F_type='pd' 时使用）
%       params.uav_control.uav_F_backstepping: 反步参数（uav_F_type='backstepping' 时使用）
%       params.uav_control.uav_F_smc: SMC 参数（uav_F_type='smc' 时使用）
%       params.uav_control.uav_F_rise: RISE 参数（uav_F_type='rise' 时使用）
%       params.uav_control.uav_M_pd: PD 参数（uav_M_type='pd' 时使用）
%       params.uav_control.uav_M_backstepping: 反步参数（uav_M_type='backstepping' 时使用）
%       params.uav_control.uav_M_smc: SMC 参数（uav_M_type='smc' 时使用）
%       params.uav_control.uav_M_rise: RISE 参数（uav_M_type='rise' 时使用）
%       params.uav_disturbance: 无人机扰动配置（独立于负载 with_disturbance）
%       params.rbf: RBF控制器参数（仅当controller_type='rbf'时有效）
%           params.rbf.c_rbf: 归一化二维中心 (2 x 5)，中心点 + 四个角点
%           params.rbf.center_scale_pos: 位置通道归一化尺度 [误差; 误差导数]
%           params.rbf.center_scale_att: 姿态通道归一化尺度 [误差; 误差导数]
%           params.rbf.b_rbf: 归一化单位下的节点宽度 (5 x 1)
%           params.rbf.theta_rbf_init: RBF参数初始值 (30 x 1)
%           params.rbf.theta_norm_max: 每通道权重的闭球投影半径（唯一参数保护）
%           params.rbf.load_rbf_params: 是否从Excel加载RBF参数
%           params.rbf.rbf_params_filename: RBF参数文件名
%           params.rbf.adaptation: 固定学习率与泄漏系数 (gama, gama_att, sigma)
%       params.eso: 扩展状态观测器参数（以 Fin/Min 为输入，显式含重力与姿态映射）

global Quadrotor_Num
global m_p1 m_p2 m_p3 m_p4 m_p
global Jx_p Jy_p Jz_p
global a1_p a2_p a3_p
global b1_p b2_p b3_p l_i
global m_L J_L g b1_L b2_L b3_L

% ==================== 控制器类型和干扰状态配置 ====================
% 【负载控制器】在此选择，下方 switch 将自动加载对应参数
default_controller_type = 'rbf';  % 可选: 'pd', 'backstepping', 'rbf', 'smc', 'rise'
default_with_disturbance = false;  % 仅控制负载上的扰动：true=启用, false=禁用

% 【无人机推力/力矩控制】默认联动切换；如有需要，可分别修改 default_uav_F_type / default_uav_M_type
default_uav_controller_type = 'backstepping';  % 论文仿真固定使用无人机反步内环
default_uav_F_type = default_uav_controller_type;
default_uav_M_type = default_uav_controller_type;
default_uav_disturbance_enabled = false;
if nargin >= 3 && ~isempty(uav_control_override) && isstruct(uav_control_override)
    if isfield(uav_control_override, 'uav_controller_type') && ~isempty(uav_control_override.uav_controller_type)
        default_uav_F_type = uav_control_override.uav_controller_type;
        default_uav_M_type = uav_control_override.uav_controller_type;
    end
    if isfield(uav_control_override, 'uav_F_type') && ~isempty(uav_control_override.uav_F_type)
        default_uav_F_type = uav_control_override.uav_F_type;
    end
    if isfield(uav_control_override, 'uav_M_type') && ~isempty(uav_control_override.uav_M_type)
        default_uav_M_type = uav_control_override.uav_M_type;
    end
end

% 如果传入了覆盖参数，使用传入的参数；否则使用默认配置
if nargin >= 1 && ~isempty(controller_type_override)
    controller_type = controller_type_override;
else
    controller_type = default_controller_type;
end

if nargin >= 2 && ~isempty(with_disturbance_override)
    with_disturbance = with_disturbance_override;
else
    with_disturbance = default_with_disturbance;
end

% 初始化参数结构体
params = struct();
params.controller_type = controller_type;
params.with_disturbance = with_disturbance;
% Snapshot nominal model BEFORE any plant-only mismatch is applied.
params.payload_model = struct('mass', m_L, 'inertia', J_L, 'gravity', g);

% 初始化子结构体
params.allocation = struct();
params.uav_control = struct();
params.uav_disturbance = struct();
params.payload_control = struct();  % 货物控制器参数
params.rbf = struct();  % RBF参数（仅当controller_type='rbf'时使用）
params.eso = struct();  % ESO参数（主要供 RBF 负载控制器使用）
params.uav_model = struct();  % 控制器侧无人机转动惯量模型
params.uav_ekf = struct();  % UAV inertia EKF parameters

% 初始化参数数组
params.allocation.KsOmega_i = zeros(Quadrotor_Num, 1);
params.allocation.Kq_i = zeros(Quadrotor_Num, 1);
params.allocation.alpha_filter = 0.3;

params.uav_control.c1 = zeros(3, 3, Quadrotor_Num);
params.uav_control.c2 = zeros(3, 3, Quadrotor_Num);
params.uav_control.uav_F_pd = struct();
params.uav_control.uav_F_backstepping = struct();
params.uav_control.uav_F_smc = struct();
params.uav_control.uav_F_rise = struct();
params.uav_control.uav_M_pd = struct();
params.uav_control.uav_M_backstepping = struct();
params.uav_control.uav_M_smc = struct();
params.uav_control.uav_M_rise = struct();
params.uav_control.command_limit = struct();
params.uav_control.reference = struct();
params.uav_model.inertia_diag = [Jx_p; Jy_p; Jz_p];
params.uav_model.arm_length = l_i;
params.uav_model.a_p = [a1_p; a2_p; a3_p];
params.uav_model.b_p = [b1_p; b2_p; b3_p];
params.uav_ekf.enabled = false;
params.uav_ekf.initial_inertia_diag = params.uav_model.inertia_diag;
params.uav_ekf.initial_cov_diag = [1.0e-3; 1.0e-3; 1.0e-3];
params.uav_ekf.process_noise_diag = [1.0e-7; 1.0e-7; 1.0e-7];
params.uav_ekf.measurement_noise_diag = [1.0e-4; 1.0e-4; 1.0e-4];
params.uav_ekf.inertia_min = [0.02; 0.02; 0.02];
params.uav_ekf.inertia_max = [0.50; 0.50; 0.50];
params.uav_ekf.update_min_omega_norm = 1.0e-4;
params.uav_ekf.update_min_torque_norm = 1.0e-4;
params.uav_ekf.arm_length = l_i;

if with_disturbance
    max_tilt_deg = 55;
else
    max_tilt_deg = 45;
end
params.uav_control.command_limit.enabled = true;
params.uav_control.command_limit.max_tilt_angle = deg2rad(max_tilt_deg);
params.uav_control.command_limit.min_vertical_force = 1.0e-3;
params.uav_control.command_limit.min_force_norm = 1.0e-6;
if with_disturbance
    params.uav_control.command_limit.max_moment = [1.6; 1.6; 1.0];
else
    params.uav_control.command_limit.max_moment = [1.2; 1.2; 0.8];
end

params.uav_control.reference.enable_sinusoidal_yaw = false;
params.uav_control.reference.psi0 = 0;
params.uav_control.reference.yaw_amplitude = 0;
params.uav_control.reference.yaw_frequency = 0.6;
params.uav_control.reference.delta_psi = 0;
params.uav_control.reference.yaw_rate_limit = deg2rad(30);

if isempty(m_L) || isempty(b1_L) || isempty(b2_L) || isempty(b3_L)
    load_lifting_parameters;
end
% ESO：预测模型已显式包含重力、质量与 RBF 输出，因此不再需要单独的输入系数。
% 姿态通道按 eta_ddot = T(eta)*J0^{-1}(M - omega x J0 omega) + T_dot*omega 更新。
omega_o_default = 50;
params.eso.beta_pos = [3 * omega_o_default * ones(3, 1), ...
                       3 * omega_o_default^2 * ones(3, 1), ...
                       omega_o_default^3 * ones(3, 1)];
params.eso.beta_att = [3 * omega_o_default * ones(3, 1), ...
                       3 * omega_o_default^2 * ones(3, 1), ...
                       omega_o_default^3 * ones(3, 1)];
params.eso.use_compensation = true;
params.eso.compensation_scale = 1.0;

% RBF 自适应律：固定学习率 + 泄漏项，去掉与误差相关的可变增益和 Lyapunov 方程求解。
params.rbf.adaptation = struct( ...
    'gama', 20, ...
    'gama_att', 20, ...
    'sigma', 0.05);

% ==================== 无人机扰动配置（独立于负载扰动） ====================
% 1) initial_state: 在仿真开始时直接对无人机状态叠加一个小偏置
% 2) schedule: 在指定时间窗内，对 load_lifting_UAV_dynamics 叠加加性扰动
%    position_accel 单位为 m/s^2，attitude_accel 单位为 rad/s^2
%    位置扰动可适当比姿态扰动大一些
params.uav_disturbance.enabled = logical(with_disturbance);
params.uav_disturbance.initial_state = struct( ...
    'enabled', true, ...
    'uav_ids', [1, 2, 3], ...
    'position', [0.02; -0.015; 0.03], ...
    'velocity', zeros(3, 1), ...
    'attitude', (pi / 180) * [0.4; -0.3; 0.2], ...
    'angular_rate', zeros(3, 1));
params.uav_disturbance.schedule = [ ...
    struct('name', 'startup_small_bias', ...
           'enabled', false, ...
           'uav_ids', 1:Quadrotor_Num, ...
           'start_time', 0.0, ...
           'end_time', 1.5, ...
           'position_accel', [0.18; -0.12; 0.15], ...
           'attitude_accel', (pi / 180) * [0.8; -0.6; 0.5]); ...
    struct('name', 'uav_big_pos_disturbance_10s', ...
           'enabled', true, ...
           'uav_ids', [1, 2, 3], ...
           'start_time', 10.0, ...
           'end_time', 10.5, ...
           'position_accel', [1.5; -1.2; 1.0], ...
           'attitude_accel', [0; 0; 0])];

% 初始化货物控制器参数（所有控制器类型）
params.payload_control.pd = struct();
params.payload_control.backstepping = struct();
params.payload_control.smc = struct();
params.payload_control.rise = struct();

% ==================== 根据控制器类型和干扰项设置参数 ====================
switch lower(controller_type)
    case 'pd'
        % ========== PD控制器参数（负载控制） ==========
        if with_disturbance
            % 有干扰：需要更强的增益
            params.allocation.KsOmega_i(1) = 50;
            params.allocation.KsOmega_i(2) = 35;
            params.allocation.KsOmega_i(3) = 25;
            params.allocation.Kq_i(1) = 60;
            params.allocation.Kq_i(2) = 60;
            params.allocation.Kq_i(3) = 60;
            params.allocation.alpha_filter = 0.25;  % 更平滑
            
            % PD控制器参数（货物控制）
            params.payload_control.pd.k_RL = 18;  % 姿态旋转矩阵误差增益
            params.payload_control.pd.k_omegaL = 6;  % 角速度误差增益
            params.payload_control.pd.k_p_L = diag([10, 10, 10]);  % 位置误差增益
            params.payload_control.pd.k_v_L = diag(2 * sqrt(m_L * diag(params.payload_control.pd.k_p_L)));  % 速度误差增益
        else
            % 无干扰：较温和的增益
            params.allocation.KsOmega_i(1) = 40;
            params.allocation.KsOmega_i(2) = 28;
            params.allocation.KsOmega_i(3) = 18;
            params.allocation.Kq_i(1) = 50;
            params.allocation.Kq_i(2) = 50;
            params.allocation.Kq_i(3) = 50;
            params.allocation.alpha_filter = 0.3;
            
            % PD控制器参数（货物控制）
            params.payload_control.pd.k_RL = 15;  % 姿态旋转矩阵误差增益
            params.payload_control.pd.k_omegaL = 5;  % 角速度误差增益
            params.payload_control.pd.k_p_L = diag([8, 8, 8]);  % 位置误差增益
            params.payload_control.pd.k_v_L = diag(2 * sqrt(m_L * diag(params.payload_control.pd.k_p_L)));  % 速度误差增益
        end
        
    case 'backstepping'
        % ========== 反步控制器参数（负载控制） ==========
        if with_disturbance
            % 有干扰：需要更强的增益
            params.allocation.KsOmega_i(1) = 50;
            params.allocation.KsOmega_i(2) = 35;
            params.allocation.KsOmega_i(3) = 25;
            params.allocation.Kq_i(1) = 60;
            params.allocation.Kq_i(2) = 60;
            params.allocation.Kq_i(3) = 60;
            params.allocation.alpha_filter = 0.25;
            
            % 反步控制器参数（货物控制）
            params.payload_control.backstepping.c1 = diag([4, 4, 4]);  % 姿态第一层增益
            params.payload_control.backstepping.c2 = diag([2, 2, 2]);  % 姿态第二层增益
            params.payload_control.backstepping.c3 = diag([4, 4, 4]);  % 位置第一层增益
            params.payload_control.backstepping.c4 = diag([2, 2, 2]);  % 位置第二层增益
        else
            % 无干扰：中等增益
            params.allocation.KsOmega_i(1) = 43.7;
            params.allocation.KsOmega_i(2) = 28.23;
            params.allocation.KsOmega_i(3) = 17.54;
            params.allocation.Kq_i(1) = 50;
            params.allocation.Kq_i(2) = 50.1;
            params.allocation.Kq_i(3) = 50;
            params.allocation.alpha_filter = 0.3;
            
            % 反步控制器参数（货物控制）
            params.payload_control.backstepping.c1 = diag([3, 3, 3]);  % 姿态第一层增益
            params.payload_control.backstepping.c2 = diag([1.5, 1.5, 1.5]);  % 姿态第二层增益
            params.payload_control.backstepping.c3 = diag([3, 3, 3]);  % 位置第一层增益
            params.payload_control.backstepping.c4 = diag([1.5, 1.5, 1.5]);  % 位置第二层增益
        end
        
    case 'rbf'
        % ========== RBF神经网络控制器参数（负载控制） ==========
        if with_disturbance
            % 有干扰：高增益，快速响应
            params.allocation.KsOmega_i(1) = 55;
            params.allocation.KsOmega_i(2) = 40;
            params.allocation.KsOmega_i(3) = 30;
            params.allocation.Kq_i(1) = 65;
            params.allocation.Kq_i(2) = 65;
            params.allocation.Kq_i(3) = 65;
            params.allocation.alpha_filter = 0.2;  % 响应更快
            % 与反步控制器完全相同的名义增益：四组消融必须共用同一名义控制
            params.payload_control.backstepping.c1 = diag([4, 4, 4]);
            params.payload_control.backstepping.c2 = diag([2, 2, 2]);
            params.payload_control.backstepping.c3 = diag([4, 4, 4]);
            params.payload_control.backstepping.c4 = diag([2, 2, 2]);
        else
            % 无干扰：使用和反步控制器相同的参数（为了让去掉RBF补偿后结果一致）
            params.allocation.KsOmega_i(1) = 43.7;  % 与反步控制器一致
            params.allocation.KsOmega_i(2) = 28.23;  % 与反步控制器一致
            params.allocation.KsOmega_i(3) = 17.54;  % 与反步控制器一致
            params.allocation.Kq_i(1) = 50;  % 与反步控制器一致
            params.allocation.Kq_i(2) = 50.1;  % 与反步控制器一致
            params.allocation.Kq_i(3) = 50;  % 与反步控制器一致
            params.allocation.alpha_filter = 0.3;  % 与反步控制器一致
            % 与反步控制器完全相同的名义增益
            params.payload_control.backstepping.c1 = diag([3, 3, 3]);
            params.payload_control.backstepping.c2 = diag([1.5, 1.5, 1.5]);
            params.payload_control.backstepping.c3 = diag([3, 3, 3]);
            params.payload_control.backstepping.c4 = diag([1.5, 1.5, 1.5]);
        end
        
        % ========== RBF网络参数配置（返修版：统一二维基函数） ==========
        % 每个通道输入 [本通道误差; 本通道误差导数]，输出与自适应更新共用同一套
        % 归一化基函数。中心在归一化平面上取"中心点 + 四个角点"，不做聚类。
        params.rbf.c_rbf = [-1, -1,  1,  1, 0; ...
                            -1,  1, -1,  1, 0];   % 2 x 5（归一化单位）
        % 归一化尺度：误差 / 误差导数 各自的特征量级（物理单位）
        params.rbf.center_scale_pos = [0.5; 0.5];    % [m; m/s]
        params.rbf.center_scale_att = [0.10; 0.50];  % [rad; rad/s]
        % 归一化单位下的节点宽度
        params.rbf.b_rbf = 0.8 * ones(5, 1);

        % RBF参数初始值（30x1：6自由度 × 5个RBF节点）
        params.rbf.theta_rbf_init = zeros(30, 1);

        % 唯一的参数保护：每个通道权重向量的闭球投影半径
        params.rbf.theta_norm_max = 5;
        params.rbf.enable_compensation = true;
        params.rbf.compensation_scale = 1.0;
        params.rbf.enable_adaptation = true;
        params.rbf.base_mode = 'backstepping';
        
        % RBF参数加载选项
        params.rbf.load_rbf_params = false;  % 是否从Excel文件加载上次训练的RBF参数
        params.rbf.rbf_params_filename = 'rbf_params_rise.xlsx';  % RBF参数文件名
        
        % ========== RBF控制器的RISE基准控制参数 ==========
        if with_disturbance
            params.payload_control.rise.k_s_pos = [1.8; 1.8; 2.5];
            params.payload_control.rise.k1_pos = [5.0; 5.0; 8.0];
            params.payload_control.rise.k2_pos = [2.5; 2.5; 4.0];
            params.payload_control.rise.Imax_pos = 40;
            params.payload_control.rise.eps_s_pos = 0.05;

            params.payload_control.rise.k_s_att = [10.0; 10.0; 8.0];
            params.payload_control.rise.k1_att = [25.0; 25.0; 15.0];
            params.payload_control.rise.k2_att = [6.0; 6.0; 3.0];
            params.payload_control.rise.Imax_att = 80;
            params.payload_control.rise.eps_s_att = 0.05;
        else
            params.payload_control.rise.k_s_pos = [1.5; 1.5; 2.0];
            params.payload_control.rise.k1_pos = [4.0; 4.0; 6.0];
            params.payload_control.rise.k2_pos = [2.0; 2.0; 3.0];
            params.payload_control.rise.Imax_pos = 30;
            params.payload_control.rise.eps_s_pos = 0.05;

            params.payload_control.rise.k_s_att = [8.0; 8.0; 6.0];
            params.payload_control.rise.k1_att = [20.0; 20.0; 12.0];
            params.payload_control.rise.k2_att = [5.0; 5.0; 2.0];
            params.payload_control.rise.Imax_att = 60;
            params.payload_control.rise.eps_s_att = 0.05;
        end
        
    case 'smc'
        % ========== 滑模控制器参数（负载控制） ==========
        if with_disturbance
            % 有干扰：最高增益，强鲁棒性
            params.allocation.KsOmega_i(1) = 60;
            params.allocation.KsOmega_i(2) = 45;
            params.allocation.KsOmega_i(3) = 35;
            params.allocation.Kq_i(1) = 70;
            params.allocation.Kq_i(2) = 70;
            params.allocation.Kq_i(3) = 70;
            params.allocation.alpha_filter = 0.2;
            
            % SMC控制器参数（货物控制）
            params.payload_control.smc.c1_pos = diag([6, 7, 40]);  % 位置滑模面参数
            params.payload_control.smc.c2_att = diag([2.5, 2.5, 5.0]);  % 姿态滑模面参数
            params.payload_control.smc.k_smc = 0.08;  % 位置切换增益
            params.payload_control.smc.q_smc = 6;  % 位置等效增益
            params.payload_control.smc.k_smc2 = 0.15;  % 姿态切换增益
            params.payload_control.smc.q_smc2 = 14;  % 姿态等效增益
            params.payload_control.smc.smooth_eps_att = 0.05;  % 姿态切换项平滑宽度
        else
            % 无干扰：姿态优先的温和增益，避免初始大位置力通过绳索分配激发pitch误差
            params.allocation.KsOmega_i(1) = 50;
            params.allocation.KsOmega_i(2) = 35;
            params.allocation.KsOmega_i(3) = 25;
            params.allocation.Kq_i(1) = 60;
            params.allocation.Kq_i(2) = 60;
            params.allocation.Kq_i(3) = 60;
            params.allocation.alpha_filter = 0.3;
            
            % SMC控制器参数（货物控制）
            params.payload_control.smc.c1_pos = diag([3, 3, 8]);  % 位置滑模面参数
            params.payload_control.smc.c2_att = diag([1, 1, 2]);  % 姿态滑模面参数
            params.payload_control.smc.k_smc = 0.03;  % 位置切换增益
            params.payload_control.smc.q_smc = 2.5;  % 位置等效增益
            params.payload_control.smc.k_smc2 = 0.05;  % 姿态切换增益
            params.payload_control.smc.q_smc2 = 4;  % 姿态等效增益
            params.payload_control.smc.smooth_eps_att = 0.05;  % 姿态切换项平滑宽度
        end
        
    case 'rise'
        % ========== RISE控制器参数（负载控制） ==========
        if with_disturbance
            % 有干扰：需要更强的增益
            params.allocation.KsOmega_i(1) = 50;
            params.allocation.KsOmega_i(2) = 35;
            params.allocation.KsOmega_i(3) = 25;
            params.allocation.Kq_i(1) = 60;
            params.allocation.Kq_i(2) = 60;
            params.allocation.Kq_i(3) = 60;
            params.allocation.alpha_filter = 0.25;
            
            % RISE控制器参数（货物控制）
            % 位置控制参数
            params.payload_control.rise.k_s_pos = [1.8; 1.8; 2.5];      % 滑模面增益 [X; Y; Z]
            params.payload_control.rise.k1_pos = [5.0; 5.0; 8.0];      % RISE增益1 [X; Y; Z]
            params.payload_control.rise.k2_pos = [2.5; 2.5; 4.0];      % RISE增益2 [X; Y; Z]
            params.payload_control.rise.Imax_pos = 40;                 % 位置积分限幅
            params.payload_control.rise.eps_s_pos = 0.05;              % 位置平滑sign函数参数
            % 姿态控制参数
            params.payload_control.rise.k_s_att = [10.0; 10.0; 8.0];   % 滑模面增益 [Roll; Pitch; Yaw]
            params.payload_control.rise.k1_att = [25.0; 25.0; 15.0];    % RISE增益1 [Roll; Pitch; Yaw]
            params.payload_control.rise.k2_att = [6.0; 6.0; 3.0];       % RISE增益2 [Roll; Pitch; Yaw]
            params.payload_control.rise.Imax_att = 80;                  % 姿态积分限幅
            params.payload_control.rise.eps_s_att = 0.05;               % 姿态平滑sign函数参数
        else
            % 无干扰：中等增益
            params.allocation.KsOmega_i(1) = 43.7;
            params.allocation.KsOmega_i(2) = 28.23;
            params.allocation.KsOmega_i(3) = 17.54;
            params.allocation.Kq_i(1) = 50;
            params.allocation.Kq_i(2) = 50.1;
            params.allocation.Kq_i(3) = 50;
            params.allocation.alpha_filter = 0.3;
            
            % RISE控制器参数（货物控制）
            % 位置控制参数
            params.payload_control.rise.k_s_pos = [1.5; 1.5; 2.0];      % 滑模面增益 [X; Y; Z]
            params.payload_control.rise.k1_pos = [4.0; 4.0; 6.0];       % RISE增益1 [X; Y; Z]
            params.payload_control.rise.k2_pos = [2.0; 2.0; 3.0];       % RISE增益2 [X; Y; Z]
            params.payload_control.rise.Imax_pos = 30;                  % 位置积分限幅
            params.payload_control.rise.eps_s_pos = 0.05;               % 位置平滑sign函数参数
            % 姿态控制参数
            params.payload_control.rise.k_s_att = [8.0; 8.0; 6.0];     % 滑模面增益 [Roll; Pitch; Yaw]
            params.payload_control.rise.k1_att = [20.0; 20.0; 12.0];   % RISE增益1 [Roll; Pitch; Yaw]
            params.payload_control.rise.k2_att = [5.0; 5.0; 2.0];      % RISE增益2 [Roll; Pitch; Yaw]
            params.payload_control.rise.Imax_att = 60;                  % 姿态积分限幅
            params.payload_control.rise.eps_s_att = 0.05;               % 姿态平滑sign函数参数
        end
        
    otherwise
        % 未知控制器类型，使用默认的反步控制器参数
        warning('未知的控制器类型 "%s"，使用默认的反步控制器参数', controller_type);
        if with_disturbance
            params.allocation.KsOmega_i(1) = 50;
            params.allocation.KsOmega_i(2) = 35;
            params.allocation.KsOmega_i(3) = 25;
            params.allocation.Kq_i(1) = 60;
            params.allocation.Kq_i(2) = 60;
            params.allocation.Kq_i(3) = 60;
            params.allocation.alpha_filter = 0.25;
        else
            params.allocation.KsOmega_i(1) = 43.7;
            params.allocation.KsOmega_i(2) = 28.23;
            params.allocation.KsOmega_i(3) = 17.54;
            params.allocation.Kq_i(1) = 50;
            params.allocation.Kq_i(2) = 50.1;
            params.allocation.Kq_i(3) = 50;
            params.allocation.alpha_filter = 0.3;
        end
end

% ==================== 无人机控制器参数 ====================
% 推力 F_Plant 和力矩 M 可分别切换；默认都跟随 default_uav_controller_type
params.uav_control.uav_F_type = default_uav_F_type;
params.uav_control.uav_M_type = default_uav_M_type;

switch lower(default_uav_F_type)
    case 'pd'
        if with_disturbance
            kp_pd = diag([10, 10, 12]);
            kv_pd = diag(2 * sqrt(m_p(1) * diag(kp_pd)));
            params.uav_control.uav_F_pd = struct('k_p', kp_pd, 'k_v', kv_pd);
            params.uav_control.uav_F_rise = struct( ...
                'k_s_pos', [1.8; 1.8; 2.4], ...
                'k1_pos', [5.0; 5.0; 7.0], ...
                'k2_pos', [2.5; 2.5; 3.5], ...
                'Imax_pos', 35, ...
                'eps_s_pos', 0.05);
        else
            kp_pd = diag([8, 8, 8]);
            kv_pd = diag(2 * sqrt(m_p(1) * diag(kp_pd)));
            params.uav_control.uav_F_pd = struct('k_p', kp_pd, 'k_v', kv_pd);
            params.uav_control.uav_F_rise = struct( ...
                'k_s_pos', [1.5; 1.5; 2.0], ...
                'k1_pos', [4.0; 4.0; 6.0], ...
                'k2_pos', [2.0; 2.0; 3.0], ...
                'Imax_pos', 30, ...
                'eps_s_pos', 0.05);
        end
        params.uav_control.uav_F_backstepping = struct('c3', diag([3, 3, 3]), 'c4', diag([1.5, 1.5, 1.5]));
    case 'backstepping'
        if with_disturbance
            params.uav_control.uav_F_backstepping = struct('c3', diag([4, 4, 4]), 'c4', diag([2, 2, 2]));
            params.uav_control.uav_F_rise = struct( ...
                'k_s_pos', [1.8; 1.8; 2.4], ...
                'k1_pos', [5.0; 5.0; 7.0], ...
                'k2_pos', [2.5; 2.5; 3.5], ...
                'Imax_pos', 35, ...
                'eps_s_pos', 0.05);
        else
            params.uav_control.uav_F_backstepping = struct('c3', diag([3, 3, 3]), 'c4', diag([1.5, 1.5, 1.5]));
            params.uav_control.uav_F_rise = struct( ...
                'k_s_pos', [1.5; 1.5; 2.0], ...
                'k1_pos', [4.0; 4.0; 6.0], ...
                'k2_pos', [2.0; 2.0; 3.0], ...
                'Imax_pos', 30, ...
                'eps_s_pos', 0.05);
        end
        kp_pd = diag([8, 8, 8]);
        kv_pd = diag(2 * sqrt(m_p(1) * diag(kp_pd)));
        params.uav_control.uav_F_pd = struct('k_p', kp_pd, 'k_v', kv_pd);
    otherwise  % 'rise' 及其他
        if with_disturbance
            params.uav_control.uav_F_rise = struct( ...
                'k_s_pos', [2.257920; 2.257920; 3.171840], ...
                'k1_pos', [6.272000; 6.272000; 9.251200], ...
                'k2_pos', [3.136000; 3.136000; 4.625600], ...
                'Imax_pos', 42.0, ...
                'eps_s_pos', 0.045);
        else
            params.uav_control.uav_F_rise = struct( ...
                'k_s_pos', [1.749000; 1.669500; 2.438000], ...
                'k1_pos', [4.664000; 4.452000; 7.314000], ...
                'k2_pos', [2.332000; 2.226000; 3.657000], ...
                'Imax_pos', 34.5, ...
                'eps_s_pos', 0.045);
        end
        kp_pd = diag([8, 8, 8]);
        kv_pd = diag(2 * sqrt(m_p(1) * diag(kp_pd)));
        params.uav_control.uav_F_pd = struct('k_p', kp_pd, 'k_v', kv_pd);
        params.uav_control.uav_F_backstepping = struct('c3', diag([3, 3, 3]), 'c4', diag([1.5, 1.5, 1.5]));
end

switch lower(default_uav_M_type)
    case 'pd'
        if with_disturbance
            params.uav_control.uav_M_pd = struct('k_p_att', diag([35, 35, 22]), 'k_d_att', diag([9, 9, 5]));
            params.uav_control.uav_M_rise = struct( ...
                'k_s_att', [9.0; 9.0; 7.0], ...
                'k1_att', [18.0; 18.0; 12.0], ...
                'k2_att', [4.5; 4.5; 2.5], ...
                'Imax_att', 60, ...
                'eps_s_att', 0.05);
        else
            params.uav_control.uav_M_pd = struct('k_p_att', diag([28, 28, 18]), 'k_d_att', diag([6, 6, 4]));
            params.uav_control.uav_M_rise = struct( ...
                'k_s_att', [7.0; 7.0; 5.5], ...
                'k1_att', [14.0; 14.0; 9.0], ...
                'k2_att', [3.5; 3.5; 2.0], ...
                'Imax_att', 45, ...
                'eps_s_att', 0.05);
        end
        params.uav_control.uav_M_backstepping = struct('c1', zeros(3, 3, Quadrotor_Num), 'c2', zeros(3, 3, Quadrotor_Num));
    case 'backstepping'
        if with_disturbance
            params.uav_control.uav_M_backstepping.c1(:,:,1) = diag([25, 25, 60]);
            params.uav_control.uav_M_backstepping.c1(:,:,2) = diag([25, 25, 25]);
            params.uav_control.uav_M_backstepping.c1(:,:,3) = diag([25, 25, 25]);
            params.uav_control.uav_M_backstepping.c2(:,:,1) = diag([15, 15, 3]);
            params.uav_control.uav_M_backstepping.c2(:,:,2) = diag([15, 15, 15]);
            params.uav_control.uav_M_backstepping.c2(:,:,3) = diag([15, 15, 15]);
            params.uav_control.uav_M_rise = struct( ...
                'k_s_att', [9.0; 9.0; 7.0], ...
                'k1_att', [18.0; 18.0; 12.0], ...
                'k2_att', [4.5; 4.5; 2.5], ...
                'Imax_att', 60, ...
                'eps_s_att', 0.05);
        else
            params.uav_control.uav_M_backstepping.c1(:,:,1) = diag([20, 20, 50]);
            params.uav_control.uav_M_backstepping.c1(:,:,2) = diag([20, 20, 20]);
            params.uav_control.uav_M_backstepping.c1(:,:,3) = diag([20, 20, 20]);
            params.uav_control.uav_M_backstepping.c2(:,:,1) = diag([12, 12, 2]);
            params.uav_control.uav_M_backstepping.c2(:,:,2) = diag([12, 12, 12]);
            params.uav_control.uav_M_backstepping.c2(:,:,3) = diag([12, 12, 12]);
            params.uav_control.uav_M_rise = struct( ...
                'k_s_att', [7.0; 7.0; 5.5], ...
                'k1_att', [14.0; 14.0; 9.0], ...
                'k2_att', [3.5; 3.5; 2.0], ...
                'Imax_att', 45, ...
                'eps_s_att', 0.05);
        end
        params.uav_control.uav_M_pd = struct('k_p_att', diag([28, 28, 18]), 'k_d_att', diag([6, 6, 4]));
    otherwise
        if with_disturbance
            params.uav_control.uav_M_rise = struct( ...
                'k_s_att', [10.350000; 10.350000; 7.840000], ...
                'k1_att', [20.700000; 20.700000; 13.440000], ...
                'k2_att', [5.175000; 5.175000; 2.800000], ...
                'Imax_att', 72.0, ...
                'eps_s_att', 0.045);
        else
            params.uav_control.uav_M_rise = struct( ...
                'k_s_att', [7.840000; 7.700000; 5.940000], ...
                'k1_att', [15.680000; 15.400000; 9.720000], ...
                'k2_att', [3.920000; 3.850000; 2.160000], ...
                'Imax_att', 50.4, ...
                'eps_s_att', 0.045);
        end
        params.uav_control.uav_M_pd = struct('k_p_att', diag([28, 28, 18]), 'k_d_att', diag([6, 6, 4]));
        params.uav_control.uav_M_backstepping = struct('c1', zeros(3, 3, Quadrotor_Num), 'c2', zeros(3, 3, Quadrotor_Num));
end

if ~isfield(params.uav_control.uav_M_backstepping, 'c1') || isempty(params.uav_control.uav_M_backstepping.c1)
    params.uav_control.uav_M_backstepping.c1 = zeros(3, 3, Quadrotor_Num);
end
if ~isfield(params.uav_control.uav_M_backstepping, 'c2') || isempty(params.uav_control.uav_M_backstepping.c2)
    params.uav_control.uav_M_backstepping.c2 = zeros(3, 3, Quadrotor_Num);
end

if with_disturbance
    params.uav_control.uav_F_smc = struct( ...
        'lambda_pos', [5.0; 5.0; 8.0], ...
        'k_lin_pos', [6.0; 6.0; 6.0], ...
        'k_switch_pos', [0.08; 0.08; 0.08], ...
        'smooth_eps', 0.05);
    params.uav_control.uav_M_smc = struct( ...
        'lambda_att', [1.2; 1.2; 2.5], ...
        'k_lin_att', [0.6; 0.6; 0.6], ...
        'k_switch_att', [0.08; 0.08; 0.08], ...
        'smooth_eps', 0.05);
else
    params.uav_control.uav_F_smc = struct( ...
        'lambda_pos', [4.2; 4.2; 6.0], ...
        'k_lin_pos', [5.0; 5.0; 5.0], ...
        'k_switch_pos', [0.05; 0.05; 0.05], ...
        'smooth_eps', 0.05);
    params.uav_control.uav_M_smc = struct( ...
        'lambda_att', [1.4; 1.5; 2.0], ...
        'k_lin_att', [0.6; 0.6; 0.6], ...
        'k_switch_att', [0.05; 0.05; 0.05], ...
        'smooth_eps', 0.05);
end

% 精简论文仓库不加载外部调参覆盖文件，保证仿真只依赖仓库内的论文参数。
disable_saved_overrides = true;
if nargin >= 4 && ~isempty(params_override) && isstruct(params_override) && ...
        isfield(params_override, 'control_flags') && isstruct(params_override.control_flags) && ...
        isfield(params_override.control_flags, 'disable_saved_overrides')
    disable_saved_overrides = logical(params_override.control_flags.disable_saved_overrides);
    params_override = rmfield(params_override, 'control_flags');
end

if ~disable_saved_overrides
    saved_override = load_saved_controller_overrides(controller_type, with_disturbance, default_uav_F_type, default_uav_M_type);
    if ~isempty(fieldnames(saved_override))
        params = merge_struct_recursive(params, saved_override);
    end

    if strcmpi(default_uav_F_type, 'rise') || strcmpi(default_uav_M_type, 'rise')
        uav_txt_override = load_uav_rise_override_from_txt(with_disturbance, '', pwd);
        if ~isempty(fieldnames(uav_txt_override))
            params = merge_struct_recursive(params, uav_txt_override);
        end
    end

    if ~strcmpi(default_uav_F_type, 'rise')
        uav_f_override = load_uav_controller_override_from_txt(default_uav_F_type, with_disturbance, '', pwd);
        if ~isempty(fieldnames(uav_f_override))
            params = merge_struct_recursive(params, uav_f_override);
        end
    end
    if ~strcmpi(default_uav_M_type, default_uav_F_type)
        uav_m_override = load_uav_controller_override_from_txt(default_uav_M_type, with_disturbance, '', pwd);
        if ~isempty(fieldnames(uav_m_override))
            params = merge_struct_recursive(params, uav_m_override);
        end
    end
end

if nargin >= 4 && ~isempty(params_override) && isstruct(params_override)
    params = merge_struct_recursive(params, params_override);
end

params.uav_model = finalize_uav_model_params(params.uav_model);

params.uav_control.c1 = params.uav_control.uav_M_backstepping.c1;
params.uav_control.c2 = params.uav_control.uav_M_backstepping.c2;

end

function uav_model = finalize_uav_model_params(uav_model)
global Jx_p Jy_p Jz_p
global a1_p a2_p a3_p
global b1_p b2_p b3_p l_i

if nargin < 1 || isempty(uav_model)
    uav_model = struct();
end

if ~isfield(uav_model, 'arm_length') || isempty(uav_model.arm_length)
    uav_model.arm_length = l_i;
end

has_explicit_ab = isfield(uav_model, 'a_p') && ~isempty(uav_model.a_p) && ...
    isfield(uav_model, 'b_p') && ~isempty(uav_model.b_p);

if isfield(uav_model, 'inertia_diag') && ~isempty(uav_model.inertia_diag)
    raw_inertia = uav_model.inertia_diag;
    if size(raw_inertia, 1) == 3 && size(raw_inertia, 2) >= 1
        inertia_diag = reshape(raw_inertia(:, 1), 3, 1);
    else
        inertia_diag = reshape(raw_inertia, 3, 1);
    end
else
    inertia_diag = [Jx_p; Jy_p; Jz_p];
end
uav_model.inertia_diag = inertia_diag;

if has_explicit_ab
    if size(uav_model.a_p, 1) == 3 && size(uav_model.a_p, 2) >= 1
        uav_model.a_p = reshape(uav_model.a_p(:, 1), 3, 1);
    else
        uav_model.a_p = reshape(uav_model.a_p, 3, 1);
    end
    if size(uav_model.b_p, 1) == 3 && size(uav_model.b_p, 2) >= 1
        uav_model.b_p = reshape(uav_model.b_p(:, 1), 3, 1);
    else
        uav_model.b_p = reshape(uav_model.b_p, 3, 1);
    end
else
    [a_p, b_p] = compute_ab_from_inertia(inertia_diag, uav_model.arm_length);
    uav_model.a_p = a_p;
    uav_model.b_p = b_p;
end

if isempty(uav_model.a_p)
    uav_model.a_p = [a1_p; a2_p; a3_p];
end
if isempty(uav_model.b_p)
    uav_model.b_p = [b1_p; b2_p; b3_p];
end
end
