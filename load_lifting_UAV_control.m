function [F_plant, M_plant, angled_p, Omegaid_now, e_Ri_all, e_Omegai_all, Rd_i_now, R_i, x_att] = load_lifting_UAV_control(ui, x_plant, angleid_p, Omegaid, Rd_i, controller_config, t)
%% 第三部分：计算无人机的控制力和控制力矩
% 输入参数：
%   ui: 无人机输入力 (3xQuadrotor_Num) - 世界坐标系
%   x_plant: 无人机状态 (12xQuadrotor_Num)
%   angleid_p: 上一时刻期望角度 (3xQuadrotor_Num)
%   Omegaid: 上一时刻期望角速度 (3xQuadrotor_Num)
%   Rd_i: 上一时刻期望旋转矩阵 (3x3xQuadrotor_Num)
%   i: 当前迭代次数
%   controller_type: 控制器类型 ('pd', 'backstepping', 'rbf', 'smc' 等)
%   with_disturbance: 是否包含干扰项 (true/false)
% 输出参数：
%   F_plant: 无人机控制力 (Quadrotor_Numx1)
%   M_plant: 无人机控制力矩 (3xQuadrotor_Num)
%   angled_p: 当前期望角度 (3xQuadrotor_Num)
%   Omegaid_now: 当前期望角速度 (3xQuadrotor_Num)
%   e_Ri_all: 无人机姿态旋转矩阵误差 (3xQuadrotor_Num)
%   e_Omegai_all: 无人机角速度误差 (3xQuadrotor_Num)
%   Rd_i_now: 当前期望旋转矩阵 (3x3xQuadrotor_Num)

global dt Quadrotor_Num e3
global a1_p a2_p a3_p J_p
global b1_p b2_p b3_p
a_p = [a1_p; a2_p; a3_p];
b_p = [b1_p; b2_p; b3_p];

% 姿态角度
x_att = zeros(3,Quadrotor_Num); % phi theta psi
x_ome = zeros(3,Quadrotor_Num); % phi_dot theta_dot psi_dot
R_i = zeros(3,3, Quadrotor_Num);
Rd_3_i = zeros(3, Quadrotor_Num);
Rd_3_i_hat = zeros(3,3, Quadrotor_Num);
R_psid_i = zeros(3,Quadrotor_Num);
Rd_1_i = zeros(3, Quadrotor_Num);
Rd_2_i = zeros(3, Quadrotor_Num);

Rd_i_now = zeros(3,3,Quadrotor_Num);
Rd_i_now_dot = zeros(3, 3, Quadrotor_Num);
% 预期角度
angled_p = zeros(3,Quadrotor_Num);
% 预期角速度
angled_p_dot = zeros(3,Quadrotor_Num);
% 
Omegaid_now_skew = zeros(3,3,Quadrotor_Num);
Omegaid_now = zeros(3,Quadrotor_Num);
Omegaid_now_dot = zeros(3,Quadrotor_Num);
Omegai_hat = zeros(3,3, Quadrotor_Num);
% M_plant
M_plant = zeros(3, Quadrotor_Num);
F_plant = zeros(Quadrotor_Num,1);

% 从controller_config读取参数
if nargin >= 6 && ~isempty(controller_config) && isstruct(controller_config)
    % 使用传入的controller_config
    controller_params = controller_config;
    controller_type = controller_params.controller_type;
    with_disturbance = controller_params.with_disturbance;
else
    % 如果未传入，从配置文件读取（向后兼容）
    controller_params = load_lifting_controller_params();
    controller_type = controller_params.controller_type;
    with_disturbance = controller_params.with_disturbance;
end

uav_M_type = 'rise';
if isfield(controller_params, 'uav_control') && isfield(controller_params.uav_control, 'uav_M_type')
    uav_M_type = lower(controller_params.uav_control.uav_M_type);
elseif isfield(controller_params, 'uav_control') && isfield(controller_params.uav_control, 'uav_F_type')
    uav_M_type = lower(controller_params.uav_control.uav_F_type);
end

e_Ri_all = zeros(3, Quadrotor_Num);
e_Omegai_all = zeros(3, Quadrotor_Num);
persistent psi_last
persistent ekf_state_bank prev_M_plant_bank
tilt_limit_cfg = get_uav_tilt_limit_config(controller_params);
yaw_ref_cfg = get_uav_yaw_reference_config(controller_params);

uav_ekf_enabled = false;
if isfield(controller_params, 'uav_ekf') && isstruct(controller_params.uav_ekf) && ...
        isfield(controller_params.uav_ekf, 'enabled')
    uav_ekf_enabled = logical(controller_params.uav_ekf.enabled);
end

if isempty(prev_M_plant_bank) || ~isequal(size(prev_M_plant_bank), [3, Quadrotor_Num])
    prev_M_plant_bank = zeros(3, Quadrotor_Num);
end
if isempty(ekf_state_bank) || numel(ekf_state_bank) ~= Quadrotor_Num
    ekf_state_bank = repmat(struct('J_hat', [], 'P', [], 'prev_omega', [], 'initialized', false), 1, Quadrotor_Num);
end

for q_num = 1 : Quadrotor_Num
    % 无人机姿态
    x_att(:,q_num) = [x_plant(1,q_num); x_plant(3,q_num); x_plant(5,q_num)];  
    x_ome(:,q_num) = [x_plant(2,q_num); x_plant(4,q_num); x_plant(6,q_num)];
    % R
    R_i(:,:,q_num) = eul2rotm_ZYX(x_att(:,q_num)); 
    % 对参考控制向量施加最大倾角约束，避免无人机出现接近翻转的姿态指令
    ui_cmd = enforce_uav_tilt_limit(ui(:, q_num), tilt_limit_cfg, e3);

    % 姿态旋转矩阵误差
    Rd_3_i(:, q_num) = ui_cmd / norm(ui_cmd);
    Rd_3_i_hat(:,:,q_num) = skew(Rd_3_i(:, q_num)); 

    % ====== 时间变化的期望偏航 psi_d(t) : 正弦 ======
    % 参数（你可自行调整）
    psi0   = yaw_ref_cfg.psi0;
    A      = yaw_ref_cfg.yaw_amplitude;
    w      = yaw_ref_cfg.yaw_frequency;
    % 注意：t 是作为标量参数传入的（当前时刻的时间值）
    if yaw_ref_cfg.enable_sinusoidal_yaw
        psi_cmd = psi0 + A * sin(w * t * 2);
    else
        psi_cmd = psi0;
    end
    
    % （可选）不同无人机加固定偏置，让每架机航向略不同
    delta_psi = yaw_ref_cfg.delta_psi;
    psi_cmd = psi_cmd + (q_num-1)*delta_psi;
    
    % ====== 限速 + wrap（避免突变导致姿态指令过激）======
    psi_dot_max = yaw_ref_cfg.yaw_rate_limit;
    max_step = psi_dot_max * dt;
    
    if isempty(psi_last) || numel(psi_last) ~= Quadrotor_Num
        psi_last = nan(1, Quadrotor_Num);
    end
    if isnan(psi_last(q_num))
        psi_last(q_num) = psi_cmd;
    end
    wrap = @(x) atan2(sin(x), cos(x));
    dpsi = wrap(psi_cmd - psi_last(q_num));
    dpsi = max(min(dpsi, max_step), -max_step);
    psi_d = wrap(psi_last(q_num) + dpsi);
    psi_last(q_num) = psi_d;
    
    % ====== 用 psi_d 构造参考方向向量 b1d（即 R_psid_i）======
    b1d = [cos(psi_d); sin(psi_d); 0];
    
    % ====== 退化保护：避免 b1d 与 b3 近似共线导致除零 ======
    b3 = Rd_3_i(:, q_num);
    tmp = (skew(b3)^2) * b1d;              % 对应你后面 Rd_1_i 分子
    if norm(tmp) < 1e-6
        b1d = [0; 1; 0];
    end
    
    R_psid_i(:, q_num) = b1d;

    Rd_1_i(:, q_num) = -(Rd_3_i_hat(:,:,q_num)^2 * R_psid_i(:, q_num)) / norm(Rd_3_i_hat(:,:,q_num)^2 * R_psid_i(:, q_num));
    Rd_2_i(:, q_num) = (Rd_3_i_hat(:,:,q_num) * R_psid_i(:, q_num)) / norm(Rd_3_i_hat(:,:,q_num) * R_psid_i(:, q_num));
    % 姿态预期矩阵
    Rd_i_now(:, :, q_num) = [Rd_1_i(:, q_num), Rd_2_i(:, q_num), Rd_3_i(:, q_num)];
    Rd_i_now_dot(:, :, q_num) = (Rd_i_now(:, :, q_num) - Rd_i(:, :, q_num)) / dt;

    % 预期角度
    angled_p(:,q_num) = rotm2eul_ZYX(Rd_i_now(:, :, q_num));
    angled_p_dot(:, q_num) = (angled_p(:,q_num) - angleid_p(:,q_num)) / dt;

    % 计算M_plant,无人机控制力矩
    % 从旋转矩阵计算角速度：对于旋转矩阵 R，角速度 omega 满足 R' * R_dot = skew(omega)
    % 因此 omega = vee(R' * R_dot)
    if norm(Rd_i(:,:,q_num), 'fro') < 1e-8
        % No previous reference at startup: avoid fictitious derivative kick.
        Rd_i_now_dot(:,:,q_num) = zeros(3);
    end
    A_rate = Rd_i_now(:, :, q_num)' * Rd_i_now_dot(:, :, q_num);
    Omegaid_now_skew(:,:,q_num) = 0.5 * (A_rate - A_rate');
    Omegaid_now(:,q_num) = vee(Omegaid_now_skew(:,:,q_num));  % 使用vee函数提取角速度向量
    Omegaid_now_dot(:,q_num) = (Omegaid_now(:,q_num) - Omegaid(:,q_num)) / dt;

    e_Ri = 1/2 * vee(Rd_i_now(:, :, q_num)' * R_i(:,:,q_num) - R_i(:,:,q_num)' * Rd_i_now(:, :, q_num));
    
    e_Omegai = x_ome(:,q_num) - R_i(:,:,q_num)' * Rd_i_now(:, :, q_num) * Omegaid_now(:,q_num);
    
    % 保存无人机误差
    e_Ri_all(:, q_num) = e_Ri;
    e_Omegai_all(:, q_num) = e_Omegai;
    Omegai_hat(:,:,q_num) = skew(x_ome(:,q_num));
    
    controller_params_q = controller_params;
    if uav_ekf_enabled
        ekf_config_q = pick_uav_ekf_config(controller_params, q_num);
        [ekf_state_bank(q_num), ~] = update_uav_inertia_ekf( ...
            x_plant(:, q_num), prev_M_plant_bank(:, q_num), dt, ekf_state_bank(q_num), ekf_config_q);
        controller_params_q.uav_model = controller_params.uav_model;
        controller_params_q.uav_model.inertia_diag = ekf_state_bank(q_num).J_hat;
    end

    switch uav_M_type
        case 'pd'
            M_plant(:, q_num) = load_lifting_UAV_control_pd_M( ...
                x_att(:, q_num), x_ome(:, q_num), angled_p(:, q_num), ...
                Omegaid_now(:, q_num), Omegaid_now_dot(:, q_num), controller_params_q, q_num);
        case 'backstepping'
            M_plant(:, q_num) = load_lifting_UAV_control_backstepping_M( ...
                x_att(:, q_num), x_ome(:, q_num), angled_p(:, q_num), ...
                Omegaid_now(:, q_num), Omegaid_now_dot(:, q_num), controller_params_q, q_num);
        otherwise
            M_plant(:, q_num) = load_lifting_UAV_control_RISE_M( ...
                x_att(:, q_num), x_ome(:, q_num), angled_p(:, q_num), ...
                Omegaid_now(:, q_num), Omegaid_now_dot(:, q_num), controller_params_q, q_num);
    end

    if uav_ekf_enabled
        prev_M_plant_bank(:, q_num) = M_plant(:, q_num);
    end

    M_plant(:, q_num) = apply_moment_limit(M_plant(:, q_num), tilt_limit_cfg);

    % for dim = 1:3
    %     if M_plant(dim, q_num) > 3
    %         M_plant(dim, q_num) = 3;
    %     elseif M_plant(dim, q_num) < -3
    %         M_plant(dim, q_num) = -3;
    %     else
    %         M_plant(dim, q_num) = M_plant(dim, q_num);
    %     end
    % end
    
    % 计算 F_plant,无人机控制力
    F_plant(q_num) = ui_cmd' * R_i(:,:,q_num) * e3;
end

end

function tilt_limit_cfg = get_uav_tilt_limit_config(controller_params)
tilt_limit_cfg = struct( ...
    'enabled', true, ...
    'max_tilt_angle', deg2rad(45), ...
    'min_vertical_force', 1.0e-3, ...
    'min_force_norm', 1.0e-6, ...
    'max_moment', [1.2; 1.2; 0.8]);

if isfield(controller_params, 'uav_control') && isfield(controller_params.uav_control, 'command_limit') && ...
        isstruct(controller_params.uav_control.command_limit)
    cfg = controller_params.uav_control.command_limit;
    fields = fieldnames(cfg);
    for idx = 1:numel(fields)
        tilt_limit_cfg.(fields{idx}) = cfg.(fields{idx});
    end
end
end

function ui_cmd = enforce_uav_tilt_limit(ui_cmd_raw, tilt_limit_cfg, e3)
if nargin < 3 || isempty(e3)
    e3 = [0; 0; 1];
end

ui_norm = norm(ui_cmd_raw);
if ui_norm < tilt_limit_cfg.min_force_norm
    ui_cmd = tilt_limit_cfg.min_vertical_force * e3;
    return;
end

vertical_force = dot(ui_cmd_raw, e3);
if vertical_force < tilt_limit_cfg.min_vertical_force
    vertical_force = tilt_limit_cfg.min_vertical_force;
end

horizontal_force = ui_cmd_raw - dot(ui_cmd_raw, e3) * e3;
horizontal_norm = norm(horizontal_force);

if tilt_limit_cfg.enabled
    max_horizontal = tan(tilt_limit_cfg.max_tilt_angle) * vertical_force;
    if horizontal_norm > max_horizontal && horizontal_norm > 0
        horizontal_force = horizontal_force / horizontal_norm * max_horizontal;
    end
end

ui_cmd = horizontal_force + vertical_force * e3;
end

function yaw_ref_cfg = get_uav_yaw_reference_config(controller_params)
yaw_ref_cfg = struct( ...
    'enable_sinusoidal_yaw', false, ...
    'psi0', 0, ...
    'yaw_amplitude', 0, ...
    'yaw_frequency', 0.6, ...
    'delta_psi', 0, ...
    'yaw_rate_limit', deg2rad(30));

if isfield(controller_params, 'uav_control') && isfield(controller_params.uav_control, 'reference') && ...
        isstruct(controller_params.uav_control.reference)
    cfg = controller_params.uav_control.reference;
    fields = fieldnames(cfg);
    for idx = 1:numel(fields)
        yaw_ref_cfg.(fields{idx}) = cfg.(fields{idx});
    end
end
end

function M_limited = apply_moment_limit(M_raw, tilt_limit_cfg)
M_limited = M_raw;
if ~isfield(tilt_limit_cfg, 'max_moment') || isempty(tilt_limit_cfg.max_moment)
    return;
end

limit = tilt_limit_cfg.max_moment;
if isscalar(limit)
    limit = limit * ones(3, 1);
else
    limit = limit(:);
end

for axis_idx = 1:3
    M_limited(axis_idx) = min(max(M_limited(axis_idx), -limit(axis_idx)), limit(axis_idx));
end
end

function ekf_cfg = pick_uav_ekf_config(controller_params, q_num)
ekf_cfg = controller_params.uav_ekf;
ekf_cfg.initial_inertia_diag = pick_ekf_vector(ekf_cfg.initial_inertia_diag, q_num);
ekf_cfg.initial_cov_diag = pick_ekf_vector(ekf_cfg.initial_cov_diag, q_num);
ekf_cfg.process_noise_diag = pick_ekf_vector(ekf_cfg.process_noise_diag, q_num);
ekf_cfg.measurement_noise_diag = pick_ekf_vector(ekf_cfg.measurement_noise_diag, q_num);
ekf_cfg.inertia_min = pick_ekf_vector(ekf_cfg.inertia_min, q_num);
ekf_cfg.inertia_max = pick_ekf_vector(ekf_cfg.inertia_max, q_num);
if ~isfield(ekf_cfg, 'arm_length') || isempty(ekf_cfg.arm_length)
    if isfield(controller_params, 'uav_model') && isfield(controller_params.uav_model, 'arm_length')
        ekf_cfg.arm_length = controller_params.uav_model.arm_length;
    else
        ekf_cfg.arm_length = 1.0;
    end
end
end

function value = pick_ekf_vector(raw_value, q_num)
if isscalar(raw_value)
    value = raw_value * ones(3, 1);
elseif isvector(raw_value)
    raw_value = raw_value(:);
    if numel(raw_value) == 3
        value = raw_value;
    else
        value = raw_value(min(q_num, numel(raw_value))) * ones(3, 1);
    end
elseif size(raw_value, 1) == 3
    value = raw_value(:, min(q_num, size(raw_value, 2)));
else
    value = reshape(raw_value(:, :, min(q_num, size(raw_value, 3))), 3, 1);
end
end
