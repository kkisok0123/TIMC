%% 四架无人机
% 允许批处理脚本在运行前注入配置覆盖项
controller_type_override_local = [];
with_disturbance_override_local = [];
uav_controller_type_override_local = [];
uav_F_type_override_local = [];
uav_M_type_override_local = [];
controller_param_override_local = [];
save_run_data_local = true;
run_output_dir_local = fullfile(pwd, 'simulation_runs');
enable_default_plots_local = true;
simulation_dt_override_local = [];
simulation_Tf_override_local = [];

if exist('controller_type_override', 'var')
    controller_type_override_local = controller_type_override;
end
if exist('with_disturbance_override', 'var')
    with_disturbance_override_local = with_disturbance_override;
end
if exist('uav_controller_type_override', 'var')
    uav_controller_type_override_local = uav_controller_type_override;
end
if exist('uav_F_type_override', 'var')
    uav_F_type_override_local = uav_F_type_override;
end
if exist('uav_M_type_override', 'var')
    uav_M_type_override_local = uav_M_type_override;
end
if exist('controller_param_override', 'var')
    controller_param_override_local = controller_param_override;
end
if exist('save_run_data', 'var')
    save_run_data_local = save_run_data;
end
if exist('run_output_dir', 'var')
    run_output_dir_local = run_output_dir;
end
if exist('enable_default_plots', 'var')
    enable_default_plots_local = enable_default_plots;
end
if exist('simulation_dt_override', 'var')
    simulation_dt_override_local = simulation_dt_override;
end
if exist('simulation_Tf_override', 'var')
    simulation_Tf_override_local = simulation_Tf_override;
end

clc;
% 批处理模式下不执行 clearvars，以保留 batch_* 等循环变量；否则清除工作区
if ~exist('batch_output_dir', 'var')
    clearvars -except controller_type_override_local with_disturbance_override_local ...
        uav_controller_type_override_local uav_F_type_override_local uav_M_type_override_local ...
        controller_param_override_local save_run_data_local run_output_dir_local ...
        enable_default_plots_local simulation_dt_override_local simulation_Tf_override_local
end
close all;
% Reset the existing persistent controller banks at every simulation start.
clear load_lifting_UAV_control load_lifting_payload_control_rbf load_lifting_UAV_control_backstepping_F
format long;

%% ==================== 参数加载 ====================
global dt Quadrotor_Num e3
global Tf l_plant load_height t
load_lifting_parameters;
if ~isempty(simulation_dt_override_local)
    dt = simulation_dt_override_local;
end
if ~isempty(simulation_Tf_override_local)
    Tf = simulation_Tf_override_local;
end

%% ==================== 控制器类型选择 ====================
% 可选值：
%   'pd' 或 'classic' - PD控制器（经典控制器）
%   'backstepping' - 反步控制器
%   'smc' - SMC滑模控制器
%   'rbf' - RBF神经网络控制器
% 注意：控制器类型和干扰状态现在统一在 load_lifting_controller_params.m 中配置
% 从统一参数配置文件加载控制器类型和干扰状态
uav_control_override_local = struct();
if ~isempty(uav_controller_type_override_local)
    uav_control_override_local.uav_controller_type = uav_controller_type_override_local;
end
if ~isempty(uav_F_type_override_local)
    uav_control_override_local.uav_F_type = uav_F_type_override_local;
end
if ~isempty(uav_M_type_override_local)
    uav_control_override_local.uav_M_type = uav_M_type_override_local;
end
if isempty(fieldnames(uav_control_override_local))
    uav_control_override_local = [];
end
controller_config = load_lifting_controller_params( ...
    controller_type_override_local, ...
    with_disturbance_override_local, ...
    uav_control_override_local, ...
    controller_param_override_local);
payload_controller_type = controller_config.controller_type;
with_disturbance = controller_config.with_disturbance;
uav_disturbance_config = [];
experiment_config = struct();
if isfield(controller_config, 'uav_disturbance')
    uav_disturbance_config = controller_config.uav_disturbance;
end
if isfield(controller_config, 'experiment') && isstruct(controller_config.experiment)
    experiment_config = controller_config.experiment;
end
% Plant-only mismatch: controllers retain the nominal model snapshot.
global m_L J_L Jx_L Jy_L Jz_L
payload_plant = controller_config.payload_model;
if isfield(experiment_config, 'payload_plant')
    mismatch = experiment_config.payload_plant;
    if isfield(mismatch, 'mass_scale')
        validateattributes(mismatch.mass_scale, {'numeric'}, {'scalar','finite','positive'});
        payload_plant.mass = payload_plant.mass * mismatch.mass_scale;
    end
    if isfield(mismatch, 'inertia_scale')
        validateattributes(mismatch.inertia_scale, {'numeric'}, {'scalar','finite','positive'});
        payload_plant.inertia = payload_plant.inertia * mismatch.inertia_scale;
    end
end
m_L = payload_plant.mass; J_L = payload_plant.inertia;
Jx_L = J_L(1,1); Jy_L = J_L(2,2); Jz_L = J_L(3,3);
% N samples, N-1 zero-order-held integration intervals; no step beyond Tf.
validateattributes(dt, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(Tf, {'numeric'}, {'scalar','real','finite','nonnegative'});
num_intervals = round(Tf / dt);
if abs(num_intervals * dt - Tf) > 1e-10 * max(1, Tf)
    error('TIMC:TimeGrid', 'Tf must be an integer multiple of dt.');
end
t = (0:num_intervals) * dt;
N = numel(t);
trajectory_config = struct();
takeoff_iter = 0;
if ~isempty(fieldnames(experiment_config)) && isfield(experiment_config, 'trajectory') && ...
        isstruct(experiment_config.trajectory)
    trajectory_config = experiment_config.trajectory;
    if isfield(trajectory_config, 'takeoff_duration') && ~isempty(trajectory_config.takeoff_duration)
        if trajectory_config.takeoff_duration ~= 0
            error('TIMC:TakeoffUnsupported', 'This simulation starts directly in transport; takeoff_duration must be zero.');
        end
    end
end

%% ==================== 系统初始状态 ====================
% 初始状态矩阵
% 无人机1初始状态
xPlant(:,1) = [0;   0;   0; 0; 0; 0;
               0.5; 0; 4.2; 0; 0; 0];
% 无人机2初始状态
xPlant(:,2) = [0;   0;   0; 0; 0; 0; 
              -0.5; 0; 4.6; 0; 0; 0];
% 无人机3初始状态
xPlant(:,3) = [0;   0;   0; 0; 0; 0; 
              -0.5; 0; 3.8; 0; 0; 0];
% 货物初始状态
xL = [0; 0;   0; 0;   0; 0; 
      0; 0; 4.2; 0; 0.1; 0];

if ~isempty(fieldnames(experiment_config)) && isfield(experiment_config, 'initial_state') && ...
        isstruct(experiment_config.initial_state)
    init_cfg = experiment_config.initial_state;
    if isfield(init_cfg, 'xPlant') && ~isempty(init_cfg.xPlant)
        xPlant = init_cfg.xPlant;
    else
        if isfield(init_cfg, 'uav_positions') && ~isempty(init_cfg.uav_positions)
            xPlant([7, 9, 11], :) = init_cfg.uav_positions;
        end
        if isfield(init_cfg, 'uav_velocities') && ~isempty(init_cfg.uav_velocities)
            xPlant([8, 10, 12], :) = init_cfg.uav_velocities;
        end
        if isfield(init_cfg, 'uav_attitudes') && ~isempty(init_cfg.uav_attitudes)
            xPlant([1, 3, 5], :) = init_cfg.uav_attitudes;
        end
        if isfield(init_cfg, 'uav_angular_rates') && ~isempty(init_cfg.uav_angular_rates)
            xPlant([2, 4, 6], :) = init_cfg.uav_angular_rates;
        end
    end

    if isfield(init_cfg, 'xL') && ~isempty(init_cfg.xL)
        xL = init_cfg.xL;
    else
        if isfield(init_cfg, 'load_position') && ~isempty(init_cfg.load_position)
            xL([7, 9, 11]) = reshape(init_cfg.load_position, 3, 1);
        end
        if isfield(init_cfg, 'load_velocity') && ~isempty(init_cfg.load_velocity)
            xL([8, 10, 12]) = reshape(init_cfg.load_velocity, 3, 1);
        end
        if isfield(init_cfg, 'load_attitude') && ~isempty(init_cfg.load_attitude)
            xL([1, 3, 5]) = reshape(init_cfg.load_attitude, 3, 1);
        end
        if isfield(init_cfg, 'load_angular_rate') && ~isempty(init_cfg.load_angular_rate)
            xL([2, 4, 6]) = reshape(init_cfg.load_angular_rate, 3, 1);
        end
    end
end

if ~isempty(uav_disturbance_config) && isfield(uav_disturbance_config, 'enabled') && ...
        uav_disturbance_config.enabled && isfield(uav_disturbance_config, 'initial_state') && ...
        isfield(uav_disturbance_config.initial_state, 'enabled') && uav_disturbance_config.initial_state.enabled
    initial_uav_disturbance = uav_disturbance_config.initial_state;
    if ~isfield(initial_uav_disturbance, 'uav_ids') || isempty(initial_uav_disturbance.uav_ids)
        initial_uav_ids = 1:Quadrotor_Num;
    else
        initial_uav_ids = unique(initial_uav_disturbance.uav_ids(:).');
    end

    initial_position = zeros(3, 1);
    initial_velocity = zeros(3, 1);
    initial_attitude = zeros(3, 1);
    initial_angular_rate = zeros(3, 1);
    if isfield(initial_uav_disturbance, 'position') && ~isempty(initial_uav_disturbance.position)
        initial_position = reshape(initial_uav_disturbance.position, 3, 1);
    end
    if isfield(initial_uav_disturbance, 'velocity') && ~isempty(initial_uav_disturbance.velocity)
        initial_velocity = reshape(initial_uav_disturbance.velocity, 3, 1);
    end
    if isfield(initial_uav_disturbance, 'attitude') && ~isempty(initial_uav_disturbance.attitude)
        initial_attitude = reshape(initial_uav_disturbance.attitude, 3, 1);
    end
    if isfield(initial_uav_disturbance, 'angular_rate') && ~isempty(initial_uav_disturbance.angular_rate)
        initial_angular_rate = reshape(initial_uav_disturbance.angular_rate, 3, 1);
    end

    for q_num = initial_uav_ids
        if q_num < 1 || q_num > Quadrotor_Num
            continue;
        end
        xPlant([1, 3, 5], q_num) = xPlant([1, 3, 5], q_num) + initial_attitude;
        xPlant([2, 4, 6], q_num) = xPlant([2, 4, 6], q_num) + initial_angular_rate;
        xPlant([7, 9, 11], q_num) = xPlant([7, 9, 11], q_num) + initial_position;
        xPlant([8, 10, 12], q_num) = xPlant([8, 10, 12], q_num) + initial_velocity;
    end
end

%% ==================== 几何参数 ====================
% 货物质心指向悬挂点的向量（机体坐标系）
rho = [0.5 -0.5 -0.5;
       0.0  0.4 -0.4;
       0.1  0.1  0.1];
% 单位向量
e3 = [0; 0; 1];

%% ==================== 绳索状态初始化 ====================
% 绳索方向向量（当前时刻和上一时刻）
qi = zeros(3, Quadrotor_Num);
for q_num = 1 : Quadrotor_Num
    qi(:, q_num) = -e3;      % 当前时刻初始化为e3
end
% 期望绳索方向
qi_d = zeros(3, Quadrotor_Num);
% 绳索角速度
sOmega = zeros(3, Quadrotor_Num);
sOmega_d = zeros(3, Quadrotor_Num);

%% ==================== 无人机状态初始化 ====================
% 期望角度和角速度
omegaid_p = zeros(3, Quadrotor_Num);
Omegaid = zeros(3, Quadrotor_Num);
% 期望旋转矩阵
Rd_i_now = zeros(3, 3, Quadrotor_Num);

%% ==================== 期望轨迹（货物） ====================
% 直接从运输初态开始，参考从 t=0 生成；没有起飞或轨迹时间重置。
%% ==================== 龙格库塔法中间变量 ====================
% 无人机动力学RK4变量
k1_plant = zeros(12, Quadrotor_Num);
k2_plant = zeros(12, Quadrotor_Num);
k3_plant = zeros(12, Quadrotor_Num);
k4_plant = zeros(12, Quadrotor_Num);

k1_plant_att = zeros(6, Quadrotor_Num);
k2_plant_att = zeros(6, Quadrotor_Num);
k3_plant_att = zeros(6, Quadrotor_Num);
k4_plant_att = zeros(6, Quadrotor_Num);
%% ==================== 数据保存数组 ====================
% 状态保存
X_UAV_save = zeros(N, 12, Quadrotor_Num);
X_L_save = zeros(N, 12);
Us_save = zeros(N,3,Quadrotor_Num);
% 组织所有保存数组到结构体
Save = struct();
Save.Fplant_Save = zeros(N, Quadrotor_Num);
Save.Mplant_Save = zeros(N, 3, Quadrotor_Num);
Save.Us_Save = zeros(N, 3, Quadrotor_Num);
Save.omegaid_p_Save = zeros(N, 3, Quadrotor_Num);
Save.Fin_Save = zeros(N, 3);
Save.Min_Save = zeros(N, 3);
Save.Fd_Save = zeros(N, 3);
Save.Md_Save = zeros(N, 3);
Save.qi_Save = zeros(N, 3, Quadrotor_Num);
Save.Miu_Save = zeros(N, 3, Quadrotor_Num);
Save.Tension_Save = zeros(N, Quadrotor_Num);
Save.a_Save = zeros(N, 3, Quadrotor_Num);
Save.Us_Fin_sub_Save = zeros(N, 3, Quadrotor_Num);
Save.Input_error_all = zeros(N, 3, 2);
Save.Errors_Save = struct();
Save.Errors_Save.e_RL = zeros(N, 3);
Save.Errors_Save.e_Ome_L = zeros(N, 3);
Save.Errors_Save.e_p_L = zeros(N, 3);
Save.Errors_Save.e_v_L = zeros(N, 3);
Save.Errors_Save.e_qi = zeros(N, 3, Quadrotor_Num);
Save.Errors_Save.e_sOmegai = zeros(N, 3, Quadrotor_Num);
Save.Errors_Save.e_Ri = zeros(N, 3, Quadrotor_Num);
Save.Errors_Save.e_Omegai = zeros(N, 3, Quadrotor_Num);
% 新增：姿态误差函数和姿态角跟踪
Save.Errors_Save.error_RL_norm_sq = zeros(N, 1);  % 1/2||RL-RL_d||^2
Save.Errors_Save.error_qi_norm_sq = zeros(N, Quadrotor_Num);  % 1/2||qi - qi_d||^2 (每个无人机)
Save.Errors_Save.Att_L_actual = zeros(N, 3);  % 实际姿态角 [Roll, Pitch, Yaw]
Save.Errors_Save.Att_L_desired = zeros(N, 3);  % 期望姿态角 [Roll, Pitch, Yaw]
Save.Errors_Save.e_Att_L = zeros(N, 3);  % 姿态角跟踪误差 [eRoll, ePitch, eYaw]
% 新增：无人机姿态角跟踪
Save.Errors_Save.UAV_Att_actual = zeros(N, 3, Quadrotor_Num);  % 无人机实际姿态角 [Roll, Pitch, Yaw] x Quadrotor_Num
Save.Errors_Save.UAV_Att_desired = zeros(N, 3, Quadrotor_Num);  % 无人机期望姿态角 [Roll, Pitch, Yaw] x Quadrotor_Num
Save.Errors_Save.UAV_e_Att = zeros(N, 3, Quadrotor_Num);  % 无人机姿态角跟踪误差 [eRoll, ePitch, eYaw] x Quadrotor_Num
Save.Errors_Save.e_p_UAV = zeros(N, 3, Quadrotor_Num);  % 无人机位置跟踪误差 [ex, ey, ez] x Quadrotor_Num

% ESO数据保存（仅当使用RBF控制器时）
if strcmpi(payload_controller_type, 'rbf')
    Save.ESO_Save = struct();
    Save.ESO_Save.z1_pos = zeros(N, 3);      % 估计位置
    Save.ESO_Save.z2_pos = zeros(N, 3);      % 估计速度
    Save.ESO_Save.z3_pos = zeros(N, 3);      % 位置总扰动
    Save.ESO_Save.z1_att = zeros(N, 3);      % 估计姿态
    Save.ESO_Save.z2_att = zeros(N, 3);      % 估计角速度
    Save.ESO_Save.z3_att = zeros(N, 3);      % 姿态总扰动
    Save.ESO_Save.Fd_eso_comp = zeros(N, 3); % ESO补偿力
    Save.ESO_Save.Md_eso_comp = zeros(N, 3); % ESO补偿力矩
end

%% ==================== 动画设置 ====================
animate_fig = [];
animate_step = 10;

N_total = N;
fprintf('总迭代次数：%d\n', N_total);

%% ==================== 生成货物期望轨迹（t从0开始） ====================
t_main = t;   % 长度为N，从t=0开始计时
[DesPosL_main, RL_d_main, DesAttL_main] = load_lifting_desireTrajectory(t_main, dt, trajectory_config);

% 数据保存数组（无起飞阶段，主循环从 t=0 开始）
X_UAV_save_extended = zeros(N_total, 12, Quadrotor_Num);
X_L_save_extended = zeros(N_total, 12);
% 扩展Save结构体数组
Save_extended = Save;
field_names = fieldnames(Save);
for f = 1:length(field_names)
    field_name = field_names{f};
    if isstruct(Save.(field_name))
        % 如果是结构体（如Errors_Save），递归处理
        Save_extended.(field_name) = Save.(field_name);
        % 扩展Errors_Save中的新字段
        if strcmp(field_name, 'Errors_Save')
            % 初始化新字段（无论是否存在都初始化）
            Save_extended.Errors_Save.error_RL_norm_sq = zeros(N_total, 1);
            Save_extended.Errors_Save.error_qi_norm_sq = zeros(N_total, Quadrotor_Num);
            Save_extended.Errors_Save.Att_L_actual = zeros(N_total, 3);
            Save_extended.Errors_Save.Att_L_desired = zeros(N_total, 3);
            Save_extended.Errors_Save.e_Att_L = zeros(N_total, 3);
            Save_extended.Errors_Save.UAV_Att_actual = zeros(N_total, 3, Quadrotor_Num);
            Save_extended.Errors_Save.UAV_Att_desired = zeros(N_total, 3, Quadrotor_Num);
            Save_extended.Errors_Save.UAV_e_Att = zeros(N_total, 3, Quadrotor_Num);
            Save_extended.Errors_Save.e_p_UAV = zeros(N_total, 3, Quadrotor_Num);
        end
        % 扩展ESO_Save结构体（如果存在）
        if strcmp(field_name, 'ESO_Save') && isfield(Save, 'ESO_Save')
            Save_extended.ESO_Save = struct();
            Save_extended.ESO_Save.z1_pos = zeros(N_total, 3);
            Save_extended.ESO_Save.z2_pos = zeros(N_total, 3);
            Save_extended.ESO_Save.z3_pos = zeros(N_total, 3);
            Save_extended.ESO_Save.z1_att = zeros(N_total, 3);
            Save_extended.ESO_Save.z2_att = zeros(N_total, 3);
            Save_extended.ESO_Save.z3_att = zeros(N_total, 3);
            Save_extended.ESO_Save.Fd_eso_comp = zeros(N_total, 3);
            Save_extended.ESO_Save.Md_eso_comp = zeros(N_total, 3);
        end
    else
        % 如果是数组，扩展大小
        current_size = size(Save.(field_name));
        if length(current_size) == 2 && current_size(1) == N
            % 2D数组 (N x ...)
            new_size = [N_total, current_size(2:end)];
            Save_extended.(field_name) = zeros(new_size);
        elseif length(current_size) == 3 && current_size(1) == N
            % 3D数组 (N x ... x ...)
            new_size = [N_total, current_size(2:end)];
            Save_extended.(field_name) = zeros(new_size);
        elseif strcmp(field_name, 'Mplant_Save') && current_size(1) == N + 1
            % 特殊处理 Mplant_Save: (N+1) x 3 x Quadrotor_Num
            new_size = [N_total + 1, current_size(2:end)];
            Save_extended.(field_name) = zeros(new_size);
        else
            % 其他情况，保持原样
            Save_extended.(field_name) = Save.(field_name);
        end
    end
end
% 无人机期望轨迹（来自 Des_Pos_Quadrotor，用于 UAV 位置误差：e = X_UAV - Des_Pos_UAV_Save）
Save_extended.Des_Pos_UAV_Save = zeros(N_total, 3, Quadrotor_Num);
% 无人机外环控制力 F_Plant (3×Quadrotor_Num)，用于绘制各 UAV 的 Fx,Fy,Fz
Save_extended.F_Plant_Save = zeros(N_total, 3, Quadrotor_Num);

% 期望轨迹（从 t=0 开始）
DesPosL_extended = DesPosL_main;
DesAttL_extended = DesAttL_main;
RL_d_extended = RL_d_main;

%% ==================== 控制器相关变量初始化 ====================
% RBF 控制器参数初始化（如果使用 RBF 控制器）
if strcmpi(payload_controller_type, 'rbf')
    % 从统一参数配置文件加载RBF参数
    rbf_config = controller_config.rbf;
    
    % ==================== RBF参数加载选项 ====================
    % 是否从Excel文件加载上次训练的RBF参数（从配置文件读取）
    load_rbf_params = rbf_config.load_rbf_params;
    rbf_params_filename = rbf_config.rbf_params_filename;
    
    if load_rbf_params
        % 尝试从Excel文件加载RBF参数（从多个sheet读取）
        % 使用英文sheet名称以提高兼容性
        sheet_pos = 'Position';  % 位置控制sheet名称
        sheet_att = 'Attitude';  % 姿态控制sheet名称
        
        try
            if exist(rbf_params_filename, 'file')
                pos_data = [];
                att_data = [];
                
                % 方法1：尝试通过sheet名称读取
                try
                    % 从"Position"sheet读取前15个参数
                    try
                        pos_data = readmatrix(rbf_params_filename, 'Sheet', sheet_pos, 'Range', 'B2:B16');
                    catch
                        pos_cell = readcell(rbf_params_filename, 'Sheet', sheet_pos);
                        if size(pos_cell, 1) >= 16
                            pos_data = cell2mat(pos_cell(2:16, 2));
                        end
                    end
                    
                    % 从"Attitude"sheet读取后15个参数
                    try
                        att_data = readmatrix(rbf_params_filename, 'Sheet', sheet_att, 'Range', 'B2:B16');
                    catch
                        att_cell = readcell(rbf_params_filename, 'Sheet', sheet_att);
                        if size(att_cell, 1) >= 16
                            att_data = cell2mat(att_cell(2:16, 2));
                        end
                    end
                catch
                    % 方法2：如果sheet名称失败，尝试使用sheet索引（1和2）
                    try
                        % 尝试从第1个sheet读取位置参数
                        try
                            pos_data = readmatrix(rbf_params_filename, 'Sheet', 1, 'Range', 'B2:B16');
                        catch
                            pos_cell = readcell(rbf_params_filename, 'Sheet', 1);
                            if size(pos_cell, 1) >= 16
                                pos_data = cell2mat(pos_cell(2:16, 2));
                            end
                        end
                        
                        % 尝试从第2个sheet读取姿态参数
                        try
                            att_data = readmatrix(rbf_params_filename, 'Sheet', 2, 'Range', 'B2:B16');
                        catch
                            att_cell = readcell(rbf_params_filename, 'Sheet', 2);
                            if size(att_cell, 1) >= 16
                                att_data = cell2mat(att_cell(2:16, 2));
                            end
                        end
                    catch
                        % 方法3：如果只有1个sheet，尝试读取全部30个参数
                        try
                            all_data = readmatrix(rbf_params_filename, 'Sheet', 1, 'Range', 'B2:B31');
                            if ~isempty(all_data) && length(all_data) == 30
                                pos_data = all_data(1:15);
                                att_data = all_data(16:30);
                            end
                        catch
                            all_cell = readcell(rbf_params_filename, 'Sheet', 1);
                            if size(all_cell, 1) >= 31
                                all_data = cell2mat(all_cell(2:31, 2));
                                if length(all_data) == 30
                                    pos_data = all_data(1:15);
                                    att_data = all_data(16:30);
                                end
                            end
                        end
                    end
                end
                
                % 检查数据有效性并合并
                if ~isempty(pos_data) && length(pos_data) == 15 && ...
                   ~isempty(att_data) && length(att_data) == 15
                    theta_rbf = [pos_data(:); att_data(:)];  % 合并为30x1列向量
                    fprintf('成功从 %s 加载RBF参数\n', rbf_params_filename);
                    fprintf('  - 位置控制参数: %d个\n', length(pos_data));
                    fprintf('  - 姿态控制参数: %d个\n', length(att_data));
                else
                    fprintf('RBF参数文件格式不正确或数据不完整（位置控制:%d个，姿态控制:%d个），使用配置文件中的默认初值\n', ...
                        length(pos_data), length(att_data));
                    theta_rbf = rbf_config.theta_rbf_init;
                end
            else
                fprintf('RBF参数文件 %s 不存在，使用配置文件中的默认初值\n', rbf_params_filename);
                theta_rbf = rbf_config.theta_rbf_init;
            end
        catch ME
            warning('加载RBF参数时出错: %s，使用配置文件中的默认初值', ME.message);
            theta_rbf = rbf_config.theta_rbf_init;
        end
    else
        % 使用配置文件中的默认初值
        theta_rbf = rbf_config.theta_rbf_init;  % RBF 参数初值（30x1）
    end
    
else
    theta_rbf = [];  % 非RBF时为空
end

%% ==================== ESO状态初始化 ====================
% 如果是RBF控制器，初始化ESO状态变量
if strcmpi(payload_controller_type, 'rbf')
    % 初始化ESO状态（18x1）：[z1_pos(3); z2_pos(3); z3_pos(3); z1_att(3); z2_att(3); z3_att(3)]
    eso_state_prev = zeros(18, 1);
    % 使用初始负载状态初始化ESO
    PosL_init = [xL(7); xL(9); xL(11)];       % 初始位置
    VelL_init = [xL(8); xL(10); xL(12)];      % 初始速度
    AttL_init = [xL(1); xL(3); xL(5)];        % 初始姿态
    OmeL_init = [xL(2); xL(4); xL(6)];       % 初始角速度
    
    eso_state_prev(1:3) = PosL_init;      % z1_pos: 估计位置
    eso_state_prev(4:6) = VelL_init;      % z2_pos: 估计速度
    eso_state_prev(7:9) = zeros(3,1);     % z3_pos: 估计位置总扰动（初始为0）
    eso_state_prev(10:12) = AttL_init;    % z1_att: 估计姿态
    eso_state_prev(13:15) = OmeL_init;    % z2_att: 估计角速度
    eso_state_prev(16:18) = zeros(3,1);   % z3_att: 估计姿态总扰动（初始为0）
    
    % 初始化上一时刻的控制输入（用于ESO更新）
    u_prev = zeros(6, 1);  % [Fd_prev(3); Md_prev(3)]
else
    eso_state_prev = [];
    u_prev = [];
end

%% ==================== 打印当前控制器配置信息 ====================
fprintf('\n');
fprintf('========================================\n');
fprintf('        控制器配置信息\n');
fprintf('========================================\n');
fprintf('控制器类型: %s\n', controller_config.controller_type);
if with_disturbance
    fprintf('干扰状态: 已启用 (with_disturbance = true)\n');
else
    fprintf('干扰状态: 未启用 (with_disturbance = false)\n');
end
if isfield(controller_config, 'uav_control')
    if isfield(controller_config.uav_control, 'uav_F_type')
        fprintf('无人机推力控制器: %s\n', controller_config.uav_control.uav_F_type);
    end
    if isfield(controller_config.uav_control, 'uav_M_type')
        fprintf('无人机力矩控制器: %s\n', controller_config.uav_control.uav_M_type);
    end
end
uav_disturbance_enabled = ~isempty(uav_disturbance_config) && isfield(uav_disturbance_config, 'enabled') && ...
    uav_disturbance_config.enabled;
uav_initial_state_enabled = uav_disturbance_enabled && isfield(uav_disturbance_config, 'initial_state') && ...
    isfield(uav_disturbance_config.initial_state, 'enabled') && uav_disturbance_config.initial_state.enabled;
uav_schedule_count = 0;
if uav_disturbance_enabled && isfield(uav_disturbance_config, 'schedule') && ~isempty(uav_disturbance_config.schedule)
    uav_schedule_count = sum(arrayfun(@(item) ~isfield(item, 'enabled') || item.enabled, uav_disturbance_config.schedule));
end
if uav_disturbance_enabled
    fprintf('无人机扰动: 已启用 (初始状态=%d, 时变扰动段=%d)\n', uav_initial_state_enabled, uav_schedule_count);
else
    fprintf('无人机扰动: 未启用\n');
end
fprintf('========================================\n');
fprintf('所有控制器参数统一配置在: load_lifting_controller_params.m\n');
fprintf('如需调整参数，请修改该文件中的对应控制器配置\n');
fprintf('========================================\n\n');

% 上一时刻货物状态：初始化为初始状态，避免第1步求导/差分出现空
x_L_prev = xL;

% 主循环：运行N次迭代，从 t=0 开始
for i = 1:N
    save_idx = i;
    
    % % 调试断点：当 save_idx 等于 2731 时自动暂停
    % if save_idx == 2373
    %     fprintf('\n========================================\n');
    %     fprintf('调试断点触发：save_idx = %d\n', save_idx);
    %     fprintf('当前迭代次数 i = %d\n', i);
    %     fprintf('起飞阶段迭代次数 takeoff_iter = %d\n', takeoff_iter);
    %     fprintf('========================================\n');
    %     fprintf('程序已暂停，进入调试模式...\n');
    %     fprintf('可以使用以下命令检查变量：\n');
    %     fprintf('  - whos          : 查看所有变量\n');
    %     fprintf('  - disp(xL)      : 查看货物状态\n');
    %     fprintf('  - disp(xPlant) : 查看无人机状态\n');
    %     fprintf('输入 dbcont 或按 F5 继续执行\n');
    %     fprintf('========================================\n\n');
    %     keyboard;  % 暂停并进入调试模式
    % end
    
    %% 提取状态变量
    Att_L = [xL(1); xL(3); xL(5)];        % PHI（roll） THETA（pitch） PSI（yaw）
    Ome_L = [xL(2); xL(4); xL(6)];        % PHI_dot THETA_dot PSI_dot
    RL_cur = eul2rotm_ZYX(Att_L);
    if i == 1
        RL_prev = RL_cur;
    end
    %% ========== 第一部分：计算货物的控制力和控制力矩 ==========
    % 主循环严格跟踪从 t_main=0 开始生成的轨迹（第i步对应t_main(i)=(i-1)*dt）
    % 通过接口函数调用控制器，支持多种控制器类型
    params = struct();
    params.x_L = xL;
    params.Des_Att = DesAttL_main(:,:,i);
    params.Des_pL = DesPosL_main(:,:,i);
    params.RL_d = RL_d_main(:,:,i);
    params.x_L_prev = x_L_prev;  % 上一时刻状态（可选）
    params.with_disturbance = with_disturbance;  % 是否包含干扰（可根据需要修改）
    params.controller_config = controller_config;  % 传递控制器配置

    % 如果是 RBF 控制器，添加额外参数
    if strcmpi(payload_controller_type, 'rbf')
        params.theta_rbf_prev = theta_rbf;  % 上一时刻的 RBF 参数
        params.dt = dt;  % 时间步长
        params.eso_state_prev = eso_state_prev;  % 上一时刻的ESO状态
        params.u_prev = u_prev;  % 上一时刻的控制输入
        
        % 调用接口函数，获取 RBF 和 ESO 相关输出
        % 输出顺序：Fd, Md, errors_controller, theta_rbf, fn, fn_att, h_rbf_all, eso_state
        [Fd, Md, errors_controller, theta_rbf, ~, ~, ~, eso_state] = ...
            load_lifting_payload_control_interface(payload_controller_type, params);
        
        % 更新ESO状态和控制输入（用于下一次迭代）
        eso_state_prev = eso_state;
        u_prev = [Fd; Md];  % 保存当前控制输入，用于下一次ESO更新
    else
        % 经典控制器或其他控制器
        [Fd, Md, errors_controller, RL] = ...
            load_lifting_payload_control_interface(payload_controller_type, params);
    end
    
    % 从接口返回的 errors 结构体中提取误差
    e_RL = errors_controller.e_RL;
    e_OmeL = errors_controller.e_Ome_L;
    e_PosL = errors_controller.e_p_L;
    e_VelL = errors_controller.e_v_L;
    
    % 更新上一时刻状态（用于下一次迭代）
    x_L_prev = xL;
    
    %% ========== 第二部分：解算模块 - 将控制力和控制力矩分配到各无人机 ==========
    [Fin, Min, miui, qi_d_now, sOmegaid_now, e_qi, e_sOmegai, a, Us, sOmega_dot, input_error,miu_i_d] = ...
        load_lifting_allocation(Fd, Md, RL_cur, rho, qi, sOmega, sOmega_d, qi_d, Ome_L, controller_config);
    
    %% ========== 第三部分：计算无人机的控制力和控制力矩 ==========
    [Fplant, Mplant, angled_p, Omegaid_now, e_Ri_all, e_Omegai_all, Rd_i_now, R_i, x_att] = ...
        load_lifting_UAV_control(Us, xPlant, omegaid_p, Omegaid, Rd_i_now, controller_config, t(i));
    
    Us_save(i,:,:) = Us;
    %% 收集所有误差用于保存和绘图
    errors = struct();
    % 货物误差
    errors.e_RL = e_RL;              % 旋转矩阵误差 (3x1)
    errors.e_Ome_L = e_OmeL;        % 物体角速度误差 (3x1)
    errors.e_p_L = e_PosL;            % 位置误差 (3x1)
    errors.e_v_L = e_VelL;            % 速度误差 (3x1)
    % 绳索误差（每个无人机）
    errors.e_qi = e_qi;              % 绳索方向误差 (3xQuadrotor_Num)
    errors.e_sOmegai = e_sOmegai;    % 绳索角速度误差 (3xQuadrotor_Num)
    % 无人机误差（每个无人机）
    errors.e_Ri = e_Ri_all;          % 无人机姿态旋转矩阵误差 (3xQuadrotor_Num)
    errors.e_Omegai = e_Omegai_all; % 无人机角速度误差 (3xQuadrotor_Num)
    % 输入误差（已在input_error中保存）
    errors.input_error = input_error;  % (3x2): [Fd-F_in, Md-M_in)

    % 计算额外的误差函数
    % 1. 姿态误差函数：1/2||RL-RL_d||^2
    RL_diff = RL_cur - RL_d_main(:,:,i);
    error_RL_norm_sq = 0.5 * norm(RL_diff, 'fro')^2;  % Frobenius范数的平方
    
    % 2. 绳索姿态误差函数：1/2||qi - qi_d||^2 (每个无人机)
    error_qi_norm_sq = zeros(Quadrotor_Num, 1);
    for q_num = 1:Quadrotor_Num
        qi_diff = qi(:, q_num) - qi_d_now(:, q_num);
        error_qi_norm_sq(q_num) = 0.5 * norm(qi_diff)^2;
    end
    
    % 3. 姿态角跟踪误差
    Att_L_desired = DesAttL_main(:, 1, i);  % 期望姿态角 [Roll, Pitch, Yaw]
    e_Att_L = Att_L - Att_L_desired;  % 姿态角误差
    
    % 统一保存数据（使用辅助函数，保存到扩展后的Save_extended）
    Save_extended = save_simulation_data(save_idx, Fin, Min, Fplant, Mplant, Fd, Md, a, Us, miui, ...
                                input_error, angled_p, qi, errors, Save_extended);
    
    % 保存额外的误差函数和姿态角数据
    Save_extended.Errors_Save.error_RL_norm_sq(save_idx) = error_RL_norm_sq;
    Save_extended.Errors_Save.error_qi_norm_sq(save_idx, :) = error_qi_norm_sq';
    Save_extended.Errors_Save.Att_L_actual(save_idx, :) = Att_L';
    Save_extended.Errors_Save.Att_L_desired(save_idx, :) = Att_L_desired';
    Save_extended.Errors_Save.e_Att_L(save_idx, :) = e_Att_L';
    
    % 保存ESO数据（仅当使用RBF控制器时）
    if strcmpi(payload_controller_type, 'rbf') && isfield(Save_extended, 'ESO_Save')
        % 提取ESO状态
        z1_pos = eso_state(1:3);
        z2_pos = eso_state(4:6);
        z3_pos = eso_state(7:9);
        z1_att = eso_state(10:12);
        z2_att = eso_state(13:15);
        z3_att = eso_state(16:18);
        
        % 计算ESO补偿项
        global m_L
        Fd_eso_comp = controller_config.payload_model.mass * z3_pos;  % nominal mass, not true plant mass
        Md_eso_comp = z3_att;         % ESO姿态补偿力矩
        
        % 保存ESO数据
        Save_extended.ESO_Save.z1_pos(save_idx, :) = z1_pos';
        Save_extended.ESO_Save.z2_pos(save_idx, :) = z2_pos';
        Save_extended.ESO_Save.z3_pos(save_idx, :) = z3_pos';
        Save_extended.ESO_Save.z1_att(save_idx, :) = z1_att';
        Save_extended.ESO_Save.z2_att(save_idx, :) = z2_att';
        Save_extended.ESO_Save.z3_att(save_idx, :) = z3_att';
        Save_extended.ESO_Save.Fd_eso_comp(save_idx, :) = Fd_eso_comp';
        Save_extended.ESO_Save.Md_eso_comp(save_idx, :) = Md_eso_comp';
    end
    
    % 保存 Rd_i_now（姿态跟踪与误差在动力学更新后统一保存，见主循环末尾）
    for q_num = 1:Quadrotor_Num
        UAV_Rd_i = Rd_i_now(:,:,q_num);
        Save_extended.Rd_i_now_Save(save_idx, :, :, q_num) = UAV_Rd_i;
        Save.R_i_Save(i, :, :, q_num) = R_i(:, :, q_num);
        Save.angled_p_Save(i, :, q_num) = angled_p(:, q_num);
        Save.x_att_Save(i, :, q_num) = x_att(:, q_num);
        
        % 计算姿态角跟踪误差（实际 - 期望）
        Save.eRoll_Save(i, q_num) = x_att(1, q_num) - angled_p(1, q_num);
        Save.ePitch_Save(i, q_num) = x_att(2, q_num) - angled_p(2, q_num);
        Save.eYaw_Save(i, q_num) = x_att(3, q_num) - angled_p(3, q_num);
    

    end

    % 计算 Us_Fin_sub（用于无人机动力学计算）
    Us_Fin_sub = Us;  % (3 x Quadrotor_Num)

    PosL = xL([7,9,11]);
    q_now_dot = zeros(3, Quadrotor_Num);
    for q_num = 1:Quadrotor_Num
        q_now_dot(:,q_num) = cross(sOmega(:,q_num), qi(:,q_num));
    end
    %% 几何关系求解
    x_plant = zeros(3, Quadrotor_Num);
    pU_dot = zeros(3, Quadrotor_Num);
    % 货物连接点速度：直接使用货物状态里的线速度（避免差分噪声）
    PosL_dot = [xL(8); xL(10); xL(12)];
    % Exact body-rate identity at the logged state: R_dot = R*skew(omega).
    RL_dot = RL_cur * skew(Ome_L);
    Des_Pos_Quadrotor = zeros(3, 2, Quadrotor_Num);
    F_Plant = zeros(3,Quadrotor_Num);
    %  无人机期望位置与力控制（按 uav_control.uav_F_type 选择 PD / 反步 / SMC / RISE）
    uav_F_type = 'rise';
    if isfield(controller_config, 'uav_control') && isfield(controller_config.uav_control, 'uav_F_type')
        uav_F_type = lower(controller_config.uav_control.uav_F_type);
    end
    for q_num = 1 : Quadrotor_Num
        % 先计算右侧向量（假设结果为3×1）
        x_plant(:, q_num) = PosL + RL_cur*rho(:, q_num) - l_plant(q_num) * qi(:,q_num);
        Des_Pos_Quadrotor(:, 1, q_num) = x_plant(:, q_num);
        % UAV 速度用一致的解析式（避免差分放大 qi 的高频抖动）
        pU_dot = PosL_dot + RL_dot * rho(:, q_num) - l_plant(q_num) * q_now_dot(:, q_num);
        Des_Pos_Quadrotor(:, 2, q_num) = pU_dot;
        switch uav_F_type
            case 'pd'
                F_Plant(:, q_num) = load_lifting_UAV_control_pd_F(xPlant(:, q_num), Des_Pos_Quadrotor(:, :, q_num), controller_config, q_num);
            case 'backstepping'
                F_Plant(:, q_num) = load_lifting_UAV_control_backstepping_F(xPlant(:, q_num), Des_Pos_Quadrotor(:, :, q_num), controller_config, q_num);
            case 'smc'
                F_Plant(:, q_num) = load_lifting_UAV_control_smc_F(xPlant(:, q_num), Des_Pos_Quadrotor(:, :, q_num), controller_config, q_num);
            otherwise
                F_Plant(:, q_num) = load_lifting_UAV_control_RISE_F(xPlant(:, q_num), Des_Pos_Quadrotor(:, :, q_num), controller_config, q_num);
        end
    end
    RL_prev = RL_cur;

    % 保存无人机期望轨迹 Des_Pos_Quadrotor（用于 UAV 位置误差：e = X_UAV - Des_Pos_UAV_Save）
    for q_num = 1:Quadrotor_Num
        Save_extended.Des_Pos_UAV_Save(save_idx, :, q_num) = Des_Pos_Quadrotor(:, 1, q_num)';
    end
    % 保存无人机外环控制力 F_Plant（用于绘制各 UAV 的 Fx,Fy,Fz）
    for q_num = 1:Quadrotor_Num
        Save_extended.F_Plant_Save(save_idx, :, q_num) = F_Plant(:, q_num)';
    end

    save_idx = i;
    X_UAV_save_extended(save_idx,:,:) = xPlant;
    X_L_save_extended(save_idx,:) = xL;

    % 无人机轨迹跟踪误差 e_p_UAV = X_UAV - Des_Pos_UAV（基于更新后的 xPlant）
    for q_num = 1:Quadrotor_Num
        pos_actual = xPlant([7,9,11], q_num);  % x,y,z
        pos_desired = Des_Pos_Quadrotor(:, 1, q_num);
        Save_extended.Errors_Save.e_p_UAV(save_idx, :, q_num) = (pos_actual - pos_desired)';
    end

    % 无人机姿态跟踪：用更新后的 xPlant 提取实际角度，与期望角度 angled_p 计算误差
    for q_num = 1:Quadrotor_Num
        UAV_Att_actual = xPlant([1,3,5], q_num);  % roll, pitch, yaw
        UAV_Att_desired = angled_p(:, q_num);
        UAV_e_Att = UAV_Att_actual - UAV_Att_desired;
        Save_extended.Errors_Save.UAV_Att_actual(save_idx, :, q_num) = UAV_Att_actual';
        Save_extended.Errors_Save.UAV_Att_desired(save_idx, :, q_num) = UAV_Att_desired';
        Save_extended.Errors_Save.UAV_e_Att(save_idx, :, q_num) = UAV_e_Att';
    end

    % All logs above are x_k/ref_k/u_k at t_k. Advance only N-1 intervals.
    if i < N
    %% 更新绳索角速度（在第三部分计算完成后）
    % 修复：qi是2D数组，不是3D
    q_now = qi;
    q_now_dot = zeros(3, Quadrotor_Num);
    for q_num = 1 : Quadrotor_Num
        q_now_dot(:, q_num) = cross(sOmega(:, q_num), q_now(:, q_num));
        % 计算完成之后，更新绳索sOmega
        sOmega(:,q_num) = sOmega(:,q_num) + dt * sOmega_dot(:,q_num);
    end

    %% 用于保存上一次的结果用来求导
    % 更新下一次的q_i
    qi_d = qi_d_now;
    qi = qi + dt * q_now_dot;
    % 平滑更新qi：结合积分更新和期望方向的平滑过渡
    % 方法：归一化 + 平滑过渡到期望方向（使用SLERP球面线性插值）
    alpha_smooth = 0.15;  % 平滑系数，值越小越平滑（0-1之间），可根据需要调整
    for q_num = 1 : Quadrotor_Num
        % 先进行积分更新
        qi_new = qi(:, q_num) + dt * q_now_dot(:, q_num);
        % 归一化，确保是单位向量
        qi_new = qi_new / (norm(qi_new) + 1e-10);  % 添加小值避免除零

        % 平滑过渡到期望方向（避免突然跳变）
        % 计算当前qi_new和期望qi_d_now之间的角度
        qi_d_normalized = qi_d_now(:, q_num) / (norm(qi_d_now(:, q_num)) + 1e-10);
        dot_product = qi_new' * qi_d_normalized;
        % 限制dot_product在[-1, 1]范围内，避免数值误差
        dot_product = max(-1, min(1, dot_product));
        angle_diff = acos(dot_product);

        % 如果角度差异较大，使用球面线性插值（SLERP）进行平滑过渡
        if angle_diff > 1e-6  % 避免除零
            % 使用SLERP进行平滑插值
            % t_smooth控制插值比例：角度差异越大，插值比例越小（更保守）
            t_smooth = min(1, alpha_smooth * dt / (angle_diff + 1e-6));  % 基于时间步长和角度差异的自适应平滑
            % SLERP公式：在单位球面上进行线性插值
            sin_angle = sin(angle_diff);
            if sin_angle > 1e-6
                qi(:, q_num) = (sin((1-t_smooth)*angle_diff) / sin_angle) * qi_new + ...
                              (sin(t_smooth*angle_diff) / sin_angle) * qi_d_normalized;
            else
                % sin(angle_diff)太小，直接线性插值
                qi(:, q_num) = (1-t_smooth) * qi_new + t_smooth * qi_d_normalized;
            end
        else
            % 角度差异很小，直接使用积分结果
            qi(:, q_num) = qi_new;
        end

        % 最终归一化，确保数值精度
        qi(:, q_num) = qi(:, q_num) / (norm(qi(:, q_num)) + 1e-10);
    end
    % 更新sOmega_d
    sOmega_d = sOmegaid_now;
    % 更新omegaid
    omegaid_p = angled_p;
    Omegaid = Omegaid_now;
    % 传入到货物中
    % 计算当前时间（主循环从t=0开始，第i步对应时间 t_main(i) = (i-1)*dt）
    t_curr = t_main(i);
    k1_o = load_lifting_payload_dynamics(xL, Fin, Min, t_curr, with_disturbance);
    tmpXL1 = xL + dt/2*k1_o;
    tmpt1 = t_curr + dt/2;
    k2_o = load_lifting_payload_dynamics(tmpXL1, Fin, Min, tmpt1, with_disturbance);
    k3_o = load_lifting_payload_dynamics(xL + dt/2*k2_o, Fin, Min, t_curr + dt/2, with_disturbance);
    k4_o = load_lifting_payload_dynamics(xL + dt * k3_o, Fin, Min, t_curr + dt, with_disturbance);
    xL = xL + dt/6 *(k1_o + 2*k2_o + 2*k3_o + k4_o);
    % 货物当前位置
    PosL = [xL(7); xL(9); xL(11)];

    % 传入到无人机和绳索中
    for q_num = 1 : Quadrotor_Num
        % 无人机
        k1_plant(:, q_num) = load_lifting_UAV_dynamics(xPlant(:,q_num), F_Plant(:,q_num), Mplant(:,q_num), ...
            t_curr, uav_disturbance_config, q_num);
        k2_plant(:, q_num) = load_lifting_UAV_dynamics(xPlant(:,q_num) + dt/2 * k1_plant(:, q_num), F_Plant(:,q_num), Mplant(:,q_num), ...
            t_curr + dt/2, uav_disturbance_config, q_num);
        k3_plant(:, q_num) = load_lifting_UAV_dynamics(xPlant(:,q_num) + dt/2 * k2_plant(:, q_num), F_Plant(:,q_num), Mplant(:,q_num), ...
            t_curr + dt/2, uav_disturbance_config, q_num);
        k4_plant(:, q_num) = load_lifting_UAV_dynamics(xPlant(:,q_num) + dt   * k3_plant(:, q_num), F_Plant(:,q_num), Mplant(:,q_num), ...
            t_curr + dt, uav_disturbance_config, q_num);
        xPlant(:,q_num) = xPlant(:,q_num) + dt/6 * (k1_plant(:, q_num) + 2 * k2_plant(:, q_num) + 2 * k3_plant(:, q_num) + k4_plant(:, q_num));
    end
    end

    % % 实时动画显示（每animate_step步更新一次）
    % if mod(i, animate_step) == 0 || i == 1
    %     animate_fig = animate_system(xL, xPlant, rho, Quadrotor_Num, animate_fig);
    %     % 显示当前时间
    %     if isvalid(animate_fig)
    %         axes_handle = gca(animate_fig);
    %         title(axes_handle, sprintf('无人机吊运系统实时动画 - 时间: %.2f s', t(i)));
    %     end
    % end

end

%% 时间向量（从0开始）
t_extended = t;  % t 已从 0 开始

%% 限制绘图时间范围到85秒
plot_time_limit = 85;  % 绘图时间限制（秒）
plot_idx_limit = min(floor(plot_time_limit / dt) + 1, N_total);  % 计算对应的索引
t_plot = t_extended(1:plot_idx_limit);  % 截取时间向量

% 截取数据到85秒
X_UAV_plot = X_UAV_save_extended(1:plot_idx_limit, :, :);
X_L_plot = X_L_save_extended(1:plot_idx_limit, :);
Des_pL_plot = DesPosL_extended(:, :, 1:plot_idx_limit);
Des_Att_plot = DesAttL_extended(:, :, 1:plot_idx_limit);

% 截取Save结构体中的数据
Save_plot = Save_extended;
field_names = fieldnames(Save_extended);
for f = 1:length(field_names)
    field_name = field_names{f};
    if isstruct(Save_extended.(field_name))
        % 如果是结构体（如Errors_Save），递归处理
        Save_plot.(field_name) = Save_extended.(field_name);
        if isfield(Save_extended.(field_name), 'e_RL')
            Save_plot.(field_name).e_RL = Save_extended.(field_name).e_RL(1:plot_idx_limit, :);
        end
        if isfield(Save_extended.(field_name), 'e_Ome_L')
            Save_plot.(field_name).e_Ome_L = Save_extended.(field_name).e_Ome_L(1:plot_idx_limit, :);
        end
        if isfield(Save_extended.(field_name), 'e_p_L')
            Save_plot.(field_name).e_p_L = Save_extended.(field_name).e_p_L(1:plot_idx_limit, :);
        end
        if isfield(Save_extended.(field_name), 'e_v_L')
            Save_plot.(field_name).e_v_L = Save_extended.(field_name).e_v_L(1:plot_idx_limit, :);
        end
        if isfield(Save_extended.(field_name), 'e_qi')
            Save_plot.(field_name).e_qi = Save_extended.(field_name).e_qi(1:plot_idx_limit, :, :);
        end
        if isfield(Save_extended.(field_name), 'e_sOmegai')
            Save_plot.(field_name).e_sOmegai = Save_extended.(field_name).e_sOmegai(1:plot_idx_limit, :, :);
        end
        if isfield(Save_extended.(field_name), 'e_Ri')
            Save_plot.(field_name).e_Ri = Save_extended.(field_name).e_Ri(1:plot_idx_limit, :, :);
        end
        if isfield(Save_extended.(field_name), 'e_Omegai')
            Save_plot.(field_name).e_Omegai = Save_extended.(field_name).e_Omegai(1:plot_idx_limit, :, :);
        end
    else
        % 如果是数组，截取前plot_idx_limit行
        current_data = Save_extended.(field_name);
        if size(current_data, 1) == N_total
            if ndims(current_data) == 2
                Save_plot.(field_name) = current_data(1:plot_idx_limit, :);
            elseif ndims(current_data) == 3
                Save_plot.(field_name) = current_data(1:plot_idx_limit, :, :);
            else
                Save_plot.(field_name) = current_data;
            end
        else
            Save_plot.(field_name) = current_data;
        end
    end
end

%% 绘图
% 组织绘图数据（简化版本：直接传入Save结构体，使用截取到85秒的数据）
plotData = create_plot_data(t_plot, plot_idx_limit, Quadrotor_Num, X_UAV_plot, X_L_plot, Des_pL_plot, Des_Att_plot, Save_plot);

% 主函数中的历史默认绘图先全部关闭。
% 如需查看扰动后的无人机位置和姿态变化，见后面的专用绘图部分。


%% ==================== 保存RBF参数到Excel ====================
% 如果使用RBF控制器，保存训练后的RBF参数
if strcmpi(payload_controller_type, 'rbf') && ~isempty(theta_rbf)
    % RBF参数文件名（与加载时使用相同的文件名）
    rbf_params_filename = rbf_config.rbf_params_filename;
    
    try
        % 准备数据：theta_rbf是30x1的列向量
        % 前15个参数：位置控制（X, Y, Z各5个RBF节点）
        % 后15个参数：姿态控制（Roll, Pitch, Yaw各5个RBF节点）
        
        % ==================== Sheet 1: 位置控制参数 ====================
        pos_labels = cell(15, 1);
        pos_labels{1} = 'X方向_RBF节点1';
        pos_labels{2} = 'X方向_RBF节点2';
        pos_labels{3} = 'X方向_RBF节点3';
        pos_labels{4} = 'X方向_RBF节点4';
        pos_labels{5} = 'X方向_RBF节点5';
        pos_labels{6} = 'Y方向_RBF节点1';
        pos_labels{7} = 'Y方向_RBF节点2';
        pos_labels{8} = 'Y方向_RBF节点3';
        pos_labels{9} = 'Y方向_RBF节点4';
        pos_labels{10} = 'Y方向_RBF节点5';
        pos_labels{11} = 'Z方向_RBF节点1';
        pos_labels{12} = 'Z方向_RBF节点2';
        pos_labels{13} = 'Z方向_RBF节点3';
        pos_labels{14} = 'Z方向_RBF节点4';
        pos_labels{15} = 'Z方向_RBF节点5';
        
        % 位置控制参数（前15个）
        pos_params = theta_rbf(1:15);
        pos_data_table = [pos_labels, num2cell(pos_params)];
        pos_header = {'参数名称', '参数值'};
        pos_data_with_header = [pos_header; pos_data_table];
        
        % 保存位置控制参数到第一个sheet（使用英文名称以提高兼容性）
        writecell(pos_data_with_header, rbf_params_filename, 'Sheet', 'Position');
        
        % ==================== Sheet 2: 姿态控制参数 ====================
        att_labels = cell(15, 1);
        att_labels{1} = 'Roll方向_RBF节点1';
        att_labels{2} = 'Roll方向_RBF节点2';
        att_labels{3} = 'Roll方向_RBF节点3';
        att_labels{4} = 'Roll方向_RBF节点4';
        att_labels{5} = 'Roll方向_RBF节点5';
        att_labels{6} = 'Pitch方向_RBF节点1';
        att_labels{7} = 'Pitch方向_RBF节点2';
        att_labels{8} = 'Pitch方向_RBF节点3';
        att_labels{9} = 'Pitch方向_RBF节点4';
        att_labels{10} = 'Pitch方向_RBF节点5';
        att_labels{11} = 'Yaw方向_RBF节点1';
        att_labels{12} = 'Yaw方向_RBF节点2';
        att_labels{13} = 'Yaw方向_RBF节点3';
        att_labels{14} = 'Yaw方向_RBF节点4';
        att_labels{15} = 'Yaw方向_RBF节点5';
        
        % 姿态控制参数（后15个）
        att_params = theta_rbf(16:30);
        att_data_table = [att_labels, num2cell(att_params)];
        att_header = {'参数名称', '参数值'};
        att_data_with_header = [att_header; att_data_table];
        
        % 保存姿态控制参数到第二个sheet（使用英文名称以提高兼容性）
        writecell(att_data_with_header, rbf_params_filename, 'Sheet', 'Attitude');
        
        fprintf('RBF参数已保存到Excel文件: %s\n', rbf_params_filename);
        fprintf('  - Sheet "Position": 15个参数（X, Y, Z各5个节点）\n');
        fprintf('  - Sheet "Attitude": 15个参数（Roll, Pitch, Yaw各5个节点）\n');
    catch ME
        warning('保存RBF参数时出错: %s', ME.message);
    end
end

%% ==================== 计算稳态指标并保存到Excel ====================
% 计算稳态均方根误差和最大误差
fprintf('\n开始计算稳态指标...\n');

% 定义稳态起始时间（例如仿真时间的后80%）
steady_start_idx = max(1, floor(0.2 * N_total));  % 从20%开始计算稳态
steady_indices = steady_start_idx:N_total;

% 提取稳态数据
e_PosL_steady = Save_extended.Errors_Save.e_p_L(steady_indices, :);  % 位置误差
e_RL_steady = Save_extended.Errors_Save.e_RL(steady_indices, :);  % 旋转矩阵误差
e_Att_L_steady = Save_extended.Errors_Save.e_Att_L(steady_indices, :);  % 姿态角误差

% 计算稳态均方根误差（RMS）
delta_x = sqrt(mean(e_PosL_steady(:, 1).^2));
delta_y = sqrt(mean(e_PosL_steady(:, 2).^2));
delta_z = sqrt(mean(e_PosL_steady(:, 3).^2));
delta_RL_1 = sqrt(mean(e_RL_steady(:, 1).^2));
delta_RL_2 = sqrt(mean(e_RL_steady(:, 2).^2));
delta_RL_3 = sqrt(mean(e_RL_steady(:, 3).^2));

% 计算稳态最大误差
ex_max = max(abs(e_PosL_steady(:, 1)));
ey_max = max(abs(e_PosL_steady(:, 2)));
ez_max = max(abs(e_PosL_steady(:, 3)));
eRL_1_max = max(abs(e_RL_steady(:, 1)));
eRL_2_max = max(abs(e_RL_steady(:, 2)));
eRL_3_max = max(abs(e_RL_steady(:, 3)));

% 创建Excel文件名（包含控制器类型、干扰状态和时间戳）
% 控制器类型名称映射
controller_name_map = struct();
controller_name_map.pd = 'PD';
controller_name_map.classic = 'PD';
controller_name_map.backstepping = 'Backstepping';
controller_name_map.smc = 'SMC';
controller_name_map.rbf = 'RBF+RISE';

% 获取控制器名称
if isfield(controller_name_map, lower(payload_controller_type))
    controller_name = controller_name_map.(lower(payload_controller_type));
else
    controller_name = upper(payload_controller_type);  % 默认使用大写
end
if strcmpi(payload_controller_type, 'rbf')
    fprintf('控制器显示名称: %s\n', controller_name);
end

% 干扰状态标识
if with_disturbance
    disturbance_str = 'withDisturbance';
else
    disturbance_str = 'noDisturbance';
end

uav_F_type_meta = 'rise';
uav_M_type_meta = 'rise';
if isfield(controller_config, 'uav_control')
    if isfield(controller_config.uav_control, 'uav_F_type') && ~isempty(controller_config.uav_control.uav_F_type)
        uav_F_type_meta = lower(controller_config.uav_control.uav_F_type);
    end
    if isfield(controller_config.uav_control, 'uav_M_type') && ~isempty(controller_config.uav_control.uav_M_type)
        uav_M_type_meta = lower(controller_config.uav_control.uav_M_type);
    end
end
uav_control_str = sprintf('uavF_%s_uavM_%s', upper(uav_F_type_meta), upper(uav_M_type_meta));
uav_control_label = sprintf('F-%s / M-%s', upper(uav_F_type_meta), upper(uav_M_type_meta));

% 生成文件名：simulation_results_[控制器]_[干扰状态]_[时间戳].xlsx
excel_filename_results = sprintf('simulation_results_%s_%s_%s.xlsx', ...
    controller_name, disturbance_str, datestr(now, 'yyyymmdd_HHMMSS'));

% ==================== Sheet 1: 误差数据 ====================
% 保存位置误差、旋转矩阵误差、姿态误差函数、绳索误差函数
error_data = [t_extended', ...
              Save_extended.Errors_Save.e_p_L, ...
              Save_extended.Errors_Save.e_RL, ...
              Save_extended.Errors_Save.error_RL_norm_sq, ...
              Save_extended.Errors_Save.error_qi_norm_sq];
% 生成error_qi_norm_sq的列名
qi_error_cols = cell(Quadrotor_Num, 1);
for q_num = 1:Quadrotor_Num
    qi_error_cols{q_num} = sprintf('error_qi_norm_sq_UAV%d', q_num);
end
error_header = [{'t(s)', 'ePosL_x', 'ePosL_y', 'ePosL_z', ...
                 'eRL_1', 'eRL_2', 'eRL_3', ...
                 'error_RL_norm_sq'}, qi_error_cols'];
error_data_with_header = [error_header; num2cell(error_data)];
% writecell(error_data_with_header, excel_filename_results, 'Sheet', 'Errors');

% ==================== Sheet 2: 稳态指标 ====================
steady_state_data = {
    '指标名称', '数值';
    'delta_x (RMS)', delta_x;
    'delta_y (RMS)', delta_y;
    'delta_z (RMS)', delta_z;
    'delta_RL_1 (RMS)', delta_RL_1;
    'delta_RL_2 (RMS)', delta_RL_2;
    'delta_RL_3 (RMS)', delta_RL_3;
    'ex_max', ex_max;
    'ey_max', ey_max;
    'ez_max', ez_max;
    'eRL_1_max', eRL_1_max;
    'eRL_2_max', eRL_2_max;
    'eRL_3_max', eRL_3_max;
};
% writecell(steady_state_data, excel_filename_results, 'Sheet', 'SteadyState');

% ==================== Sheet 3: 无人机控制力Us ====================
for q_num = 1:Quadrotor_Num
    sheet_name = sprintf('Us_UAV%d', q_num);
    us_data = [t_extended', squeeze(Save_extended.Us_Save(:, :, q_num))];
    us_header = {'时间(s)', 'Us_x', 'Us_y', 'Us_z'};
    us_data_with_header = [us_header; num2cell(us_data)];
    % writecell(us_data_with_header, excel_filename_results, 'Sheet', sheet_name);
end

% ==================== Sheet 4: 负载姿态角跟踪 ====================
attitude_data = [t_extended', ...
                 Save_extended.Errors_Save.Att_L_actual, ...
                 Save_extended.Errors_Save.Att_L_desired, ...
                 Save_extended.Errors_Save.e_Att_L];
attitude_header = {'t(s)', ...
                   'Roll_actual', 'Pitch_actual', 'Yaw_actual', ...
                   'Roll_desired', 'Pitch_desired', 'Yaw_desired', ...
                   'eRoll', 'ePitch', 'eYaw'};
attitude_data_with_header = [attitude_header; num2cell(attitude_data)];
% writecell(attitude_data_with_header, excel_filename_results, 'Sheet', 'Load_Attitude');

% ==================== Sheet 5: 无人机姿态角跟踪 ====================
for q_num = 1:Quadrotor_Num
    sheet_name = sprintf('UAV%d_Attitude', q_num);
    uav_att_data = [t_extended', ...
                    squeeze(Save_extended.Errors_Save.UAV_Att_actual(:, :, q_num)), ...
                    squeeze(Save_extended.Errors_Save.UAV_Att_desired(:, :, q_num)), ...
                    squeeze(Save_extended.Errors_Save.UAV_e_Att(:, :, q_num))];
    uav_att_header = {'t(s)', ...
                      'Roll_actual', 'Pitch_actual', 'Yaw_actual', ...
                      'Roll_desired', 'Pitch_desired', 'Yaw_desired', ...
                      'eRoll', 'ePitch', 'eYaw'};
    uav_att_data_with_header = [uav_att_header; num2cell(uav_att_data)];
    % writecell(uav_att_data_with_header, excel_filename_results, 'Sheet', sheet_name);
end

% fprintf('结果已保存到Excel文件: %s\n', excel_filename_results);
% fprintf('  - Sheet "Errors": 误差数据\n');
% fprintf('  - Sheet "SteadyState": 稳态指标\n');
% fprintf('  - Sheet "Us_UAV1-3": 无人机控制力\n');
% fprintf('  - Sheet "Load_Attitude": 负载姿态角跟踪数据\n');
% fprintf('  - Sheet "UAV1_Attitude" ~ "UAV%d_Attitude": 无人机姿态角跟踪数据\n', Quadrotor_Num);

%% ==================== 保存本次运行数据到 MAT 文件 ====================
run_summary = struct();
run_summary.controller_type = payload_controller_type;
run_summary.with_disturbance = with_disturbance;
run_summary.controller_label = controller_name;
run_summary.disturbance_label = disturbance_str;
run_summary.uav_F_type = uav_F_type_meta;
run_summary.uav_M_type = uav_M_type_meta;
run_summary.uav_control_label = uav_control_label;
run_summary.delta_x = delta_x;
run_summary.delta_y = delta_y;
run_summary.delta_z = delta_z;
run_summary.delta_RL_1 = delta_RL_1;
run_summary.delta_RL_2 = delta_RL_2;
run_summary.delta_RL_3 = delta_RL_3;
run_summary.ex_max = ex_max;
run_summary.ey_max = ey_max;
run_summary.ez_max = ez_max;
run_summary.eRL_1_max = eRL_1_max;
run_summary.eRL_2_max = eRL_2_max;
run_summary.eRL_3_max = eRL_3_max;

run_result = struct();
run_result.meta = struct( ...
    'controller_type', payload_controller_type, ...
    'with_disturbance', with_disturbance, ...
    'controller_name', controller_name, ...
    'disturbance_str', disturbance_str, ...
    'uav_F_type', uav_F_type_meta, ...
    'uav_M_type', uav_M_type_meta, ...
    'uav_control_str', uav_control_str, ...
    'uav_control_label', uav_control_label, ...
    'timestamp', datestr(now, 'yyyymmdd_HHMMSS'), ...
    'dt', dt, ...
    'Tf', Tf, ...
    'N', N, ...
    'N_total', N_total, ...
    'takeoff_iter', takeoff_iter, ...
    'trajectory_mode', resolve_trajectory_mode(trajectory_config));
run_result.controller_config = controller_config;
run_result.experiment = experiment_config;
run_result.payload_plant = payload_plant;
run_result.meta.state_timing = 'x_k at t_k, pre-integration';
run_result.meta.control_timing = 'u_k held on [t_k,t_{k+1}); final sample diagnostic only';
run_result.meta.integration_intervals = N - 1;
run_result.time = struct('t', t, 't_main', t_main, 't_extended', t_extended);
run_result.states = struct('X_UAV', X_UAV_save_extended, 'X_L', X_L_save_extended);
run_result.desired = struct('DesPosL_main', DesPosL_main, 'RL_d_main', RL_d_main, ...
    'DesAttL_main', DesAttL_main, 'DesPosL_extended', DesPosL_extended, ...
    'DesAttL_extended', DesAttL_extended);
run_result.save_data = Save_extended;
run_result.plot_data = plotData;
run_result.metrics = run_summary;
if exist('theta_rbf', 'var')
    run_result.theta_rbf = theta_rbf;
end

if save_run_data_local
    if ~exist(run_output_dir_local, 'dir')
        mkdir(run_output_dir_local);
    end
    run_mat_filename = fullfile(run_output_dir_local, ...
        sprintf('simulation_run_%s_%s_%s.mat', controller_name, uav_control_str, disturbance_str));
    workspace_mat_filename = fullfile(run_output_dir_local, ...
        sprintf('simulation_workspace_%s_%s_%s.mat', controller_name, uav_control_str, disturbance_str));
    save(run_mat_filename, 'run_result', '-v7.3');
    save(workspace_mat_filename, '-v7.3');
    fprintf('运行结果已保存到 MAT 文件: %s\n', run_mat_filename);
    fprintf('完整工作区已保存到 MAT 文件: %s\n', workspace_mat_filename);
end

assignin('base', 'simulation_run_output', run_result);

%% ==================== 绘制图片 ====================
fprintf('\n开始绘制图片...\n');
if enable_default_plots_local

% 时间向量（从 t=0 开始，无起飞阶段）
t_plot_full = t_extended(1:N_total);
uav_position_plot = X_UAV_save_extended(1:N_total, [7, 9, 11], :);
uav_attitude_plot = X_UAV_save_extended(1:N_total, [1, 3, 5], :);
axis_names_pos = {'X (m)', 'Y (m)', 'Z (m)'};
axis_names_att = {'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)'};
uav_colors = lines(Quadrotor_Num);

fig_uav_position = figure('Name', '无人机位置变化（扰动后）', 'Position', [120, 120, 1200, 850]);
for axis_idx = 1:3
    subplot(3, 1, axis_idx);
    hold on;
    for q_num = 1:Quadrotor_Num
        plot(t_plot_full, squeeze(uav_position_plot(:, axis_idx, q_num)), ...
            'LineWidth', 1.8, 'Color', uav_colors(q_num, :), ...
            'DisplayName', sprintf('UAV%d', q_num));
    end
    xlabel('t (s)', 'FontSize', 16);
    ylabel(axis_names_pos{axis_idx}, 'FontSize', 16);
    title(sprintf('无人机%s变化', axis_names_pos{axis_idx}(1)), 'FontSize', 16);
    grid on;
    legend('Location', 'best');
    set(gca, 'FontSize', 13);
    hold off;
end

fig_uav_attitude = figure('Name', '无人机姿态变化（扰动后）', 'Position', [180, 180, 1200, 850]);
for axis_idx = 1:3
    subplot(3, 1, axis_idx);
    hold on;
    for q_num = 1:Quadrotor_Num
        plot(t_plot_full, squeeze(uav_attitude_plot(:, axis_idx, q_num)), ...
            'LineWidth', 1.8, 'Color', uav_colors(q_num, :), ...
            'DisplayName', sprintf('UAV%d', q_num));
    end
    xlabel('t (s)', 'FontSize', 16);
    ylabel(axis_names_att{axis_idx}, 'FontSize', 16);
    title(sprintf('无人机%s变化', axis_names_att{axis_idx}), 'FontSize', 16);
    grid on;
    legend('Location', 'best');
    set(gca, 'FontSize', 13);
    hold off;
end

fprintf('\n主函数默认历史绘图已关闭，仅绘制无人机位置和姿态变化。\n');
end
% fprintf('\n所有图片和Excel文件已保存完成！\n');

% %% ==================== 保存数据到Excel ====================
% % 创建Excel文件名（带时间戳）
% excel_filename = sprintf('simulation_data_%s.xlsx', datestr(now, 'yyyymmdd_HHMMSS'));
% 
% % 保存 Us (N_total x 3 x Quadrotor_Num)
% % 为每个无人机创建一个工作表
% for q_num = 1 : Quadrotor_Num
%     sheet_name = sprintf('Us_UAV%d', q_num);
%     % 转置为 (3 x N_total)，每行是一个方向（x, y, z）
%     data_to_save = squeeze(Save_extended.Us_Save(:, :, q_num))';  % (3 x N_total)
%     % 添加表头
%     header = {'时间(s)', 'X方向', 'Y方向', 'Z方向'};
%     data_with_header = [header; num2cell([t_extended', data_to_save'])];  % (N_total+1 x 4)
%     writecell(data_with_header, excel_filename, 'Sheet', sheet_name);
% end
% 
% % 保存 Fin (N_total x 3)
% sheet_name = 'Fin';
% data_to_save = Save_extended.Fin_Save;  % (N_total x 3)
% header = {'时间(s)', 'X方向', 'Y方向', 'Z方向'};
% data_with_header = [header; num2cell([t_extended', data_to_save])];
% writecell(data_with_header, excel_filename, 'Sheet', sheet_name);
% 
% % 保存 Min (N_total x 3)
% sheet_name = 'Min';
% data_to_save = Save_extended.Min_Save;  % (N_total x 3)
% header = {'时间(s)', 'X方向', 'Y方向', 'Z方向'};
% data_with_header = [header; num2cell([t_extended', data_to_save])];
% writecell(data_with_header, excel_filename, 'Sheet', sheet_name);
% 
% % 保存 Mplant (N_total+1 x 3 x Quadrotor_Num)
% % 为每个无人机创建一个工作表
% for q_num = 1 : Quadrotor_Num
%     sheet_name = sprintf('Mplant_UAV%d', q_num);
%     % 提取数据：Save_extended.Mplant_Save 是 (N_total+1) x 3 x Quadrotor_Num
%     % 转置为 (3 x N_total+1)，每行是一个方向（x, y, z）
%     mplant_data = squeeze(Save_extended.Mplant_Save(:, :, q_num));  % (N_total+1) x 3
%     data_to_save = mplant_data';  % (3 x N_total+1)，转置后每行是一个方向
%     % 时间向量（需要匹配N_total+1的长度）
%     t_mplant = [t_extended, t_extended(end) + dt];  % 扩展时间向量，长度为 N_total+1
%     % 确保维度匹配：t_mplant' 是 (N_total+1) x 1，data_to_save' 是 (N_total+1) x 3
%     % 添加表头
%     header = {'时间(s)', 'X方向', 'Y方向', 'Z方向'};
%     time_col = t_mplant(:);  % 确保是列向量 (N_total+1) x 1
%     data_matrix = [time_col, mplant_data];  % (N_total+1) x 4
%     data_with_header = [header; num2cell(data_matrix)];  % (N_total+2) x 4
%     writecell(data_with_header, excel_filename, 'Sheet', sheet_name);
% end
% 
% fprintf('数据已保存到Excel文件: %s\n', excel_filename);
% 
% %% ==================== 保存位置和姿态数据到Excel ====================
% % 创建Excel文件名（带时间戳）
% excel_filename_states = sprintf('states_data_%s.xlsx', datestr(now, 'yyyymmdd_HHMMSS'));
% 
% % 保存所有无人机的位置和姿态 (N_total x 12 x Quadrotor_Num)
% % 为每个无人机创建一个工作表
% for q_num = 1 : Quadrotor_Num
%     sheet_name = sprintf('UAV%d_States', q_num);
%     % X_UAV_save_extended: [roll, wx, pitch, wy, yaw, wz, x, vx, y, vy, z, vz]
%     data_to_save = X_UAV_save_extended(:, :, q_num);  % (N_total x 12)
%     % 添加表头
%     header = {'时间(s)', 'Roll(rad)', 'Wx(rad/s)', 'Pitch(rad)', 'Wy(rad/s)', 'Yaw(rad)', 'Wz(rad/s)', ...
%               'X(m)', 'Vx(m/s)', 'Y(m)', 'Vy(m/s)', 'Z(m)', 'Vz(m/s)'};
%     data_with_header = [header; num2cell([t_extended', data_to_save])];
%     writecell(data_with_header, excel_filename_states, 'Sheet', sheet_name);
% end
% 
% % 保存货物位置和姿态 (N_total x 12)
% sheet_name = 'Load_States';
% % X_L_save_extended: [roll, wx, pitch, wy, yaw, wz, x, vx, y, vy, z, vz]
% data_to_save = X_L_save_extended;  % (N_total x 12)
% % 添加表头
% header = {'时间(s)', 'Roll(rad)', 'Wx(rad/s)', 'Pitch(rad)', 'Wy(rad/s)', 'Yaw(rad)', 'Wz(rad/s)', ...
%           'X(m)', 'Vx(m/s)', 'Y(m)', 'Vy(m/s)', 'Z(m)', 'Vz(m/s)'};
% data_with_header = [header; num2cell([t_extended', data_to_save])];
% writecell(data_with_header, excel_filename_states, 'Sheet', sheet_name);
% 
% fprintf('位置和姿态数据已保存到Excel文件: %s\n', excel_filename_states);

function mode = resolve_trajectory_mode(trajectory_config)
mode = 'periodic';
if nargin >= 1 && isstruct(trajectory_config) && isfield(trajectory_config, 'mode') && ...
        ~isempty(trajectory_config.mode)
    mode = trajectory_config.mode;
end
end
