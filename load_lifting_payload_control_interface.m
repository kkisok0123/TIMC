function [Fd, Md, errors, varargout] = load_lifting_payload_control_interface(controller_type, params)
%% 货物控制器统一接口
% 功能：根据控制器类型调用相应的控制器实现
% 输入参数：
%   controller_type - 控制器类型字符串：
%       'pd' 或 'classic' - PD控制器
%       'backstepping' - 反步控制器
%       'smc' - SMC滑模控制器
%       'rbf' - RBF神经网络控制器
%       'rise' - RISE控制器
%   params - 参数结构体，包含控制器所需的所有参数
%%     对于所有负载控制器，必须提供 controller_config，且该配置必须来自 load_lifting_controller_params.m
%     对于 'pd'/'classic': 需要 x_L, Des_Att, Des_pL, RL_d, x_L_prev（可选）
%     对于 'backstepping': 需要 x_L, Des_Att, Des_pL, RL_d, x_L_prev（可选）
%     对于 'smc': 需要 x_L, Des_Att, Des_pL, RL_d, x_L_prev（可选）
%     对于 'rbf': 需要 x_L, Des_Att, Des_pL, RL_d, theta_rbf_prev, dt, x_L_prev（可选）
%     对于 'backstepping_rbf_eso': 同 'rbf'，但基础控制器为 backstepping
%     对于 'rise': 需要 x_L, Des_Att, Des_pL, RL_d, dt（可选）
% 输出参数：
%   Fd - 期望力 (3x1) - 世界坐标系
%   Md - 期望力矩 (3x1) - 机体坐标系
%   errors - 误差结构体，包含不同控制器返回的误差信息
%   RL (可选) - 旋转矩阵 (3x3)，经典控制器返回
%   theta_rbf (可选) - 更新后的RBF参数 (30x1)，仅当controller_type='rbf'时返回

controller_config = require_payload_controller_config(params, controller_type);

switch lower(controller_type)
    case {'pd', 'classic'}
        % PD控制器（经典控制器）
        % params 应包含：x_L, Des_Att, Des_pL, RL_d, x_L_prev（可选，但当前实现不使用）
        if ~isfield(params, 'x_L_prev')
            params.x_L_prev = [];
        end
        [Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, RL] = ...
            load_lifting_payload_control_pd(params.x_L, params.Des_Att, params.Des_pL, params.RL_d, controller_config);
        
        % 统一误差结构体（字段名必须与save_simulation_data期望的一致）
        errors = struct();
        errors.e_RL = e_RL;
        errors.e_Ome_L = e_Ome_L;
        errors.e_p_L = e_pL;  % 注意：使用下划线以匹配save_simulation_data
        errors.e_v_L = e_vL;  % 注意：使用下划线以匹配save_simulation_data
        errors.e1_all = [];  % PD控制器不使用
        errors.e2_all = [];  % PD控制器不使用
        
        % 返回 RL（旋转矩阵）作为可选输出
        if nargout > 3
            varargout{1} = RL;
        end
        
    case 'backstepping'
        % 反步控制器
        % params 应包含：x_L, Des_Att, Des_pL, RL_d, x_L_prev（可选）
        if ~isfield(params, 'x_L_prev')
            params.x_L_prev = [];
        end
        [Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, RL] = ...
            load_lifting_payload_control_backstepping(params.x_L, params.Des_Att, params.Des_pL, params.RL_d, controller_config);
        
        % 统一误差结构体
        errors = struct();
        errors.e_RL = e_RL;
        errors.e_Ome_L = e_Ome_L;
        errors.e_p_L = e_pL;
        errors.e_v_L = e_vL;
        errors.e1_all = [];  % 反步控制器不使用
        errors.e2_all = [];  % 反步控制器不使用
        
        % 返回 RL（旋转矩阵）作为可选输出
        if nargout > 3
            varargout{1} = RL;
        end
        
    case 'smc'
        % SMC滑模控制器
        % params 应包含：x_L, Des_Att, Des_pL, RL_d, x_L_prev（可选）
        if ~isfield(params, 'x_L_prev')
            params.x_L_prev = [];
        end
        [Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, RL] = ...
            load_lifting_payload_control_smc(params.x_L, params.Des_Att, params.Des_pL, params.RL_d, controller_config);
        
        % 统一误差结构体
        errors = struct();
        errors.e_RL = e_RL;
        errors.e_Ome_L = e_Ome_L;
        errors.e_p_L = e_pL;
        errors.e_v_L = e_vL;
        errors.e1_all = [];  % SMC控制器不使用
        errors.e2_all = [];  % SMC控制器不使用
        
        % 返回 RL（旋转矩阵）作为可选输出
        if nargout > 3
            varargout{1} = RL;
        end
        
    case 'rbf'
        % 反步+RBF+ESO 组合控制器
        % params 应包含：x_L, Des_Att, Des_pL, RL_d, theta_rbf_prev, dt, x_L_prev（可选）
        %                eso_state_prev（可选），u_prev（可选）
        if ~isfield(params, 'theta_rbf_prev')
            params.theta_rbf_prev = [];
        end
        if ~isfield(params, 'dt')
            params.dt = [];
        end
        if ~isfield(params, 'x_L_prev')
            params.x_L_prev = [];
        end
        if ~isfield(params, 'with_disturbance')
            params.with_disturbance = controller_config.with_disturbance;
        end
        if ~isfield(params, 'eso_state_prev')
            params.eso_state_prev = [];  % ESO状态（可选）
        end
        if ~isfield(params, 'u_prev')
            params.u_prev = [];  % 上一时刻控制输入（可选）
        end
        
        [Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, theta_rbf, fn, fn_att, h_rbf_all, eso_state] = ...
            load_lifting_payload_control_rbf(params.x_L, params.Des_Att, params.Des_pL, params.RL_d, ...
                                              params.theta_rbf_prev, params.dt, params.x_L_prev, ...
                                              params.with_disturbance, controller_config, ...
                                              params.eso_state_prev, params.u_prev);
        
        % 统一误差结构体
        errors = struct();
        errors.e_RL = e_RL;
        errors.e_Ome_L = e_Ome_L;
        errors.e_p_L = e_pL;
        errors.e_v_L = e_vL;
        errors.e1_all = [];  % RBF控制器不使用
        errors.e2_all = [];  % RBF控制器不使用
        
        % 返回更新后的RBF参数和RBF网络输出
        if nargout > 3
            varargout{1} = theta_rbf;
        end
        if nargout > 4
            varargout{2} = fn;  % 位置控制RBF输出
        end
        if nargout > 5
            varargout{3} = fn_att;  % 姿态控制RBF输出
        end
        if nargout > 6
            varargout{4} = h_rbf_all;  % RBF基函数值 (5 x 6)
        end
        if nargout > 7
            varargout{5} = eso_state;  % ESO状态 (18x1)
        end

    case {'backstepping_rbf_eso', 'bs_rbf_eso'}
        if ~isfield(params, 'theta_rbf_prev')
            params.theta_rbf_prev = [];
        end
        if ~isfield(params, 'dt')
            params.dt = [];
        end
        if ~isfield(params, 'x_L_prev')
            params.x_L_prev = [];
        end
        if ~isfield(params, 'with_disturbance')
            params.with_disturbance = controller_config.with_disturbance;
        end
        if ~isfield(params, 'eso_state_prev')
            params.eso_state_prev = [];
        end
        if ~isfield(params, 'u_prev')
            params.u_prev = [];
        end

        [Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, theta_rbf, fn, fn_att, h_rbf_all, eso_state] = ...
            load_lifting_payload_control_backstepping_rbf_eso(params.x_L, params.Des_Att, params.Des_pL, params.RL_d, ...
                                                              params.theta_rbf_prev, params.dt, params.x_L_prev, ...
                                                              params.with_disturbance, controller_config, ...
                                                              params.eso_state_prev, params.u_prev);

        errors = struct();
        errors.e_RL = e_RL;
        errors.e_Ome_L = e_Ome_L;
        errors.e_p_L = e_pL;
        errors.e_v_L = e_vL;
        errors.e1_all = [];
        errors.e2_all = [];

        if nargout > 3
            varargout{1} = theta_rbf;
        end
        if nargout > 4
            varargout{2} = fn;
        end
        if nargout > 5
            varargout{3} = fn_att;
        end
        if nargout > 6
            varargout{4} = h_rbf_all;
        end
        if nargout > 7
            varargout{5} = eso_state;
        end
        
    case 'rise'
        % RISE控制器
        % params 应包含：x_L, Des_Att, Des_pL, RL_d, dt（可选），controller_config（必须）
        if ~isfield(params, 'dt')
            params.dt = [];
        end

        [Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, RL] = ...
            load_lifting_payload_control_rise(params.x_L, params.Des_Att, params.Des_pL, params.RL_d, controller_config, params.dt);
        
        % 统一误差结构体
        errors = struct();
        errors.e_RL = e_RL;
        errors.e_Ome_L = e_Ome_L;
        errors.e_p_L = e_pL;
        errors.e_v_L = e_vL;
        errors.e1_all = [];  % RISE控制器不使用
        errors.e2_all = [];  % RISE控制器不使用
        
        % 返回 RL（旋转矩阵）作为可选输出
        if nargout > 3
            varargout{1} = RL;
        end
        
    otherwise
        error('不支持的控制器类型: %s. 支持的类型: ''pd''/''classic'', ''backstepping'', ''smc'', ''rbf'', ''backstepping_rbf_eso'', ''rise''', controller_type);
end

end

function controller_config = require_payload_controller_config(params, controller_type)
if ~isfield(params, 'controller_config') || isempty(params.controller_config)
    error(['Payload controller "%s" requires controller_config from ' ...
           'load_lifting_controller_params.m.'], controller_type);
end

controller_config = params.controller_config;
end
