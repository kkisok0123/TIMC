function [F_in, M_in, miu_i, qi_d_now, sOmegaid_now, e_qi, e_sOmegai, a, ui, sOmega_dot, input_error,miu_i_d] = load_lifting_allocation(Fd, Md, RL, rho, qi, sOmega, sOmegai_d, qi_d, Ome_L, controller_config)
%% 第二部分：解算模块 - 将货物的控制力和控制力矩分配到各无人机
% 输入参数：
%   Fd: 期望力 (3x1) - 世界坐标系
%   Md: 期望力矩 (3x1) - 机体坐标系
%   RL: 当前旋转矩阵 (3x3)
%   rho: 力臂向量 (3xQuadrotor_Num) - 机体坐标系
%   qi: 绳索方向 (3xQuadrotor_Numx2)
%   sOmega: 绳索角速度 (3xQuadrotor_Num)
%   sOmegai_d: 期望绳索角速度 (3xQuadrotor_Num)
%   qi_d: 上一时刻期望绳索方向 (3xQuadrotor_Num)
%   Ome_L: 物体角速度 (3x1)
%   controller_type: 控制器类型 ('pd', 'backstepping', 'rbf', 'smc' 等)
%   with_disturbance: 是否包含干扰项 (true/false)
% 输出参数：
%   F_in: 实际输入力 (3x1) - 世界坐标系
%   M_in: 实际输入力矩 (3x1) - 机体坐标系
%   miu_i: 绳索张力 (3xQuadrotor_Num) - 世界坐标系
%   qi_d_now: 期望绳索方向 (3xQuadrotor_Num)
%   sOmegaid_now: 期望绳索角速度 (3xQuadrotor_Num)
%   e_qi: 绳索方向误差 (3xQuadrotor_Num)
%   e_sOmegai: 绳索角速度误差 (3xQuadrotor_Num)
%   a: 加速度 (3xQuadrotor_Num)
%   ui: 无人机输入力 (3xQuadrotor_Num) - 世界坐标系
%   sOmega_dot: 绳索角速度导数 (3xQuadrotor_Num)
%   input_error: 输入误差 (3x2): [Fd-F_in, Md-M_in]

global dt m_L m_p J_L l_plant Quadrotor_Num

rho_hat = zeros(3, 3, Quadrotor_Num);
I33 = eye(3);

% 初始化P为6行、3×Quadrotor_Num列的零矩阵
P = zeros(6, 3 * Quadrotor_Num);

% 循环填充P的每一块（按无人机序号）
for q_num = 1 : Quadrotor_Num
    rho_hat(:,:,q_num) = skew(rho(:,q_num));
    % 计算当前无人机在P中的列索引范围（每个无人机占3列）
    col_start = (q_num - 1) * 3 + 1;  % 起始列
    col_end = col_start + 2;          % 结束列（3列：start, start+1, start+2）
    
    % 上半部分：填充I33
    P(1:3, col_start:col_end) = I33;
    
    % 下半部分：填充当前无人机的rho_hat
    P(4:6, col_start:col_end) = rho_hat(:,:,q_num);
end

PT = P';
RL_diag = kron(eye(Quadrotor_Num), RL);
if rank(P*PT) < size(P*PT,1)
    warning('P*PT 可能奇异，使用伪逆');
    PPinv = pinv(P*PT);
else
    PPinv = inv(P*PT);
end

% 把坐标系转换到机体坐标系中
temp_vec = [RL'*Fd;
            Md];

% miud通过RL_diag转到世界坐标系中
miud = RL_diag * PT * PPinv * temp_vec;

miu_i_d = zeros(3,Quadrotor_Num);

% 当前q（修复：qi是2D数组，不是3D）
q_now = qi;
q_now_dot = zeros(3, Quadrotor_Num);
q_now_hat = zeros(3, 3, Quadrotor_Num);

% miu
miu_i = zeros(3, Quadrotor_Num);
% 输入F_in
F_in = zeros(3,1);
% 计算全局坐标系下的力臂向量 M_in
rho_global = zeros(3, Quadrotor_Num);
% 全局坐标系下的总力矩
Mi_in = zeros(3, Quadrotor_Num);
M_in = zeros(3, 1);

% 当前时刻的q_i_d
qi_d_now = zeros(3, Quadrotor_Num);
qi_d_now_dot = zeros(3, Quadrotor_Num);

% 绳索的角速度预期值sOmega_d
sOmegaid_now = zeros(3, Quadrotor_Num);
sOmegaid_now_dot = zeros(3, Quadrotor_Num);
% 计算误差e_q
e_qi = zeros(3, Quadrotor_Num);
% 这里的sOmega需要将上一时刻的传入
e_sOmegai = zeros(3, Quadrotor_Num);

% 从qi_d中提取上一时刻的值（需要从外部传入，这里假设有全局变量或从输入中获取）
% 注意：这里需要从外部传入qi_d，但为了简化，我们假设可以从sOmegai_d推导
% 实际上qi_d应该作为输入参数，但为了保持接口简洁，我们从循环中计算

% 从controller_config读取参数
if nargin >= 10 && ~isempty(controller_config) && isstruct(controller_config)
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

% 提取参数
KsOmega_i = controller_params.allocation.KsOmega_i;
Kq_i = controller_params.allocation.Kq_i;
alpha_filter = controller_params.allocation.alpha_filter;

for q_num = 1 : Quadrotor_Num
    % 计算当前无人机在P中的列索引范围（每个无人机占3列）
    col_start = (q_num - 1) * 3 + 1;  % 起始列
    col_end = col_start + 2;          % 结束列（3列：start, start+1, start+2）

    % 无人机i所需要的控制输入，在世界坐标系下
    miu_i_d(:, q_num) = miud(col_start:col_end, :);

    % q_dot
    rho_global(:, q_num) = RL * rho(:,q_num);
    q_now_dot(:, q_num) = cross(sOmega(:, q_num), q_now(:, q_num));

    % q_hat
    q_now_hat(:, :, q_num) = skew(q_now(:, q_num));

    % miu，在世界坐标系下
    miu_i(:, q_num) = q_now(:, q_num) * q_now(:, q_num)' * miu_i_d(:, q_num);

    % F_in_global，世界坐标系
    F_in = F_in + miu_i(:, q_num);

    % 计算机体坐标系下的总力矩
    Mi_in(:, q_num) = skew(rho(:,q_num)) * RL' * miu_i(:, q_num);

    % 转换到体坐标系
    M_in = M_in + Mi_in(:, q_num);
    
    % 期望绳索方向（世界系）- 添加平滑处理避免突然跳变
    % 计算新的期望方向
    miu_i_d_normalized = - miu_i_d(:,q_num) / norm(miu_i_d(:,q_num));
    
    % 平滑过渡：如果上一时刻的qi_d存在，使用低通滤波平滑
    if norm(qi_d(:,q_num)) > 1e-6  % 确保上一时刻的qi_d有效
        % 计算两个方向之间的角度
        dot_prod = qi_d(:,q_num)' * miu_i_d_normalized;
        dot_prod = max(-1, min(1, dot_prod));  % 限制在[-1, 1]范围内
        angle_diff = acos(dot_prod);
        
        % 如果角度变化较大，使用平滑过渡
        if angle_diff > 0.1  % 角度变化超过约5.7度时进行平滑
            % 使用一阶低通滤波器平滑过渡（alpha_filter已在上面根据控制器类型和干扰项设置）
            qi_d_now(:, q_num) = (1 - alpha_filter) * qi_d(:,q_num) + alpha_filter * miu_i_d_normalized;
            qi_d_now(:, q_num) = qi_d_now(:, q_num) / norm(qi_d_now(:, q_num));  % 归一化
        else
            % 角度变化较小，直接使用新值
            qi_d_now(:, q_num) = miu_i_d_normalized;
        end
    else
        % 上一时刻无效，直接使用新值
        qi_d_now(:, q_num) = miu_i_d_normalized;
    end
    
    qi_d_now_dot(:, q_num) = (qi_d_now(:, q_num) - qi_d(:,q_num)) / dt;

    % 期望绳索角速度与其导数（世界系）
    sOmegaid_now(:, q_num) = cross(qi_d_now(:, q_num), qi_d_now_dot(:, q_num));
    sOmegaid_now_dot(:, q_num) = (sOmegaid_now(:, q_num) - sOmegai_d(:,q_num)) / dt;
    
    % q的方向误差（世界系）
    e_qi(:, q_num) = cross(qi_d_now(:, q_num), q_now(:, q_num));

    % 绳索角速度误差（世界系）
    e_sOmegai(:, q_num) = sOmega(:,q_num) + q_now_hat(:, :, q_num)^2 * sOmegaid_now(:, q_num);
    
end

input_error = zeros(3,2);
input_error(:,1) = Fd - F_in;
input_error(:,2) = Md - M_in;

% 货物omega
x_Ome_L_hat = skew(Ome_L);
a = zeros(3, Quadrotor_Num);

% u_n_i
un_i = zeros(3, Quadrotor_Num);
% 求解un
% sOmega 绳索
% 注意：参数已在循环之前根据控制器类型和干扰项设置完成

% 绳锁转动速度导数
sOmega_dot = zeros(3,Quadrotor_Num);

% up_i
up_i = zeros(3, Quadrotor_Num);

% ui
ui = zeros(3,Quadrotor_Num);

for q_num = 1 : Quadrotor_Num
    % a 世界坐标系
    a(:, q_num) = (1/m_L) * F_in + RL * x_Ome_L_hat^2 * rho(:,q_num) + RL * rho_hat(:,:,q_num) * inv(J_L) * (x_Ome_L_hat * J_L * Ome_L - M_in);

    % un 世界坐标系
    un_i_term0 =  m_p(q_num) * l_plant(q_num) * q_now_hat(:,:,q_num);
    un_i_term1 = -Kq_i(q_num) * e_qi(:, q_num);
    un_i_term2 = - KsOmega_i(q_num) * e_sOmegai(:, q_num);
    un_i_term3 = - (q_now(:, q_num)' * sOmegaid_now(:, q_num))*q_now_dot(:, q_num);
    un_i_term4 = - q_now_hat(:, :, q_num)^2 * sOmegaid_now_dot(:, q_num);
    un_i_term5 = - m_p(q_num) * q_now_hat(:,:,q_num)^2 * a(:, q_num);
    un_i(:, q_num) = un_i_term0 * (un_i_term1 + un_i_term2 + un_i_term3 + un_i_term4) + un_i_term5;

    % 绳锁转动速度导数
    sOmega_dot(:,q_num) = -Kq_i(q_num) * e_qi(:, q_num) - KsOmega_i(q_num) * e_sOmegai(:, q_num) - (q_now(:,q_num)' * sOmegaid_now(:, q_num)) * q_now_dot(:,q_num) - q_now_hat(:,:,q_num)^2 * sOmegaid_now_dot(:, q_num);
    % up
    up_i(:, q_num) = miu_i(:,q_num) + m_p(q_num) * (l_plant(q_num) * norm(sOmega(:,q_num))^2 * q_now(:,q_num) + q_now(:,q_num) * q_now(:,q_num)' * a(:, q_num));
    % 计算无人机的力和力矩
    ui(:, q_num) = un_i(:, q_num) + up_i(:, q_num);
    % 限制 ui 的范围
    for dim = 1 : 3
        if dim == 3
            % z方向：限制在 0 到 20 之间
            if ui(dim,q_num) > 50
                ui(dim,q_num) = 50;
            elseif ui(dim,q_num) < 0
                ui(dim,q_num) = 0;
            end
        else
            % x和y方向：限制在 -20 到 20 之间
            if ui(dim,q_num) > 30
                ui(dim,q_num) = 30;
            elseif ui(dim,q_num) < -30
                ui(dim,q_num) = -30;
            end
        end
    end
end

end

%% ================== 斜对称矩阵生成函数 ==================
function S = skew(v)
    % v 为 3x1 向量, S 则为 3x3 反对称矩阵,满足 S*w = v x w
    S = [   0,  -v(3),  v(2);
         v(3),     0,   -v(1);
        -v(2),   v(1),    0   ];
end

