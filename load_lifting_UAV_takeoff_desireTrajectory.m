function [DesPos, DesAtt] = load_lifting_UAV_takeoff_desireTrajectory(q_num)
%% 期望轨迹生成函数（起飞阶段）
% 输入参数：
%   q_num: 无人机编号 (1, 2, 3, ...)
% 输出参数：
%   DesPos: 期望位置/速度/加速度 (3x3)
%           列定义: [:,1]=pos, [:,2]=vel, [:,3]=acc
%           行定义: [x;y;z]
%   DesAtt: 期望姿态角/角速度/角加速度 (3x3)
%           列定义: [:,1]=angle, [:,2]=angVel, [:,3]=angAcc
%           行定义: [roll(phi); pitch(theta); yaw(psi)]
global l_plant load_height

% 根据q_num给定该无人机的期望位置（x,y,z）
if q_num == 1
    pos_d = [0.5;  4.2; l_plant(1) + load_height];
elseif q_num == 2
    pos_d = [-0.5; 4.6; l_plant(2) + load_height];
elseif q_num == 3
    pos_d = [-0.5; 3.8; l_plant(3) + load_height];
else
    % 默认：如果无人机数量变了，先把x,y设为0，z按对应绳长+货物高度
    pos_d = [0; 0; l_plant(q_num) + load_height];
end

% 期望位置/速度/加速度（起飞阶段速度、加速度设为0）
DesPos = [pos_d(1); 0; 0; pos_d(2); 0; 0; pos_d(3); 0; 0];
DesAtt = zeros(9,1);

end
