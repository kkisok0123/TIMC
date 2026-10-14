function [X_dot] = load_lifting_UAV_dynamics(x, F_plant, M_plant, t, uav_disturbance_config, uav_index)
global g  m_p1
global a1_p a2_p a3_p
global b1_p b2_p b3_p

if nargin < 4 || isempty(t)
    t = 0;
end
if nargin < 5
    uav_disturbance_config = [];
end
if nargin < 6
    uav_index = [];
end

[attitude_disturbance, position_disturbance] = resolve_uav_disturbance(t, uav_disturbance_config, uav_index);

X_dot = zeros(12,1);
% 无人机的位姿信息
[~, ~, eta_dot] = zyx_euler_kinematics(x([1,3,5]), x([2,4,6]));
X_dot(1) = eta_dot(1);
X_dot(2) = x(4) * x(6) * a1_p + M_plant(1) * b1_p + attitude_disturbance(1);
X_dot(3) = eta_dot(2);
X_dot(4) = x(2) * x(6) * a2_p + M_plant(2) * b2_p + attitude_disturbance(2);
X_dot(5) = eta_dot(3);
X_dot(6) = x(2) * x(4) * a3_p + M_plant(3) * b3_p + attitude_disturbance(3);

% 位置状态
X_dot(7)  = x(8); 
X_dot(8)  = F_plant(1)/m_p1 + position_disturbance(1);
X_dot(9)  = x(10);
X_dot(10) = F_plant(2)/m_p1 + position_disturbance(2);
X_dot(11) = x(12);
X_dot(12) = -g + F_plant(3)/m_p1 + position_disturbance(3);
end

function [attitude_disturbance, position_disturbance] = resolve_uav_disturbance(t, uav_disturbance_config, uav_index)
attitude_disturbance = zeros(3, 1);
position_disturbance = zeros(3, 1);

if isempty(uav_disturbance_config) || ~isstruct(uav_disturbance_config)
    return;
end
if ~isfield(uav_disturbance_config, 'enabled') || ~uav_disturbance_config.enabled
    return;
end
if ~isfield(uav_disturbance_config, 'schedule') || isempty(uav_disturbance_config.schedule)
    return;
end

for schedule_idx = 1:numel(uav_disturbance_config.schedule)
    schedule_item = uav_disturbance_config.schedule(schedule_idx);

    if isfield(schedule_item, 'enabled') && ~schedule_item.enabled
        continue;
    end

    start_time = 0;
    end_time = inf;
    if isfield(schedule_item, 'start_time') && ~isempty(schedule_item.start_time)
        start_time = schedule_item.start_time;
    end
    if isfield(schedule_item, 'end_time') && ~isempty(schedule_item.end_time)
        end_time = schedule_item.end_time;
    end
    if t < start_time || t > end_time
        continue;
    end

    if ~is_target_uav(schedule_item, uav_index)
        continue;
    end

    if isfield(schedule_item, 'attitude_accel') && ~isempty(schedule_item.attitude_accel)
        attitude_disturbance = attitude_disturbance + reshape(schedule_item.attitude_accel, 3, 1);
    end
    if isfield(schedule_item, 'position_accel') && ~isempty(schedule_item.position_accel)
        position_disturbance = position_disturbance + reshape(schedule_item.position_accel, 3, 1);
    end
end
end

function is_selected = is_target_uav(schedule_item, uav_index)
if isempty(uav_index)
    is_selected = true;
    return;
end

if ~isfield(schedule_item, 'uav_ids') || isempty(schedule_item.uav_ids)
    is_selected = true;
    return;
end

is_selected = any(schedule_item.uav_ids(:) == uav_index);
end
