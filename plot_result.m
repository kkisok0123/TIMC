function plot_result(data, plotOptions)
%% 统一绘图函数
% 输入参数：
%   data: 结构体，包含所有绘图数据
%   plotOptions: 结构体，指定要绘制哪些图（可选，默认绘制所有图）
%
% data 结构体字段：
%   - t: 时间向量
%   - N: 时间步数
%   - Quadrotor_Num: 无人机数量
%   - X_UAV_save: 无人机状态保存 (N x 12 x Quadrotor_Num)
%   - X_L_save: 货物状态保存 (N x 12)
%   - Des_pL: 期望位置 (3 x 3 x N)
%   - Att_L_d: 期望姿态 (3 x N)
%   - omegaid_p_Save: 期望角度保存 (N x 3 x Quadrotor_Num)
%   - Input_error_all: 输入误差 (N x 3 x 2)
%   - Fplant_Save: 无人机力输入 (N x Quadrotor_Num)
%   - Fd_Save: 期望力 (N x 3)
%   - Md_Save: 期望力矩 (N x 3)
%   - Fin_Save: 实际力 (N x 3)
%   - Min_Save: 实际力矩 (N x 3)
%   - Us_Save: 无人机输入 (N x 3 x Quadrotor_Num)
%   - Errors_Save: 误差结构体
%
% plotOptions 结构体字段（可选，默认全部为true）：
%   - plotUAV3D: 绘制无人机3D轨迹
%   - plotPayloadTrajectory3D: 绘制货物3D轨迹对比
%   - plotPayloadTrajectory2D: 绘制货物2D轨迹对比
%   - plotPayloadAttitude: 绘制货物期望姿态
%   - plotUAVTrajectory2D: 绘制无人机2D轨迹
%   - plotUAVTrajectory3D: 绘制无人机3D轨迹
%   - plotErrors: 绘制所有误差图

% 默认绘制所有图
if nargin < 2 || isempty(plotOptions)
    plotOptions = struct();
    plotOptions.plotUAV3D = true;
    plotOptions.plotPayloadTrajectory3D = true;
    plotOptions.plotPayloadTrajectory2D = true;
    plotOptions.plotPayloadAttitude = true;
    plotOptions.plotUAVTrajectory2D = true;
    plotOptions.plotUAVTrajectory3D = true;
    plotOptions.plotErrors = false;  % 默认不绘制误差图（因为有很多）
end

% 根据选项调用相应的绘图函数
if isfield(plotOptions, 'plotUAV3D') && plotOptions.plotUAV3D
    plot_UAV_3D_trajectory(data);
end

if isfield(plotOptions, 'plotPayloadTrajectory3D') && plotOptions.plotPayloadTrajectory3D
    plot_payload_trajectory_3D(data);
end

if isfield(plotOptions, 'plotPayloadTrajectory2D') && plotOptions.plotPayloadTrajectory2D
    plot_payload_trajectory_2D(data);
end

if isfield(plotOptions, 'plotPayloadAttitude') && plotOptions.plotPayloadAttitude
    plot_payload_attitude(data);
end

if isfield(plotOptions, 'plotUAVTrajectory2D') && plotOptions.plotUAVTrajectory2D
    plot_UAV_trajectory_2D(data);
end

if isfield(plotOptions, 'plotUAVTrajectory3D') && plotOptions.plotUAVTrajectory3D
    plot_UAV_trajectory_3D(data);
end

if isfield(plotOptions, 'plotErrors') && plotOptions.plotErrors
    plot_all_errors(data);
end

end

%% ==================== 子绘图函数 ====================

function plot_UAV_3D_trajectory(data)
%% 绘制无人机3D轨迹（包含连接线）
stepping = 500;
marker_shapes = cell(1, data.Quadrotor_Num);
shapes = {'^', 'd', 's', 'o', '+', '*', 'x'};
for Num = 1:data.Quadrotor_Num
    marker_shapes{Num} = shapes{mod(Num - 1, length(shapes)) + 1};
end

figure;
hold on;
% 绘制标记点
for Num = 1:data.Quadrotor_Num
    plot3(data.X_UAV_save(1:stepping:data.N, 7, Num), ...
          data.X_UAV_save(1:stepping:data.N, 9, Num), ...
          data.X_UAV_save(1:stepping:data.N, 11, Num), ...
          marker_shapes{Num}, 'Linewidth', 2);
end
% 绘制轨迹线
for Num = 1:data.Quadrotor_Num
    plot3(data.X_UAV_save(1:stepping:data.N, 7, Num), ...
          data.X_UAV_save(1:stepping:data.N, 9, Num), ...
          data.X_UAV_save(1:stepping:data.N, 11, Num), ...
          'Linewidth', 2);
end
% 绘制连接线
pointNUM = 1:stepping:data.N;
for Num = pointNUM
    for j = 1:data.Quadrotor_Num
        plot3([data.X_UAV_save(Num,7,j), data.X_UAV_save(Num,7,mod(j,data.Quadrotor_Num) + 1)], ...
              [data.X_UAV_save(Num,9,j), data.X_UAV_save(Num,9,mod(j,data.Quadrotor_Num) + 1)], ...
              [data.X_UAV_save(Num,11,j), data.X_UAV_save(Num,11,mod(j,data.Quadrotor_Num) + 1)], ...
              'Linewidth', 1.5);
    end
end
title('无人机3D轨迹');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;
hold off;
end

function plot_payload_trajectory_3D(data)
%% 绘制货物3D轨迹对比
figure;
hold on;
% 实际轨迹
plot3(data.X_L_save(:,7), data.X_L_save(:,9), data.X_L_save(:,11), ...
      'Color', '#0000FF', 'Linewidth', 3, 'DisplayName', '实际轨迹');
% 期望轨迹
plot3(squeeze(data.Des_pL(1,1,:)), squeeze(data.Des_pL(2,1,:)), squeeze(data.Des_pL(3,1,:)), ...
      'Color', '#FF0000', 'Linewidth', 2, 'LineStyle', '--', 'DisplayName', '期望轨迹');
title('货物轨迹对比：实际 vs 期望（三维）');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Location', 'best');
grid on;
axis equal;
view(45, 30);
hold off;
end

function plot_payload_trajectory_2D(data)
%% 绘制货物2D轨迹对比（分方向）
figure;
% X方向
subplot(3,1,1);
hold on;
plot(data.t, data.X_L_save(:,7), '-b', 'LineWidth', 2, 'DisplayName', '实际X');
plot(data.t, squeeze(data.Des_pL(1,1,:)), '--r', 'LineWidth', 2, 'DisplayName', '期望X');
title('货物X方向轨迹对比');
xlabel('时间 (s)');
ylabel('X (m)');
legend('Location', 'best');
grid on;
hold off;

% Y方向
subplot(3,1,2);
hold on;
plot(data.t, data.X_L_save(:,9), '-b', 'LineWidth', 2, 'DisplayName', '实际Y');
plot(data.t, squeeze(data.Des_pL(2,1,:)), '--r', 'LineWidth', 2, 'DisplayName', '期望Y');
title('货物Y方向轨迹对比');
xlabel('时间 (s)');
ylabel('Y (m)');
legend('Location', 'best');
grid on;
hold off;

% Z方向
subplot(3,1,3);
hold on;
plot(data.t, data.X_L_save(:,11), '-b', 'LineWidth', 2, 'DisplayName', '实际Z');
plot(data.t, squeeze(data.Des_pL(3,1,:)), '--r', 'LineWidth', 2, 'DisplayName', '期望Z');
title('货物Z方向轨迹对比');
xlabel('时间 (s)');
ylabel('Z (m)');
legend('Location', 'best');
grid on;
hold off;
end

function plot_payload_attitude(data)
%% 绘制货物期望姿态
figure;
quatrotor_plot(data.t, data.Att_L_d(1,:), '-b');
hold on;
quatrotor_plot(data.t, data.Att_L_d(2,:), '-r');
quatrotor_plot(data.t, data.Att_L_d(3,:), '-g');
grid on;
title('货物预期姿态');
legend('预期滚转角\phi', '预期俯仰角\theta', '预期偏航角\psi');
hold off;
end

function plot_UAV_trajectory_2D(data)
%% 绘制无人机2D轨迹（第一架）
figure;
quatrotor_plot(data.t, data.X_UAV_save(:,7,1), '-b');
hold on;
quatrotor_plot(data.t, data.X_UAV_save(:,9,1), '-r');
quatrotor_plot(data.t, data.X_UAV_save(:,11,1), '-g');
grid on;
title('无人机实际轨迹二维图');
legend('X', 'Y', 'Z');
hold off;
end

function plot_UAV_trajectory_3D(data)
%% 绘制无人机3D轨迹（第一架）
figure;
plot3(data.X_UAV_save(:,7,2), data.X_UAV_save(:,9,2), data.X_UAV_save(:,11,2), ...
      'Color', '#0000FF', 'Linewidth', 3);
title('无人机实际三维轨迹');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
end

function plot_all_errors(data)
%% 绘制所有误差图
if ~isfield(data, 'Errors_Save')
    warning('Errors_Save 数据不存在，跳过误差图绘制');
    return;
end

% 货物旋转矩阵误差
figure;
hold on;
plot(data.t, data.Errors_Save.e_RL(:,1), '-b', 'LineWidth', 1.5);
plot(data.t, data.Errors_Save.e_RL(:,2), '-r', 'LineWidth', 1.5);
plot(data.t, data.Errors_Save.e_RL(:,3), '-g', 'LineWidth', 1.5);
title('货物旋转矩阵误差 e\_RL');
xlabel('时间 (s)');
ylabel('误差');
legend('X方向', 'Y方向', 'Z方向', 'Location', 'best');
grid on;
hold off;

% 货物角速度误差
figure;
hold on;
plot(data.t, data.Errors_Save.e_Ome_L(:,1), '-b', 'LineWidth', 1.5);
plot(data.t, data.Errors_Save.e_Ome_L(:,2), '-r', 'LineWidth', 1.5);
plot(data.t, data.Errors_Save.e_Ome_L(:,3), '-g', 'LineWidth', 1.5);
title('货物角速度误差 e\_Ome\_L');
xlabel('时间 (s)');
ylabel('误差 (rad/s)');
legend('X方向', 'Y方向', 'Z方向', 'Location', 'best');
grid on;
hold off;

% 货物位置误差
figure;
hold on;
plot(data.t, data.Errors_Save.e_p_L(:,1), '-b', 'LineWidth', 1.5);
plot(data.t, data.Errors_Save.e_p_L(:,2), '-r', 'LineWidth', 1.5);
plot(data.t, data.Errors_Save.e_p_L(:,3), '-g', 'LineWidth', 1.5);
title('货物位置误差 e\_p\_L');
xlabel('时间 (s)');
ylabel('误差 (m)');
legend('X方向', 'Y方向', 'Z方向', 'Location', 'best');
grid on;
hold off;

% 货物速度误差
figure;
hold on;
plot(data.t, data.Errors_Save.e_v_L(:,1), '-b', 'LineWidth', 1.5);
plot(data.t, data.Errors_Save.e_v_L(:,2), '-r', 'LineWidth', 1.5);
plot(data.t, data.Errors_Save.e_v_L(:,3), '-g', 'LineWidth', 1.5);
title('货物速度误差 e\_v\_L');
xlabel('时间 (s)');
ylabel('误差 (m/s)');
legend('X方向', 'Y方向', 'Z方向', 'Location', 'best');
grid on;
hold off;

% 绳索方向误差（每个无人机）
for q_num = 1:data.Quadrotor_Num
    figure;
    hold on;
    plot(data.t, data.Errors_Save.e_qi(:,1,q_num), '-b', 'LineWidth', 1.5);
    plot(data.t, data.Errors_Save.e_qi(:,2,q_num), '-r', 'LineWidth', 1.5);
    plot(data.t, data.Errors_Save.e_qi(:,3,q_num), '-g', 'LineWidth', 1.5);
    title(sprintf('无人机%d绳索方向误差 e\\_qi', q_num));
    xlabel('时间 (s)');
    ylabel('误差');
    legend('X方向', 'Y方向', 'Z方向', 'Location', 'best');
    grid on;
    hold off;
end

% 绳索角速度误差（每个无人机）
for q_num = 1:data.Quadrotor_Num
    figure;
    hold on;
    plot(data.t, data.Errors_Save.e_sOmegai(:,1,q_num), '-b', 'LineWidth', 1.5);
    plot(data.t, data.Errors_Save.e_sOmegai(:,2,q_num), '-r', 'LineWidth', 1.5);
    plot(data.t, data.Errors_Save.e_sOmegai(:,3,q_num), '-g', 'LineWidth', 1.5);
    title(sprintf('无人机%d绳索角速度误差 e\\_sOmegai', q_num));
    xlabel('时间 (s)');
    ylabel('误差 (rad/s)');
    legend('X方向', 'Y方向', 'Z方向', 'Location', 'best');
    grid on;
    hold off;
end

% 无人机姿态旋转矩阵误差（每个无人机）
for q_num = 1:data.Quadrotor_Num
    figure;
    hold on;
    plot(data.t, data.Errors_Save.e_Ri(:,1,q_num), '-b', 'LineWidth', 1.5);
    plot(data.t, data.Errors_Save.e_Ri(:,2,q_num), '-r', 'LineWidth', 1.5);
    plot(data.t, data.Errors_Save.e_Ri(:,3,q_num), '-g', 'LineWidth', 1.5);
    title(sprintf('无人机%d姿态旋转矩阵误差 e\\_Ri', q_num));
    xlabel('时间 (s)');
    ylabel('误差');
    legend('X方向', 'Y方向', 'Z方向', 'Location', 'best');
    grid on;
    hold off;
end

% 无人机角速度误差（每个无人机）
for q_num = 1:data.Quadrotor_Num
    figure;
    hold on;
    plot(data.t, data.Errors_Save.e_Omegai(:,1,q_num), '-b', 'LineWidth', 1.5);
    plot(data.t, data.Errors_Save.e_Omegai(:,2,q_num), '-r', 'LineWidth', 1.5);
    plot(data.t, data.Errors_Save.e_Omegai(:,3,q_num), '-g', 'LineWidth', 1.5);
    title(sprintf('无人机%d角速度误差 e\\_Omegai', q_num));
    xlabel('时间 (s)');
    ylabel('误差 (rad/s)');
    legend('X方向', 'Y方向', 'Z方向', 'Location', 'best');
    grid on;
    hold off;
end
end
