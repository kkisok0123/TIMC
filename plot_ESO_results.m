function plot_ESO_results(t, ESO_Save, X_L_save, controller_type)
%% ESO结果绘图函数
% 输入参数：
%   t: 时间向量 (N x 1)
%   ESO_Save: ESO数据结构体，包含：
%       - z1_pos: 估计位置 (N x 3)
%       - z2_pos: 估计速度 (N x 3)
%       - z3_pos: 位置总扰动 (N x 3)
%       - z1_att: 估计姿态 (N x 3)
%       - z2_att: 估计角速度 (N x 3)
%       - z3_att: 姿态总扰动 (N x 3)
%       - Fd_eso_comp: ESO补偿力 (N x 3)
%       - Md_eso_comp: ESO补偿力矩 (N x 3)
%   X_L_save: 实际负载状态 (N x 12)
%   controller_type: 控制器类型字符串（可选，用于标题）

if nargin < 4
    controller_type = 'RBF+ESO';
end

% 提取实际状态
PosL_actual = [X_L_save(:, 7), X_L_save(:, 9), X_L_save(:, 11)];  % X, Y, Z
VelL_actual = [X_L_save(:, 8), X_L_save(:, 10), X_L_save(:, 12)];  % Vx, Vy, Vz
AttL_actual = [X_L_save(:, 1), X_L_save(:, 3), X_L_save(:, 5)];    % Roll, Pitch, Yaw
OmeL_actual = [X_L_save(:, 2), X_L_save(:, 4), X_L_save(:, 6)];    % ωx, ωy, ωz

%% ========== 图1-2: 位置和速度估计对比 ==========
figure('Name', 'ESO位置估计对比', 'Position', [100, 100, 1200, 800]);

% 位置估计对比
subplot(2, 3, 1);
plot(t, PosL_actual(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', '实际X');
hold on;
plot(t, ESO_Save.z1_pos(:, 1), 'r--', 'LineWidth', 2, 'DisplayName', 'ESO估计X');
xlabel('t (s)', 'FontSize', 18);
ylabel('位置 (m)', 'FontSize', 18);
title('X方向位置估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 2);
plot(t, PosL_actual(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '实际Y');
hold on;
plot(t, ESO_Save.z1_pos(:, 2), 'r--', 'LineWidth', 2, 'DisplayName', 'ESO估计Y');
xlabel('t (s)', 'FontSize', 18);
ylabel('位置 (m)', 'FontSize', 18);
title('Y方向位置估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 3);
plot(t, PosL_actual(:, 3), 'b-', 'LineWidth', 2, 'DisplayName', '实际Z');
hold on;
plot(t, ESO_Save.z1_pos(:, 3), 'r--', 'LineWidth', 2, 'DisplayName', 'ESO估计Z');
xlabel('t (s)', 'FontSize', 18);
ylabel('位置 (m)', 'FontSize', 18);
title('Z方向位置估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

% 速度估计对比
subplot(2, 3, 4);
plot(t, VelL_actual(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', '实际Vx');
hold on;
plot(t, ESO_Save.z2_pos(:, 1), 'r--', 'LineWidth', 2, 'DisplayName', 'ESO估计Vx');
xlabel('t (s)', 'FontSize', 18);
ylabel('速度 (m/s)', 'FontSize', 18);
title('X方向速度估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 5);
plot(t, VelL_actual(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '实际Vy');
hold on;
plot(t, ESO_Save.z2_pos(:, 2), 'r--', 'LineWidth', 2, 'DisplayName', 'ESO估计Vy');
xlabel('t (s)', 'FontSize', 18);
ylabel('速度 (m/s)', 'FontSize', 18);
title('Y方向速度估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 6);
plot(t, VelL_actual(:, 3), 'b-', 'LineWidth', 2, 'DisplayName', '实际Vz');
hold on;
plot(t, ESO_Save.z2_pos(:, 3), 'r--', 'LineWidth', 2, 'DisplayName', 'ESO估计Vz');
xlabel('t (s)', 'FontSize', 18);
ylabel('速度 (m/s)', 'FontSize', 18);
title('Z方向速度估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

sgtitle(sprintf('%s控制器 - ESO位置和速度估计对比', controller_type), 'FontSize', 18, 'FontWeight', 'bold');

%% ========== 图3-4: 姿态和角速度估计对比 ==========
figure('Name', 'ESO姿态估计对比', 'Position', [150, 150, 1200, 800]);

% 姿态估计对比
subplot(2, 3, 1);
plot(t, AttL_actual(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', '\phi 实际');
hold on;
plot(t, ESO_Save.z1_att(:, 1), 'r--', 'LineWidth', 2, 'DisplayName', '\phi ESO估计');
xlabel('t (s)', 'FontSize', 18);
ylabel('角度 (rad)', 'FontSize', 18);
title('\phi 姿态估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 2);
plot(t, AttL_actual(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '\theta 实际');
hold on;
plot(t, ESO_Save.z1_att(:, 2), 'r--', 'LineWidth', 2, 'DisplayName', '\theta ESO估计');
xlabel('t (s)', 'FontSize', 18);
ylabel('角度 (rad)', 'FontSize', 18);
title('\theta 姿态估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 3);
plot(t, AttL_actual(:, 3), 'b-', 'LineWidth', 2, 'DisplayName', '\psi 实际');
hold on;
plot(t, ESO_Save.z1_att(:, 3), 'r--', 'LineWidth', 2, 'DisplayName', '\psi ESO估计');
xlabel('t (s)', 'FontSize', 18);
ylabel('角度 (rad)', 'FontSize', 18);
title('\psi 姿态估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

% 角速度估计对比
subplot(2, 3, 4);
plot(t, OmeL_actual(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', '实际ωx');
hold on;
plot(t, ESO_Save.z2_att(:, 1), 'r--', 'LineWidth', 2, 'DisplayName', 'ESO估计ωx');
xlabel('t (s)', 'FontSize', 18);
ylabel('角速度 (rad/s)', 'FontSize', 18);
title('X方向角速度估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 5);
plot(t, OmeL_actual(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '实际ωy');
hold on;
plot(t, ESO_Save.z2_att(:, 2), 'r--', 'LineWidth', 2, 'DisplayName', 'ESO估计ωy');
xlabel('t (s)', 'FontSize', 18);
ylabel('角速度 (rad/s)', 'FontSize', 18);
title('Y方向角速度估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 6);
plot(t, OmeL_actual(:, 3), 'b-', 'LineWidth', 2, 'DisplayName', '实际ωz');
hold on;
plot(t, ESO_Save.z2_att(:, 3), 'r--', 'LineWidth', 2, 'DisplayName', 'ESO估计ωz');
xlabel('t (s)', 'FontSize', 18);
ylabel('角速度 (rad/s)', 'FontSize', 18);
title('Z方向角速度估计', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

sgtitle(sprintf('%s控制器 - ESO姿态和角速度估计对比', controller_type), 'FontSize', 18, 'FontWeight', 'bold');

%% ========== 图5: 位置总扰动估计 ==========
figure('Name', '位置总扰动估计', 'Position', [200, 200, 1000, 600]);
hold on;
plot(t, ESO_Save.z3_pos(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', 'X方向总扰动');
plot(t, ESO_Save.z3_pos(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', 'Y方向总扰动');
plot(t, ESO_Save.z3_pos(:, 3), 'g-', 'LineWidth', 2, 'DisplayName', 'Z方向总扰动');
xlabel('t (s)', 'FontSize', 18);
ylabel('总扰动 (m/s²)', 'FontSize', 18);
title(sprintf('%s控制器 - 位置总扰动估计', controller_type), 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);
hold off;

%% ========== 图6: 姿态总扰动估计 ==========
figure('Name', '姿态总扰动估计', 'Position', [250, 250, 1000, 600]);
hold on;
plot(t, ESO_Save.z3_att(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', '\phi 方向总扰动');
plot(t, ESO_Save.z3_att(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', '\theta 方向总扰动');
plot(t, ESO_Save.z3_att(:, 3), 'g-', 'LineWidth', 2, 'DisplayName', '\psi 方向总扰动');
xlabel('t (s)', 'FontSize', 18);
ylabel('总扰动 (rad/s²)', 'FontSize', 18);
title(sprintf('%s控制器 - 姿态总扰动估计', controller_type), 'FontSize', 18);
legend('Location', 'best', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);
hold off;

%% ========== 图7: 位置估计误差 ==========
figure('Name', 'ESO位置估计误差', 'Position', [300, 300, 1200, 600]);

% 位置估计误差
subplot(2, 3, 1);
pos_error_x = PosL_actual(:, 1) - ESO_Save.z1_pos(:, 1);
plot(t, pos_error_x, 'b-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (m)', 'FontSize', 18);
title('X方向位置估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 2);
pos_error_y = PosL_actual(:, 2) - ESO_Save.z1_pos(:, 2);
plot(t, pos_error_y, 'r-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (m)', 'FontSize', 18);
title('Y方向位置估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 3);
pos_error_z = PosL_actual(:, 3) - ESO_Save.z1_pos(:, 3);
plot(t, pos_error_z, 'g-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (m)', 'FontSize', 18);
title('Z方向位置估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

% 速度估计误差
subplot(2, 3, 4);
vel_error_x = VelL_actual(:, 1) - ESO_Save.z2_pos(:, 1);
plot(t, vel_error_x, 'b-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (m/s)', 'FontSize', 18);
title('X方向速度估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 5);
vel_error_y = VelL_actual(:, 2) - ESO_Save.z2_pos(:, 2);
plot(t, vel_error_y, 'r-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (m/s)', 'FontSize', 18);
title('Y方向速度估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 6);
vel_error_z = VelL_actual(:, 3) - ESO_Save.z2_pos(:, 3);
plot(t, vel_error_z, 'g-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (m/s)', 'FontSize', 18);
title('Z方向速度估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

sgtitle(sprintf('%s控制器 - ESO位置和速度估计误差', controller_type), 'FontSize', 14, 'FontWeight', 'bold');

%% ========== 图8: 姿态估计误差 ==========
figure('Name', 'ESO姿态估计误差', 'Position', [350, 350, 1200, 600]);

% 姿态估计误差
subplot(2, 3, 1);
att_error_roll = AttL_actual(:, 1) - ESO_Save.z1_att(:, 1);
plot(t, att_error_roll, 'b-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (rad)', 'FontSize', 18);
title('\phi 姿态估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 2);
att_error_pitch = AttL_actual(:, 2) - ESO_Save.z1_att(:, 2);
plot(t, att_error_pitch, 'r-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (rad)', 'FontSize', 18);
title('\theta 姿态估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 3);
att_error_yaw = AttL_actual(:, 3) - ESO_Save.z1_att(:, 3);
plot(t, att_error_yaw, 'g-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (rad)', 'FontSize', 18);
title('\psi 姿态估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

% 角速度估计误差
subplot(2, 3, 4);
ome_error_x = OmeL_actual(:, 1) - ESO_Save.z2_att(:, 1);
plot(t, ome_error_x, 'b-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (rad/s)', 'FontSize', 18);
title('X方向角速度估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 5);
ome_error_y = OmeL_actual(:, 2) - ESO_Save.z2_att(:, 2);
plot(t, ome_error_y, 'r-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (rad/s)', 'FontSize', 18);
title('Y方向角速度估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

subplot(2, 3, 6);
ome_error_z = OmeL_actual(:, 3) - ESO_Save.z2_att(:, 3);
plot(t, ome_error_z, 'g-', 'LineWidth', 2);
xlabel('t (s)', 'FontSize', 18);
ylabel('误差 (rad/s)', 'FontSize', 18);
title('Z方向角速度估计误差', 'FontSize', 18);
grid on;
set(gca, 'FontSize', 14);

sgtitle(sprintf('%s控制器 - ESO姿态和角速度估计误差', controller_type), 'FontSize', 18, 'FontWeight', 'bold');

%% ========== 图9-10: ESO补偿项 ==========
figure('Name', 'ESO补偿项', 'Position', [400, 400, 1200, 600]);

% ESO补偿力
subplot(2, 1, 1);
hold on;
plot(t, ESO_Save.Fd_eso_comp(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', 'X方向补偿力');
plot(t, ESO_Save.Fd_eso_comp(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', 'Y方向补偿力');
plot(t, ESO_Save.Fd_eso_comp(:, 3), 'g-', 'LineWidth', 2, 'DisplayName', 'Z方向补偿力');
xlabel('t (s)', 'FontSize', 18);
ylabel('补偿力 (N)', 'FontSize', 18);
title('负载ESO位置补偿力', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 14);
hold off;

% ESO补偿力矩
subplot(2, 1, 2);
hold on;
plot(t, ESO_Save.Md_eso_comp(:, 1), 'b-', 'LineWidth', 2, 'DisplayName', '\phi 方向补偿力矩');
plot(t, ESO_Save.Md_eso_comp(:, 2), 'r-', 'LineWidth', 2, 'DisplayName', '\theta 方向补偿力矩');
plot(t, ESO_Save.Md_eso_comp(:, 3), 'g-', 'LineWidth', 2, 'DisplayName', '\psi 方向补偿力矩');
xlabel('t (s)', 'FontSize', 18);
ylabel('补偿力矩 (N·m)', 'FontSize', 18);
title('负载ESO姿态补偿力矩', 'FontSize', 18);
legend('Location', 'best', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 14);
hold off;

fprintf('ESO结果绘图完成！\n');

end
