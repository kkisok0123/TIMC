function plotData = create_plot_data(t, N, Quadrotor_Num, X_UAV_save, X_L_save, Des_pL, Des_Att, Save)
%% 创建绘图数据结构
% 输入参数：
%   t: 时间向量
%   N: 时间步数
%   Quadrotor_Num: 无人机数量
%   X_UAV_save: 无人机状态保存 (N x 12 x Quadrotor_Num)
%   X_L_save: 货物状态保存 (N x 12)
%   Des_pL: 期望位置 (3 x 3 x N)
%   Des_Att: 期望姿态 (3 x 3 x N)，需要提取第一列作为 Att_L_d
%   Save: 保存数据结构体，包含所有控制输入和误差数据
% 输出：
%   plotData: 绘图数据结构体，可直接传递给 plot_result 函数

plotData = struct();
plotData.t = t;
plotData.N = N;
plotData.Quadrotor_Num = Quadrotor_Num;
plotData.X_UAV_save = X_UAV_save;
plotData.X_L_save = X_L_save;
plotData.Des_pL = Des_pL;
plotData.Att_L_d = squeeze(Des_Att(:, 1, :));  % 提取期望姿态角（第一列），结果是 (3 x N)
plotData.omegaid_p_Save = Save.omegaid_p_Save;
plotData.Input_error_all = Save.Input_error_all;
plotData.Fplant_Save = Save.Fplant_Save;
plotData.Fd_Save = Save.Fd_Save;
plotData.Md_Save = Save.Md_Save;
plotData.Fin_Save = Save.Fin_Save;
plotData.Min_Save = Save.Min_Save;
plotData.Us_Save = Save.Us_Save;
plotData.Errors_Save = Save.Errors_Save;

end


