function metrics = compute_revision_metrics(run_result, steady_ratio)
%% 返修版统一指标：位置/姿态误差、旋转角误差、约化张力与分配残差
%
% 时间窗定义为"最后 steady_ratio 比例的时间区间"，同时给出全程汇总，
% 便于正文主表与附录对照。

if nargin < 2 || isempty(steady_ratio)
    steady_ratio = 0.2;
end

metrics = struct();
metrics.steady_ratio = steady_ratio;

t = run_result.time.t(:);
N = numel(t);
errors = run_result.save_data.Errors_Save;

% 时间窗：主表取最后 steady_ratio 比例时间（与旧稿"最后 80% 时间窗"对应）
t_start = t(end) - steady_ratio * (t(end) - t(1));
idx_steady = find(t >= t_start, 1, 'first');
if isempty(idx_steady)
    idx_steady = 1;
end
idx_steady = max(1, min(idx_steady, N));

e_p = errors.e_p_L;
e_att = errors.e_Att_L;

metrics.load_pos_rmse_axis = sqrt(mean(e_p(idx_steady:end, :) .^ 2, 1));
metrics.load_pos_rms = sqrt(mean(sum(e_p(idx_steady:end, :) .^ 2, 2)));
metrics.load_pos_max = max(sqrt(sum(e_p(idx_steady:end, :) .^ 2, 2)));

% 旋转角误差：arccos(clip((tr(Rd'R)-1)/2))
if isfield(errors, 'error_RL_norm_sq')
    theta_err = acos(max(-1, min(1, 1 - 2 * errors.error_RL_norm_sq)));
    metrics.rot_err_rms = sqrt(mean(theta_err(idx_steady:end) .^ 2));
    metrics.rot_err_max = max(theta_err(idx_steady:end));
else
    metrics.rot_err_rms = sqrt(mean(sum(e_att(idx_steady:end, :) .^ 2, 2)));
    metrics.rot_err_max = max(sqrt(sum(e_att(idx_steady:end, :) .^ 2, 2)));
end
metrics.load_att_rmse_axis = sqrt(mean(e_att(idx_steady:end, :) .^ 2, 1));

% 全程汇总
metrics.load_pos_rms_full = sqrt(mean(sum(e_p .^ 2, 2)));
metrics.load_pos_max_full = max(sqrt(sum(e_p .^ 2, 2)));

% 约化模型张力指标（不是实物张力）
if isfield(run_result.save_data, 'Tension_Save')
    tension = run_result.save_data.Tension_Save;
    metrics.tension_min = min(tension(:));
    metrics.tension_min_steady = min(min(tension(idx_steady:end, :)));
    metrics.tension_negative_samples = sum(tension(:) < 0);
    metrics.tension_total_samples = numel(tension);
else
    metrics.tension_min = NaN;
    metrics.tension_min_steady = NaN;
    metrics.tension_negative_samples = NaN;
    metrics.tension_total_samples = NaN;
end

% 分配残差：Fd-Fin、Md-Min
if isfield(run_result.save_data, 'Input_error_all')
    input_error = run_result.save_data.Input_error_all;
    metrics.alloc_force_residual_rms = sqrt(mean(input_error(idx_steady:end, :, 1) .^ 2, 'all'));
    metrics.alloc_moment_residual_rms = sqrt(mean(input_error(idx_steady:end, :, 2) .^ 2, 'all'));
    metrics.alloc_force_residual_max = max(abs(input_error(idx_steady:end, :, 1)), [], 'all');
    metrics.alloc_moment_residual_max = max(abs(input_error(idx_steady:end, :, 2)), [], 'all');
else
    metrics.alloc_force_residual_rms = NaN;
    metrics.alloc_moment_residual_rms = NaN;
    metrics.alloc_force_residual_max = NaN;
    metrics.alloc_moment_residual_max = NaN;
end

% 控制周期耗时（若主循环记录了）
if isfield(run_result, 'timing') && isfield(run_result.timing, 'control_step')
    step = run_result.timing.control_step;
    metrics.control_step_mean_ms = 1e3 * mean(step);
    metrics.control_step_p95_ms = 1e3 * prctile(step, 95);
    metrics.control_step_max_ms = 1e3 * max(step);
else
    metrics.control_step_mean_ms = NaN;
    metrics.control_step_p95_ms = NaN;
    metrics.control_step_max_ms = NaN;
end
end
