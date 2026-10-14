function metrics = compute_run_steady_metrics(run_result, steady_ratio)
% Compute steady-state RMSE and max metrics for load and UAV errors.

if nargin < 2 || isempty(steady_ratio)
    steady_ratio = 0.2;
end

takeoff_iter = 0;
if isfield(run_result, 'meta') && isfield(run_result.meta, 'takeoff_iter')
    takeoff_iter = run_result.meta.takeoff_iter;
end

plot_start = takeoff_iter + 1;
errors = run_result.save_data.Errors_Save;

metrics = struct();
metrics.load_pos_rmse = nan(1, 3);
metrics.load_att_rmse = nan(1, 3);
metrics.load_pos_max = nan(1, 3);
metrics.load_att_max = nan(1, 3);
metrics.uav_pos_rmse = nan(1, 3);
metrics.uav_att_rmse = nan(1, 3);
metrics.uav_pos_max = nan(1, 3);
metrics.uav_att_max = nan(1, 3);
metrics.rope_rmse_per_uav = nan(1, 1);
metrics.rope_rmse_avg = nan;

if ~isfield(errors, 'e_p_L') || ~isfield(errors, 'e_Att_L')
    return;
end

e_p_L = errors.e_p_L(plot_start:end, :);
e_Att_L = errors.e_Att_L(plot_start:end, :);
steady_start = max(1, floor(steady_ratio * size(e_p_L, 1)));
steady_indices = steady_start:size(e_p_L, 1);

e_p_L_steady = e_p_L(steady_indices, :);
e_Att_L_steady = e_Att_L(steady_indices, :);

metrics.load_pos_rmse = sqrt(mean(e_p_L_steady .^ 2, 1));
metrics.load_att_rmse = sqrt(mean(e_Att_L_steady .^ 2, 1));
metrics.load_pos_max = max(abs(e_p_L_steady), [], 1);
metrics.load_att_max = max(abs(e_Att_L_steady), [], 1);
metrics.load_metric_vector = [metrics.load_pos_rmse, metrics.load_att_rmse];

if isfield(errors, 'e_p_UAV')
    e_p_UAV = errors.e_p_UAV(plot_start:end, :, :);
    e_p_UAV_steady = e_p_UAV(steady_indices, :, :);
    pos_samples = reshape(permute(e_p_UAV_steady, [1, 3, 2]), [], 3);
    metrics.uav_pos_rmse = sqrt(mean(pos_samples .^ 2, 1));
    metrics.uav_pos_max = max(abs(pos_samples), [], 1);
end

if isfield(errors, 'UAV_e_Att')
    UAV_e_Att = errors.UAV_e_Att(plot_start:end, :, :);
    UAV_e_Att_steady = UAV_e_Att(steady_indices, :, :);
    att_samples = reshape(permute(UAV_e_Att_steady, [1, 3, 2]), [], 3);
    metrics.uav_att_rmse = sqrt(mean(att_samples .^ 2, 1));
    metrics.uav_att_max = max(abs(att_samples), [], 1);
end

if isfield(errors, 'error_qi_norm_sq')
    rope_err_sq = errors.error_qi_norm_sq(plot_start:end, :);
    rope_err_sq = rope_err_sq(steady_indices, :);
    metrics.rope_rmse_per_uav = sqrt(mean(max(2 * rope_err_sq, 0), 1));
    metrics.rope_rmse_avg = mean(metrics.rope_rmse_per_uav);
end

metrics.uav_metric_vector = [metrics.uav_pos_rmse, metrics.uav_att_rmse];
end
