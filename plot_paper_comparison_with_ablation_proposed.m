function outputs = plot_paper_comparison_with_ablation_proposed()
%PLOT_PAPER_COMPARISON_WITH_ABLATION_PROPOSED Plot paper comparisons only.
%
% Uses previously saved paper data for PD/Backstepping/SMC/RISE and replaces
% the Proposed curve with the latest Backstepping+RBF+ESO ablation run.

clc;
close all;

root_dir = fileparts(mfilename('fullpath'));
if isempty(root_dir)
    root_dir = pwd;
end

paper_data_dir = fullfile(root_dir, 'paper_new', 'data');
ablation_data_dir = fullfile(root_dir, 'paper_new', 'backstepping_ablation_data');
graph_dir = fullfile(root_dir, 'paper_new', 'graph_ablation_proposed');
ensure_dir(graph_dir);

set_english_figure_defaults();

paper_no_disturbance = build_group_data(paper_data_dir, ablation_data_dir, false);
paper_with_disturbance = build_group_data(paper_data_dir, ablation_data_dir, true);

save(fullfile(paper_data_dir, 'comparison_group_noDisturbance_ablation_proposed.mat'), ...
    'paper_no_disturbance', '-v7.3');
save(fullfile(paper_data_dir, 'comparison_group_withDisturbance_ablation_proposed.mat'), ...
    'paper_with_disturbance', '-v7.3');

plot_comparison_figures(paper_no_disturbance, 'No Disturbance', ...
    fullfile(graph_dir, 'comparison_no_disturbance'));
plot_comparison_figures(paper_with_disturbance, 'With Disturbance', ...
    fullfile(graph_dir, 'comparison_with_disturbance'));

outputs = struct();
outputs.graph_dir = graph_dir;
outputs.paper_no_disturbance = paper_no_disturbance;
outputs.paper_with_disturbance = paper_with_disturbance;

fprintf('\nComparison figures regenerated with ablation Proposed data.\n');
fprintf('Graph folder: %s\n', graph_dir);
end

function set_english_figure_defaults()
set(groot, 'defaultFigureColor', 'w');
set(groot, 'defaultAxesFontName', 'Arial');
set(groot, 'defaultTextFontName', 'Arial');
set(groot, 'defaultAxesFontSize', 12);
set(groot, 'defaultTextInterpreter', 'none');
set(groot, 'defaultAxesTickLabelInterpreter', 'none');
set(groot, 'defaultLegendInterpreter', 'none');
end

function group_data = build_group_data(paper_data_dir, ablation_data_dir, with_disturbance)
disturbance_tag = ternary(with_disturbance, 'withDisturbance', 'noDisturbance');
labels = {'PD', 'Backstepping', 'SMC', 'RISE'};
group_data = cell(numel(labels) + 1, 1);

for idx = 1:numel(labels)
    compact_file = fullfile(paper_data_dir, sprintf('paper_data_%s_%s.mat', ...
        labels{idx}, disturbance_tag));
    loaded = load(compact_file, 'case_data');
    group_data{idx} = loaded.case_data;
end

ablation_file = fullfile(ablation_data_dir, sprintf( ...
    'simulation_run_Backstepping+RBF+ESO_uavF_BACKSTEPPING_uavM_BACKSTEPPING_%s.mat', ...
    disturbance_tag));
loaded = load(ablation_file, 'run_result');
group_data{numel(labels) + 1} = extract_paper_case_data(loaded.run_result, 'Proposed');
end

function case_data = extract_paper_case_data(run_result, display_label)
plot_start = 1;
if isfield(run_result.meta, 'takeoff_iter')
    plot_start = run_result.meta.takeoff_iter + 1;
end

t = run_result.time.t_extended(:);
plot_start = max(1, min(plot_start, numel(t)));
idx = plot_start:numel(t);

states_uav = normalize_x_uav(run_result.states.X_UAV);
states_load = normalize_x_load(run_result.states.X_L);
save_data = run_result.save_data;
errors = save_data.Errors_Save;

load_pos_actual = states_load(idx, [7, 9, 11]);
load_att_actual = states_load(idx, [1, 3, 5]);
load_pos_error = normalize_nx3(errors.e_p_L);
load_pos_error = load_pos_error(idx, :);
load_att_error = normalize_nx3(errors.e_Att_L);
load_att_error = load_att_error(idx, :);

if isfield(errors, 'Att_L_desired')
    load_att_desired = normalize_nx3(errors.Att_L_desired);
    load_att_desired = load_att_desired(idx, :);
else
    load_att_desired = load_att_actual - load_att_error;
end
load_pos_desired = load_pos_actual - load_pos_error;

rotation_error = [];
if isfield(errors, 'error_RL_norm_sq')
    rotation_error = errors.error_RL_norm_sq(:);
    rotation_error = rotation_error(idx);
end

rope_error = [];
rope_mean_error = [];
if isfield(errors, 'error_qi_norm_sq')
    rope_error = errors.error_qi_norm_sq;
    if size(rope_error, 1) ~= numel(t) && size(rope_error, 2) == numel(t)
        rope_error = rope_error.';
    end
    rope_error = rope_error(idx, :);
    rope_mean_error = row_mean_omitnan(rope_error);
end

uav_pos_actual = states_uav(idx, [7, 9, 11], :);
uav_att_actual = states_uav(idx, [1, 3, 5], :);
uav_pos_desired = get_optional_3d(save_data, 'Des_Pos_UAV_Save', idx);
uav_pos_error = get_optional_3d(errors, 'e_p_UAV', idx);
uav_att_desired = get_optional_3d(errors, 'UAV_Att_desired', idx);
uav_att_error = get_optional_3d(errors, 'UAV_e_Att', idx);

if isempty(uav_pos_error) && ~isempty(uav_pos_desired)
    uav_pos_error = uav_pos_actual - uav_pos_desired;
end
if isempty(uav_att_desired) && ~isempty(uav_att_error)
    uav_att_desired = uav_att_actual - uav_att_error;
end
if isempty(uav_att_error) && ~isempty(uav_att_desired)
    uav_att_error = uav_att_actual - uav_att_desired;
end

us = get_optional_3d(save_data, 'Us_Save', idx);
f_plant = get_optional_3d(save_data, 'F_Plant_Save', idx);

case_data = struct();
case_data.controller_type = 'rbf';
case_data.controller_label = display_label;
case_data.with_disturbance = logical(run_result.meta.with_disturbance);
case_data.disturbance_label = ternary(case_data.with_disturbance, 'With Disturbance', 'No Disturbance');
case_data.disturbance_tag = ternary(case_data.with_disturbance, 'withDisturbance', 'noDisturbance');
case_data.case_tag = sanitize_filename(display_label);
case_data.t = t(idx);
case_data.load_pos_actual = load_pos_actual;
case_data.load_pos_desired = load_pos_desired;
case_data.load_pos_error = load_pos_error;
case_data.load_att_actual = load_att_actual;
case_data.load_att_desired = load_att_desired;
case_data.load_att_error = load_att_error;
case_data.rotation_error = rotation_error;
case_data.rope_error = rope_error;
case_data.rope_mean_error = rope_mean_error;
case_data.uav_pos_actual = uav_pos_actual;
case_data.uav_pos_desired = uav_pos_desired;
case_data.uav_pos_error = uav_pos_error;
case_data.uav_att_actual = uav_att_actual;
case_data.uav_att_desired = uav_att_desired;
case_data.uav_att_error = uav_att_error;
case_data.us = us;
case_data.f_plant = f_plant;
case_data.metrics = run_result.metrics;

if isfield(save_data, 'ESO_Save')
    case_data.eso = slice_eso(save_data.ESO_Save, idx);
else
    case_data.eso = [];
end
end

function data = get_optional_3d(parent, field_name, idx)
data = [];
if ~isfield(parent, field_name)
    return;
end
data = normalize_nx3q(parent.(field_name));
data = data(idx, :, :);
end

function eso_out = slice_eso(eso_in, idx)
eso_out = struct();
fields = fieldnames(eso_in);
for k = 1:numel(fields)
    value = eso_in.(fields{k});
    if isnumeric(value) && ~isempty(value) && size(value, 1) >= max(idx)
        eso_out.(fields{k}) = value(idx, :, :);
    else
        eso_out.(fields{k}) = value;
    end
end
end

function plot_comparison_figures(group_data, condition_label, out_dir)
ensure_dir(out_dir);
if isempty(group_data)
    return;
end
group_data = sort_group_data(group_data);

plot_comparison_payload_trajectory(group_data, condition_label, out_dir);
plot_comparison_three_axes(group_data, condition_label, 'load_pos_actual', 'load_pos_desired', ...
    {'x (m)', 'y (m)', 'z (m)'}, 'Load Position Tracking', out_dir, 'comparison_load_position_tracking.png');
plot_comparison_error_three_axes(group_data, condition_label, 'load_pos_error', ...
    {'e_x (m)', 'e_y (m)', 'e_z (m)'}, 'Load Position Error', out_dir, 'comparison_load_position_error.png');
plot_comparison_three_axes(group_data, condition_label, 'load_att_actual', 'load_att_desired', ...
    {'roll (rad)', 'pitch (rad)', 'yaw (rad)'}, 'Load Attitude Tracking', out_dir, 'comparison_load_attitude_tracking.png');
plot_comparison_error_three_axes(group_data, condition_label, 'load_att_error', ...
    {'e_roll (rad)', 'e_pitch (rad)', 'e_yaw (rad)'}, 'Load Attitude Error', out_dir, 'comparison_load_attitude_error.png');
plot_comparison_single(group_data, condition_label, 'rotation_error', ...
    'Rotation Error Function', '0.5||R_L - R_{L,d}||^2', out_dir, 'comparison_rotation_error_function.png');
plot_comparison_single(group_data, condition_label, 'rope_mean_error', ...
    'Mean Rope Attitude Error', 'Mean rope attitude error', out_dir, 'comparison_mean_rope_attitude_error.png');
plot_comparison_rope_per_uav(group_data, condition_label, out_dir);
plot_comparison_norm(group_data, condition_label, 'uav_pos_error', ...
    'Mean UAV Position Error Norm', 'Mean ||e_p|| (m)', out_dir, 'comparison_mean_uav_position_error_norm.png');
plot_comparison_norm(group_data, condition_label, 'uav_att_error', ...
    'Mean UAV Attitude Error Norm', 'Mean ||e_att|| (rad)', out_dir, 'comparison_mean_uav_attitude_error_norm.png');
plot_summary_bars(group_data, condition_label, out_dir);
end

function group_data = sort_group_data(group_data)
order = {'pd', 'backstepping', 'rbf', 'smc', 'rise'};
keys = zeros(numel(group_data), 1);
for k = 1:numel(group_data)
    idx = find(strcmpi(group_data{k}.controller_type, order), 1);
    if isempty(idx)
        idx = numel(order) + 1;
    end
    keys(k) = idx;
end
[~, sort_idx] = sort(keys);
group_data = group_data(sort_idx);
end

function plot_comparison_payload_trajectory(group_data, condition_label, out_dir)
fig = new_fig('Load 3D Trajectory Comparison', [100, 100, 1000, 760]);
plot3(group_data{1}.load_pos_desired(:, 1), group_data{1}.load_pos_desired(:, 2), group_data{1}.load_pos_desired(:, 3), ...
    'k--', 'LineWidth', 2, 'DisplayName', 'Desired');
hold on;
colors = lines(numel(group_data));
for k = 1:numel(group_data)
    plot3(group_data{k}.load_pos_actual(:, 1), group_data{k}.load_pos_actual(:, 2), group_data{k}.load_pos_actual(:, 3), ...
        'LineWidth', 1.6, 'Color', colors(k, :), 'DisplayName', group_data{k}.controller_label);
end
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
title(sprintf('%s | Load 3D Trajectory Comparison', condition_label));
legend('Location', 'best');
view(45, 25);
export_fig(fig, fullfile(out_dir, 'comparison_load_trajectory_3d.png'));
end

function plot_comparison_three_axes(group_data, condition_label, actual_field, desired_field, ylabels, title_prefix, out_dir, filename)
fig = new_fig(title_prefix, [100, 100, 1250, 850]);
colors = lines(numel(group_data));
for axis_idx = 1:3
    subplot(3, 1, axis_idx);
    hold on;
    plot(group_data{1}.t, group_data{1}.(desired_field)(:, axis_idx), ...
        'k--', 'LineWidth', 1.8, 'DisplayName', 'Desired');
    for k = 1:numel(group_data)
        plot(group_data{k}.t, group_data{k}.(actual_field)(:, axis_idx), ...
            'LineWidth', 1.4, 'Color', colors(k, :), 'DisplayName', group_data{k}.controller_label);
    end
    grid on;
    xlabel('Time (s)');
    ylabel(ylabels{axis_idx});
    title(sprintf('%s - Axis %d', title_prefix, axis_idx));
    if axis_idx == 1
        legend('Location', 'eastoutside');
    end
end
sgtitle(sprintf('%s | %s', condition_label, title_prefix));
export_fig(fig, fullfile(out_dir, filename));
end

function plot_comparison_error_three_axes(group_data, condition_label, error_field, ylabels, title_prefix, out_dir, filename)
fig = new_fig(title_prefix, [100, 100, 1250, 850]);
colors = lines(numel(group_data));
for axis_idx = 1:3
    subplot(3, 1, axis_idx);
    hold on;
    for k = 1:numel(group_data)
        data = group_data{k}.(error_field);
        if isempty(data)
            continue;
        end
        plot(group_data{k}.t, data(:, axis_idx), 'LineWidth', 1.4, ...
            'Color', colors(k, :), 'DisplayName', group_data{k}.controller_label);
    end
    grid on;
    xlabel('Time (s)');
    ylabel(ylabels{axis_idx});
    title(sprintf('%s - Axis %d', title_prefix, axis_idx));
    if axis_idx == 1
        legend('Location', 'eastoutside');
    end
end
sgtitle(sprintf('%s | %s', condition_label, title_prefix));
export_fig(fig, fullfile(out_dir, filename));
end

function plot_comparison_single(group_data, condition_label, field_name, title_prefix, ylabel_text, out_dir, filename)
fig = new_fig(title_prefix, [100, 100, 1150, 620]);
colors = lines(numel(group_data));
hold on;
for k = 1:numel(group_data)
    y = group_data{k}.(field_name);
    if isempty(y)
        continue;
    end
    plot(group_data{k}.t, y, 'LineWidth', 1.5, 'Color', colors(k, :), ...
        'DisplayName', group_data{k}.controller_label);
end
grid on;
xlabel('Time (s)');
ylabel(ylabel_text);
title(sprintf('%s | %s', condition_label, title_prefix));
legend('Location', 'eastoutside');
export_fig(fig, fullfile(out_dir, filename));
end

function plot_comparison_rope_per_uav(group_data, condition_label, out_dir)
if isempty(group_data{1}.rope_error)
    return;
end
q_num = size(group_data{1}.rope_error, 2);
fig = new_fig('Rope Attitude Error Per UAV', [100, 100, 1350, 900]);
colors = lines(numel(group_data));
for q_idx = 1:q_num
    subplot(q_num, 1, q_idx);
    hold on;
    for k = 1:numel(group_data)
        if isempty(group_data{k}.rope_error) || size(group_data{k}.rope_error, 2) < q_idx
            continue;
        end
        plot(group_data{k}.t, group_data{k}.rope_error(:, q_idx), ...
            'LineWidth', 1.3, 'Color', colors(k, :), 'DisplayName', group_data{k}.controller_label);
    end
    grid on;
    xlabel('Time (s)');
    ylabel(sprintf('UAV%d', q_idx));
    title(sprintf('Rope Attitude Error - UAV%d', q_idx));
    if q_idx == 1
        legend('Location', 'eastoutside');
    end
end
sgtitle(sprintf('%s | Rope Attitude Error Per UAV', condition_label));
export_fig(fig, fullfile(out_dir, 'comparison_rope_attitude_error_per_uav.png'));
end

function plot_comparison_norm(group_data, condition_label, field_name, title_prefix, ylabel_text, out_dir, filename)
fig = new_fig(title_prefix, [100, 100, 1150, 620]);
colors = lines(numel(group_data));
hold on;
for k = 1:numel(group_data)
    data = group_data{k}.(field_name);
    if isempty(data)
        continue;
    end
    norm_series = squeeze(sqrt(sum(data.^2, 2)));
    if ismatrix(norm_series) && size(norm_series, 2) > 1
        norm_series = row_mean_omitnan(norm_series);
    end
    plot(group_data{k}.t, norm_series, 'LineWidth', 1.5, 'Color', colors(k, :), ...
        'DisplayName', group_data{k}.controller_label);
end
grid on;
xlabel('Time (s)');
ylabel(ylabel_text);
title(sprintf('%s | %s', condition_label, title_prefix));
legend('Location', 'eastoutside');
export_fig(fig, fullfile(out_dir, filename));
end

function plot_summary_bars(group_data, condition_label, out_dir)
labels = cellfun(@(d) d.controller_label, group_data, 'UniformOutput', false);
metrics = zeros(numel(group_data), 5);
for k = 1:numel(group_data)
    metrics(k, 1) = rms_vector(group_data{k}.load_pos_error);
    metrics(k, 2) = rms_vector(group_data{k}.load_att_error);
    metrics(k, 3) = rms_vector(group_data{k}.rope_mean_error);
    metrics(k, 4) = rms_vector(group_data{k}.uav_pos_error);
    metrics(k, 5) = rms_vector(group_data{k}.uav_att_error);
end

fig = new_fig('RMS Summary', [100, 100, 1200, 620]);
bar(metrics);
grid on;
set(gca, 'XTick', 1:numel(labels), 'XTickLabel', labels, 'XTickLabelRotation', 20);
ylabel('RMS value');
title(sprintf('%s | RMS Summary', condition_label));
legend({'Load Position', 'Load Attitude', 'Mean Rope', 'UAV Position', 'UAV Attitude'}, ...
    'Location', 'eastoutside');
export_fig(fig, fullfile(out_dir, 'comparison_rms_summary.png'));
end

function value = rms_vector(data)
if isempty(data)
    value = NaN;
    return;
end
data = data(:);
data = data(~isnan(data));
if isempty(data)
    value = NaN;
else
    value = sqrt(mean(data.^2));
end
end

function values = row_mean_omitnan(data)
if isempty(data)
    values = [];
    return;
end
valid = ~isnan(data);
data(~valid) = 0;
counts = sum(valid, 2);
values = sum(data, 2) ./ max(counts, 1);
values(counts == 0) = NaN;
end

function X = normalize_x_load(X)
if size(X, 2) == 12
    return;
elseif size(X, 1) == 12
    X = X.';
else
    error('Unsupported load state shape.');
end
end

function X = normalize_x_uav(X)
if ndims(X) ~= 3
    error('Unexpected UAV state dimensions.');
end
if size(X, 2) == 12
    return;
elseif size(X, 1) == 12
    X = permute(X, [2, 1, 3]);
else
    error('Unsupported UAV state shape.');
end
end

function A = normalize_nx3(A)
if isempty(A)
    return;
end
if size(A, 2) == 3
    return;
elseif size(A, 1) == 3
    A = A.';
else
    error('Expected Nx3 or 3xN array.');
end
end

function A = normalize_nx3q(A)
if isempty(A)
    return;
end
if ndims(A) ~= 3
    error('Expected a 3D array.');
end
if size(A, 2) == 3
    return;
elseif size(A, 1) == 3
    A = permute(A, [3, 1, 2]);
elseif size(A, 3) == 3
    A = permute(A, [1, 3, 2]);
else
    error('Unsupported Nx3xQ array shape.');
end
end

function fig = new_fig(name, position)
fig = figure('Visible', 'off', 'Name', name, 'Color', 'w', 'Position', position);
end

function export_fig(fig, file_path)
drawnow;
try
    exportgraphics(fig, file_path, 'Resolution', 220);
catch
    saveas(fig, file_path);
end
close(fig);
end

function ensure_dir(dir_path)
if ~exist(dir_path, 'dir')
    mkdir(dir_path);
end
end

function text = sanitize_filename(text)
text = char(text);
text = regexprep(text, '[^\w\-]+', '_');
text = regexprep(text, '_+', '_');
text = regexprep(text, '^_|_$', '');
end

function out = ternary(condition, true_value, false_value)
if condition
    out = true_value;
else
    out = false_value;
end
end
