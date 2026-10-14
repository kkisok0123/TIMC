function plot_load_lifting_comparison(run_output_dir, plot_options)
if nargin < 1 || isempty(run_output_dir)
    run_output_dir = fullfile(pwd, 'simulation_runs');
end
if nargin < 2 || isempty(plot_options)
    plot_options = default_plot_options();
else
    plot_options = merge_plot_options(plot_options);
end

graph_output_dir = 'D:\lastVersion\final';
if ~exist(graph_output_dir, 'dir')
    mkdir(graph_output_dir);
end

run_files = dir(fullfile(run_output_dir, 'simulation_run_*.mat'));
if isempty(run_files)
    error('未在目录 %s 中找到 simulation_run_*.mat 文件。', run_output_dir);
end

runs = load_and_sort_runs(run_files);
group_no_disturbance = runs(~[runs.with_disturbance]);
group_with_disturbance = runs([runs.with_disturbance]);

if plot_options.plotAttitudeError
    plot_attitude_group(group_no_disturbance, 'NoDisturbance', graph_output_dir);
    plot_attitude_group(group_with_disturbance, 'WithDisturbance', graph_output_dir);
end

if plot_options.plotPositionError
    plot_position_group(group_no_disturbance, 'NoDisturbance', graph_output_dir);
    plot_position_group(group_with_disturbance, 'WithDisturbance', graph_output_dir);
end

if plot_options.plotRLNorm
    plot_rl_norm_group(group_no_disturbance, 'NoDisturbance', graph_output_dir);
    plot_rl_norm_group(group_with_disturbance, 'WithDisturbance', graph_output_dir);
end

if plot_options.plotRmsMetrics
    plot_rms_group(group_no_disturbance, 'NoDisturbance', graph_output_dir);
    plot_rms_group(group_with_disturbance, 'WithDisturbance', graph_output_dir);
end

fprintf('\nComparison groups loaded from %s\n', run_output_dir);
fprintf('  NoDisturbance cases: %d\n', numel(group_no_disturbance));
fprintf('  WithDisturbance cases: %d\n', numel(group_with_disturbance));
fprintf('  Graph folder: %s\n', graph_output_dir);
end

function runs = load_and_sort_runs(run_files)
controller_order = {'pd', 'backstepping', 'rbf_backstepping', 'rbf_rise', 'smc', 'rise'};
runs = repmat(struct('controller_type', '', 'with_disturbance', false, ...
    'label', '', 't', [], 'e_att', [], 'e_pos', [], 'rl_norm', [], ...
    'roll_rms', 0, 'pitch_rms', 0, 'pos_norm_rms', 0), numel(run_files), 1);

for k = 1:numel(run_files)
    data = load(fullfile(run_files(k).folder, run_files(k).name), 'run_result');
    run_result = data.run_result;
    plot_start_idx = run_result.meta.takeoff_iter + 1;

    runs(k).controller_type = normalize_controller_type(run_result, run_files(k).name);
    runs(k).with_disturbance = logical(run_result.meta.with_disturbance);
    runs(k).label = get_controller_label(run_result, run_files(k).name);
    runs(k).t = run_result.time.t_extended(plot_start_idx:end);
    runs(k).e_att = run_result.save_data.Errors_Save.e_Att_L(plot_start_idx:end, :);
    runs(k).e_pos = run_result.save_data.Errors_Save.e_p_L(plot_start_idx:end, :);
    runs(k).rl_norm = run_result.save_data.Errors_Save.error_RL_norm_sq(plot_start_idx:end);

    pos_norm = sqrt(sum(runs(k).e_pos.^2, 2));
    runs(k).roll_rms = sqrt(mean(runs(k).e_att(:, 1).^2));
    runs(k).pitch_rms = sqrt(mean(runs(k).e_att(:, 2).^2));
    runs(k).pos_norm_rms = sqrt(mean(pos_norm.^2));
end

sort_keys = zeros(numel(runs), 1);
for k = 1:numel(runs)
    ctrl_idx = find(strcmpi(runs(k).controller_type, controller_order), 1);
    if isempty(ctrl_idx)
        ctrl_idx = numel(controller_order) + 1;
    end
    sort_keys(k) = ctrl_idx;
end
[~, sort_idx] = sort(sort_keys);
runs = runs(sort_idx);
end

function plot_attitude_group(group_runs, group_name, run_output_dir)
if isempty(group_runs)
    return;
end

colors = lines(numel(group_runs));
fig = figure('Name', ['Comparison_Attitude_' group_name], 'Position', [80, 80, 1200, 700]);

subplot(2, 1, 1);
hold on;
for k = 1:numel(group_runs)
    plot(group_runs(k).t, group_runs(k).e_att(:, 1), 'LineWidth', 1.2, ...
        'Color', colors(k, :), 'DisplayName', group_runs(k).label);
end
grid on;
xlabel('Time (s)');
ylabel('eRoll (rad)');
title(['Payload Roll Error Comparison - ' group_name]);
legend('Location', 'eastoutside');

subplot(2, 1, 2);
hold on;
for k = 1:numel(group_runs)
    plot(group_runs(k).t, group_runs(k).e_att(:, 2), 'LineWidth', 1.2, ...
        'Color', colors(k, :), 'DisplayName', group_runs(k).label);
end
grid on;
xlabel('Time (s)');
ylabel('ePitch (rad)');
title(['Payload Pitch Error Comparison - ' group_name]);

saveas(fig, fullfile(run_output_dir, ['comparison_attitude_' group_name '.png']));
end

function plot_position_group(group_runs, group_name, run_output_dir)
if isempty(group_runs)
    return;
end

colors = lines(numel(group_runs));
fig = figure('Name', ['Comparison_Position_' group_name], 'Position', [120, 120, 1200, 900]);
axis_labels = {'ePosX (m)', 'ePosY (m)', 'ePosZ (m)'};
titles = {'Payload Position Error X', 'Payload Position Error Y', 'Payload Position Error Z'};

for axis_idx = 1:3
    subplot(3, 1, axis_idx);
    hold on;
    for k = 1:numel(group_runs)
        plot(group_runs(k).t, group_runs(k).e_pos(:, axis_idx), 'LineWidth', 1.2, ...
            'Color', colors(k, :), 'DisplayName', group_runs(k).label);
    end
    grid on;
    xlabel('Time (s)');
    ylabel(axis_labels{axis_idx});
    title([titles{axis_idx} ' - ' group_name]);
    if axis_idx == 1
        legend('Location', 'eastoutside');
    end
end

saveas(fig, fullfile(run_output_dir, ['comparison_position_' group_name '.png']));
end

function plot_rl_norm_group(group_runs, group_name, run_output_dir)
if isempty(group_runs)
    return;
end

colors = lines(numel(group_runs));
fig = figure('Name', ['Comparison_RLNorm_' group_name], 'Position', [160, 160, 1200, 500]);
hold on;
for k = 1:numel(group_runs)
    plot(group_runs(k).t, group_runs(k).rl_norm, 'LineWidth', 1.2, ...
        'Color', colors(k, :), 'DisplayName', group_runs(k).label);
end
grid on;
xlabel('Time (s)');
ylabel('1/2||R_L-R_{L,d}||^2');
title(['Payload Rotation Error Function Comparison - ' group_name]);
legend('Location', 'eastoutside');

saveas(fig, fullfile(run_output_dir, ['comparison_rl_norm_' group_name '.png']));
end

function plot_rms_group(group_runs, group_name, run_output_dir)
if isempty(group_runs)
    return;
end

fig = figure('Name', ['Comparison_RMS_' group_name], 'Position', [200, 200, 1200, 500]);
metric_matrix = [[group_runs.roll_rms]', [group_runs.pitch_rms]', [group_runs.pos_norm_rms]'];
bar(metric_matrix);
grid on;
set(gca, 'XTick', 1:numel(group_runs), 'XTickLabel', {group_runs.label}, 'XTickLabelRotation', 25);
legend({'Roll RMS', 'Pitch RMS', 'Pos Norm RMS'}, 'Location', 'best');
title(['RMS Metric Comparison - ' group_name]);
ylabel('Magnitude');

saveas(fig, fullfile(run_output_dir, ['comparison_rms_' group_name '.png']));
end

function options = default_plot_options()
options = struct();
options.plotAttitudeError = true;
options.plotPositionError = true;
options.plotRLNorm = true;
options.plotRmsMetrics = true;
end

function options = merge_plot_options(input_options)
options = default_plot_options();
field_names = fieldnames(input_options);
for k = 1:numel(field_names)
    options.(field_names{k}) = input_options.(field_names{k});
end
end

function controller_type = normalize_controller_type(run_result, filename)
controller_type = lower(run_result.meta.controller_type);
if strcmpi(controller_type, 'rbf')
    label = get_controller_label(run_result, filename);
    if strcmpi(label, 'RBF+Backstepping')
        controller_type = 'rbf_backstepping';
    else
        controller_type = 'rbf_rise';
    end
end
end

function label = get_controller_label(run_result, filename)
label = '';
if isfield(run_result.meta, 'controller_name') && ~isempty(run_result.meta.controller_name)
    label = run_result.meta.controller_name;
end

if strcmpi(run_result.meta.controller_type, 'rbf')
    if strcmpi(label, 'RBF')
        label = 'RBF+Backstepping';
    elseif isempty(label)
        if contains(filename, 'simulation_run_RBF+RISE_', 'IgnoreCase', true)
            label = 'RBF+RISE';
        else
            label = 'RBF+Backstepping';
        end
    end
end

if isempty(label)
    label = upper(run_result.meta.controller_type);
end
end
