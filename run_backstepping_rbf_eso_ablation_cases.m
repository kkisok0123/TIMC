%% Backstepping / RBF / ESO ablation comparison
% 对比涉及 ESO 的两组负载控制器：
%   1) Backstepping + RBF + ESO
%   2) Backstepping + ESO
%
% RBF/ESO 相关案例会读取
%   负载参数_Backstepping+RBF+ESO_无干扰_当前最优.txt
%   负载参数_Backstepping+RBF+ESO_有干扰_当前最优.txt
% 作为基准参数，再只切换 RBF/ESO 消融开关。

clear;
clc;

% 默认先对比无扰动四组；如需同时生成有扰动结果，改为 [false, true]。
disturbance_options = [false, true];

output_dir = fullfile(pwd, 'paper_new', 'backstepping_ablation_data');
graph_output_dir = fullfile(pwd, 'paper_new', 'backstepping_ablation_figures');
source_data_dirs = { ...
    output_dir, ...
    fullfile(pwd, 'paper_new', 'data'), ...
    fullfile(pwd, 'simulation_runs')};

run_options = struct();
run_options.run_output_dir = output_dir;
run_options.save_run_data = true;
run_options.enable_default_plots = false;
run_options.uav_controller_type = 'backstepping';
% 参数来源改为当前最优 txt 后，必须重跑，避免复用旧 MAT 中的默认 ESO 参数。
run_options.force_rerun = false;

plot_options = struct();
plot_options.plotAttitudeError = true;
plot_options.plotPositionError = true;
plot_options.plotRLNorm = true;
plot_options.plotRmsMetrics = true;
plot_options.graph_output_dir = graph_output_dir;

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
if ~exist(graph_output_dir, 'dir')
    mkdir(graph_output_dir);
end

case_specs = build_case_specs();
all_runs = cell(numel(case_specs) * numel(disturbance_options), 1);
batch_summary = struct('label', {}, 'controller_type', {}, 'with_disturbance', {}, ...
    'source', {}, 'run_file', {});

case_idx = 0;
for disturbance_idx = 1:numel(disturbance_options)
    with_disturbance = disturbance_options(disturbance_idx);
    for spec_idx = 1:numel(case_specs)
        case_idx = case_idx + 1;
        spec = case_specs(spec_idx);

        fprintf('\n==================================================\n');
        fprintf('运行/加载案例 %d/%d: %s, disturbance=%d\n', ...
            case_idx, numel(all_runs), spec.label, with_disturbance);
        fprintf('==================================================\n');

        expected_run_file = fullfile(output_dir, expected_run_filename(spec.label, with_disturbance));

        run_result = [];
        source_file = '';
        if ~run_options.force_rerun
            [found, run_result, source_file] = find_cached_run(spec, with_disturbance, source_data_dirs);
        else
            found = false;
        end

        if found
            fprintf('复用已有结果: %s\n', source_file);
            run_result = normalize_cached_result(run_result, spec, with_disturbance);
            save_struct_atomic(expected_run_file, struct('run_result', run_result), '-v7.3');
        else
            fprintf('未找到可复用结果，开始运行: %s\n', spec.label);
            options = run_options;
            options.controller_param_override = build_params_override(spec, with_disturbance);
            run_result = run_load_lifting_case(spec.controller_type, with_disturbance, options);
            source_file = 'new_run';
        end

        all_runs{case_idx} = run_result;
        batch_summary(case_idx).label = spec.label;
        batch_summary(case_idx).controller_type = run_result.meta.controller_type;
        batch_summary(case_idx).with_disturbance = with_disturbance;
        batch_summary(case_idx).source = source_file;
        batch_summary(case_idx).run_file = expected_run_file;
    end
end

group_no_disturbance = all_runs(cellfun(@(r) ~r.meta.with_disturbance, all_runs));
group_with_disturbance = all_runs(cellfun(@(r) r.meta.with_disturbance, all_runs));

save_struct_atomic(fullfile(output_dir, 'backstepping_rbf_eso_ablation_all_runs.mat'), ...
    scalar_struct('all_runs', all_runs), '-v7.3');
save_struct_atomic(fullfile(output_dir, 'backstepping_rbf_eso_ablation_noDisturbance.mat'), ...
    scalar_struct('group_no_disturbance', group_no_disturbance), '-v7.3');
save_struct_atomic(fullfile(output_dir, 'backstepping_rbf_eso_ablation_withDisturbance.mat'), ...
    scalar_struct('group_with_disturbance', group_with_disturbance), '-v7.3');
save_struct_atomic(fullfile(output_dir, 'backstepping_rbf_eso_ablation_summary.mat'), ...
    struct('batch_summary', batch_summary));

plot_load_lifting_comparison(output_dir, plot_options);

fprintf('\n四组消融对比已完成。\n');
fprintf('结果目录: %s\n', output_dir);
fprintf('图像目录: %s\n', graph_output_dir);

function specs = build_case_specs()
specs = repmat(struct('label', '', 'controller_type', '', 'params_override', struct(), ...
    'rbf_enabled', false, 'rbf_adaptation_enabled', false, 'eso_enabled', false), 2, 1);

specs(1) = make_case('Backstepping+RBF+ESO', 'rbf', true, true, true);
specs(2) = make_case('Backstepping+ESO', 'rbf', false, false, true);
end

function spec = make_case(label, controller_type, rbf_enabled, rbf_adaptation_enabled, eso_enabled)
spec = struct();
spec.label = label;
spec.controller_type = controller_type;
spec.params_override = struct();
spec.rbf_enabled = rbf_enabled;
spec.rbf_adaptation_enabled = rbf_adaptation_enabled;
spec.eso_enabled = eso_enabled;
end

function params_override = build_params_override(spec, with_disturbance)
params_override = struct();
params_override.experiment.comparison_label = spec.label;

if ~strcmpi(spec.controller_type, 'rbf')
    return;
end

% Start from the current best Backstepping+RBF+ESO payload parameters.
params_override = load_current_payload_bsrbf_eso_config(with_disturbance, pwd);
params_override.experiment.comparison_label = spec.label;
params_override.rbf.base_mode = 'backstepping';
params_override.rbf.enable_compensation = spec.rbf_enabled;
params_override.rbf.enable_adaptation = spec.rbf_adaptation_enabled;
params_override.eso.use_compensation = spec.eso_enabled;
end

function filename = expected_run_filename(label, with_disturbance)
if with_disturbance
    disturbance_str = 'withDisturbance';
else
    disturbance_str = 'noDisturbance';
end
filename = sprintf('simulation_run_%s_uavF_BACKSTEPPING_uavM_BACKSTEPPING_%s.mat', ...
    label, disturbance_str);
end

function [found, run_result, source_file] = find_cached_run(spec, with_disturbance, source_data_dirs)
found = false;
run_result = [];
source_file = '';

for dir_idx = 1:numel(source_data_dirs)
    data_dir = source_data_dirs{dir_idx};
    if ~exist(data_dir, 'dir')
        continue;
    end

    run_files = dir(fullfile(data_dir, 'simulation_run_*.mat'));
    for file_idx = 1:numel(run_files)
        candidate = fullfile(run_files(file_idx).folder, run_files(file_idx).name);
        try
            loaded = load(candidate, 'run_result');
        catch ME
            warning('无法读取 %s: %s', candidate, ME.message);
            continue;
        end
        if ~isfield(loaded, 'run_result')
            continue;
        end

        if is_matching_run(loaded.run_result, spec, with_disturbance)
            found = true;
            run_result = loaded.run_result;
            source_file = candidate;
            return;
        end
    end
end
end

function tf = is_matching_run(run_result, spec, with_disturbance)
tf = false;
if ~isfield(run_result, 'meta') || ~isfield(run_result.meta, 'with_disturbance') || ...
        logical(run_result.meta.with_disturbance) ~= logical(with_disturbance)
    return;
end

label = get_meta_string(run_result, 'controller_name', '');
if strcmpi(label, spec.label)
    tf = true;
    return;
end

controller_type = get_meta_string(run_result, 'controller_type', '');
if strcmpi(spec.controller_type, 'backstepping')
    tf = strcmpi(controller_type, 'backstepping');
    return;
end

if ~strcmpi(controller_type, 'rbf') || ~isfield(run_result, 'controller_config')
    return;
end

base_mode = get_nested_string(run_result.controller_config, {'rbf', 'base_mode'}, '');
if isempty(base_mode) && (strcmpi(label, 'RBF') || strcmpi(label, 'RBF+Backstepping'))
    base_mode = 'backstepping';
end
if ~strcmpi(base_mode, 'backstepping')
    return;
end

rbf_enabled = get_nested_logical(run_result.controller_config, {'rbf', 'enable_compensation'}, true);
rbf_adaptation_enabled = get_nested_logical(run_result.controller_config, {'rbf', 'enable_adaptation'}, rbf_enabled);
eso_enabled = get_nested_logical(run_result.controller_config, {'eso', 'use_compensation'}, true);

tf = rbf_enabled == spec.rbf_enabled && ...
    rbf_adaptation_enabled == spec.rbf_adaptation_enabled && ...
    eso_enabled == spec.eso_enabled;
end

function run_result = normalize_cached_result(run_result, spec, with_disturbance)
if ~isfield(run_result, 'meta') || ~isstruct(run_result.meta)
    run_result.meta = struct();
end
run_result.meta.controller_name = spec.label;
run_result.meta.with_disturbance = logical(with_disturbance);
if ~isfield(run_result, 'controller_config') || ~isstruct(run_result.controller_config)
    run_result.controller_config = struct();
end
run_result.controller_config.experiment.comparison_label = spec.label;
if strcmpi(spec.controller_type, 'rbf')
    run_result.controller_config.rbf.base_mode = 'backstepping';
    run_result.controller_config.rbf.enable_compensation = spec.rbf_enabled;
    run_result.controller_config.rbf.enable_adaptation = spec.rbf_adaptation_enabled;
    run_result.controller_config.eso.use_compensation = spec.eso_enabled;
end
if isfield(run_result, 'metrics') && isstruct(run_result.metrics)
    run_result.metrics.controller_label = spec.label;
end
end

function value = get_meta_string(run_result, field_name, default_value)
value = default_value;
if isfield(run_result, 'meta') && isstruct(run_result.meta) && ...
        isfield(run_result.meta, field_name) && ~isempty(run_result.meta.(field_name))
    value = char(run_result.meta.(field_name));
end
end

function value = get_nested_string(input_struct, path_parts, default_value)
value = default_value;
node = input_struct;
for idx = 1:numel(path_parts)
    part = path_parts{idx};
    if ~isstruct(node) || ~isfield(node, part) || isempty(node.(part))
        return;
    end
    node = node.(part);
end
value = char(node);
end

function value = get_nested_logical(input_struct, path_parts, default_value)
value = default_value;
node = input_struct;
for idx = 1:numel(path_parts)
    part = path_parts{idx};
    if ~isstruct(node) || ~isfield(node, part) || isempty(node.(part))
        return;
    end
    node = node.(part);
end
value = logical(node);
end

function save_struct_atomic(target_filename, data, varargin)
if ~isstruct(data) || ~isscalar(data)
    error('save_struct_atomic:InvalidData', ...
        'save_struct_atomic requires a scalar struct input for %s.', target_filename);
end
temp_filename = make_mat_temp_filename(target_filename);
try
    save(temp_filename, '-struct', 'data', varargin{:});
    replace_mat_file(temp_filename, target_filename);
catch ME
    cleanup_file_if_present(temp_filename);
    rethrow(ME);
end
end

function data = scalar_struct(field_name, field_value)
data = struct();
data.(field_name) = field_value;
end

function temp_filename = make_mat_temp_filename(target_filename)
target_dir = fileparts(target_filename);
if isempty(target_dir)
    target_dir = pwd;
end
temp_filename = [tempname(target_dir), '.mat'];
end

function replace_mat_file(temp_filename, target_filename)
[ok, msg, msgid] = movefile(temp_filename, target_filename, 'f');
if ~ok
    error(msgid, '无法替换 MAT 文件 %s: %s', target_filename, msg);
end
end

function cleanup_file_if_present(filename)
if exist(filename, 'file')
    delete(filename);
end
end
