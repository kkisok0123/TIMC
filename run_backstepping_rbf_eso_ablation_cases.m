%% 四组消融预检查（返修版）
% 组别：Backstepping / Backstepping+RBF / Backstepping+ESO / 复合方法
% 公共参数完全一致，只切换 RBF 补偿、RBF 自适应与 ESO 补偿三个开关。
%
% 运行：matlab -batch "run_backstepping_rbf_eso_ablation_cases"
% 输出：revision_runs/<case>_<disturbance>/ 下的 run_result.mat 与汇总 CSV

clear; clc;

root_dir = fileparts(mfilename('fullpath'));
if isempty(root_dir)
    root_dir = pwd;
end
cd(root_dir);
addpath(root_dir);

disturbance_options = [false, true];
output_root = fullfile(root_dir, 'revision_runs');
if ~exist(output_root, 'dir')
    mkdir(output_root);
end

cases = build_cases();
rows = {};

for d_idx = 1:numel(disturbance_options)
    with_disturbance = disturbance_options(d_idx);
    if with_disturbance
        dist_label = 'withDisturbance';
    else
        dist_label = 'noDisturbance';
    end

    for c_idx = 1:numel(cases)
        spec = cases(c_idx);
        fprintf('\n===== [%d/%d] %s / %s =====\n', ...
            (d_idx-1)*numel(cases) + c_idx, numel(disturbance_options)*numel(cases), ...
            spec.label, dist_label);

        options = struct();
        options.controller_param_override = build_override(spec);
        options.run_output_dir = fullfile(output_root, sprintf('%s_%s', spec.label, dist_label));
        options.save_run_data = true;
        options.enable_default_plots = false;

        status = 'ok';
        try
            run_result = run_load_lifting_case(spec.controller_type, with_disturbance, options);
            metrics = compute_revision_metrics(run_result);
            save(fullfile(options.run_output_dir, 'revision_metrics.mat'), 'metrics');
        catch ME
            status = sprintf('failed: %s', ME.message);
            metrics = struct();
            warning('TIMC:RunFailed', '%s / %s failed: %s', spec.label, dist_label, ME.message);
        end

        rows(end+1, :) = {spec.label, dist_label, status, ...
            get_metric(metrics, 'load_pos_rms'), get_metric(metrics, 'load_pos_max'), ...
            get_metric(metrics, 'rot_err_rms'), get_metric(metrics, 'rot_err_max'), ...
            get_metric(metrics, 'tension_min_steady'), get_metric(metrics, 'tension_negative_samples'), ...
            get_metric(metrics, 'alloc_force_residual_rms'), get_metric(metrics, 'alloc_moment_residual_rms')}; %#ok<SAGROW>
    end
end

summary = cell2table(rows, 'VariableNames', {'case', 'disturbance', 'status', ...
    'load_pos_rms', 'load_pos_max', 'rot_err_rms', 'rot_err_max', ...
    'tension_min_steady', 'tension_negative_samples', ...
    'alloc_force_residual_rms', 'alloc_moment_residual_rms'});

summary_file = fullfile(output_root, 'revision_ablation_summary.csv');
writetable(summary, summary_file);
fprintf('\n汇总表已保存: %s\n', summary_file);
disp(summary);

function cases = build_cases()
cases = struct('label', {}, 'controller_type', {}, 'rbf_comp', {}, 'rbf_adapt', {}, 'eso_comp', {});
cases(end+1) = make_case('BS', 'backstepping', false, false, false);
cases(end+1) = make_case('BS_RBF', 'rbf', true, true, false);
cases(end+1) = make_case('BS_ESO', 'rbf', false, false, true);
cases(end+1) = make_case('BS_RBF_ESO', 'rbf', true, true, true);
end

function spec = make_case(label, controller_type, rbf_comp, rbf_adapt, eso_comp)
spec = struct();
spec.label = label;
spec.controller_type = controller_type;
spec.rbf_comp = rbf_comp;
spec.rbf_adapt = rbf_adapt;
spec.eso_comp = eso_comp;
end

function override = build_override(spec)
override = struct();
override.experiment = struct('comparison_label', spec.label);
if strcmpi(spec.controller_type, 'rbf')
    override.rbf = struct( ...
        'base_mode', 'backstepping', ...
        'enable_compensation', spec.rbf_comp, ...
        'enable_adaptation', spec.rbf_adapt);
    override.eso = struct('use_compensation', spec.eso_comp);
end
end

function value = get_metric(metrics, field_name)
if isstruct(metrics) && isfield(metrics, field_name) && ~isempty(metrics.(field_name))
    value = double(metrics.(field_name));
else
    value = NaN;
end
end
