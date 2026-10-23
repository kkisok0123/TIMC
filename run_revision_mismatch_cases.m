%% 参数失配实验（返修版）
% 目的：回答"ESO 已经存在时，增加 RBF 是否还有价值"。
% 工况：固定周期扰动；真实质量与惯量按同一比例缩放，控制器仍使用名义模型。
% 组别：BS / BS+RBF / BS+ESO / BS+RBF+ESO
% 失配：0.8 / 1.0 / 1.2 三档（1.0 作为同批次对照，保证表内自洽）
%
% 运行：matlab -batch "run_revision_mismatch_cases"
% 输出：revision_runs/mismatch/<case>_m<scale>/ 与 revision_mismatch_summary.csv

clear; clc;

root_dir = fileparts(mfilename('fullpath'));
if isempty(root_dir)
    root_dir = pwd;
end
cd(root_dir);
addpath(root_dir);

mismatch_scales = [0.8, 1.0, 1.2];
with_disturbance = true;
output_root = fullfile(root_dir, 'revision_runs', 'mismatch');
if ~exist(output_root, 'dir')
    mkdir(output_root);
end

cases = build_cases();
rows = {};
total_runs = numel(mismatch_scales) * numel(cases);
run_idx = 0;

for s_idx = 1:numel(mismatch_scales)
    scale = mismatch_scales(s_idx);
    for c_idx = 1:numel(cases)
        run_idx = run_idx + 1;
        spec = cases(c_idx);
        label = sprintf('%s_m%.1f', spec.label, scale);
        fprintf('\n===== [%d/%d] %s (mass/inertia x%.2f) =====\n', ...
            run_idx, total_runs, spec.label, scale);

        override = build_override(spec);
        override.experiment.payload_plant = struct('mass_scale', scale, 'inertia_scale', scale);
        override.experiment.comparison_label = label;

        options = struct();
        options.controller_param_override = override;
        options.run_output_dir = fullfile(output_root, label);
        options.save_run_data = true;
        options.save_workspace = false;
        options.enable_default_plots = false;

        status = 'ok';
        try
            run_result = run_load_lifting_case(spec.controller_type, with_disturbance, options);
            metrics = compute_revision_metrics(run_result);
            save(fullfile(options.run_output_dir, 'revision_metrics.mat'), 'metrics');
        catch ME
            status = sprintf('failed: %s', ME.message);
            metrics = struct();
            warning('TIMC:RunFailed', '%s failed: %s', label, ME.message);
        end

        rows(end+1, :) = {spec.label, scale, status, ...
            get_metric(metrics, 'load_pos_rms'), get_metric(metrics, 'load_pos_max'), ...
            get_metric(metrics, 'rot_err_rms'), get_metric(metrics, 'rot_err_max'), ...
            get_metric(metrics, 'tension_min_steady'), get_metric(metrics, 'tension_negative_samples'), ...
            get_metric(metrics, 'alloc_force_residual_rms')}; %#ok<SAGROW>
    end
end

summary = cell2table(rows, 'VariableNames', {'case', 'mass_inertia_scale', 'status', ...
    'load_pos_rms', 'load_pos_max', 'rot_err_rms', 'rot_err_max', ...
    'tension_min_steady', 'tension_negative_samples', 'alloc_force_residual_rms'});

summary_file = fullfile(output_root, 'revision_mismatch_summary.csv');
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
