%% 极端参数失配（+/-50%）：检验 RBF 的增量是否随失配增大
% 若复合相对 ESO-only 的比例仍约 0.90，说明 RBF 的收益与失配无关。

clear; clc;
root_dir = fileparts(mfilename('fullpath'));
if isempty(root_dir), root_dir = pwd; end
cd(root_dir);

scales = [0.5, 1.5];
output_root = fullfile(root_dir, 'revision_runs', 'mismatch_extreme');
if ~exist(output_root, 'dir'), mkdir(output_root); end

cases = struct('label', {}, 'controller_type', {}, 'rbf_comp', {}, 'rbf_adapt', {}, 'eso_comp', {});
cases(end+1) = make_case('BS_RBF', 'rbf', true, true, false);
cases(end+1) = make_case('BS_ESO', 'rbf', false, false, true);
cases(end+1) = make_case('BS_RBF_ESO', 'rbf', true, true, true);

rows = {};
for s_idx = 1:numel(scales)
    scale = scales(s_idx);
    for c_idx = 1:numel(cases)
        spec = cases(c_idx);
        label = sprintf('%s_m%.1f', spec.label, scale);
        fprintf('\n===== %s =====\n', label);

        override = struct();
        override.experiment = struct('comparison_label', label, ...
            'payload_plant', struct('mass_scale', scale, 'inertia_scale', scale));
        override.rbf = struct('base_mode', 'backstepping', ...
            'enable_compensation', spec.rbf_comp, 'enable_adaptation', spec.rbf_adapt);
        override.eso = struct('use_compensation', spec.eso_comp);

        options = struct();
        options.controller_param_override = override;
        options.run_output_dir = fullfile(output_root, label);
        options.save_run_data = true;
        options.save_workspace = false;
        options.enable_default_plots = false;

        status = 'ok';
        try
            r = run_load_lifting_case('rbf', true, options);
            m = compute_revision_metrics(r);
        catch ME
            status = sprintf('failed: %s', ME.message);
            m = struct();
            warning('TIMC:RunFailed', '%s: %s', label, ME.message);
        end
        rows(end+1, :) = {spec.label, scale, status, ...
            get_metric(m, 'load_pos_rms'), get_metric(m, 'rot_err_rms'), ...
            get_metric(m, 'tension_min_steady')}; %#ok<SAGROW>
    end
end

T = cell2table(rows, 'VariableNames', {'case','mass_inertia_scale','status','load_pos_rms','rot_err_rms','tension_min_steady'});
writetable(T, fullfile(output_root, 'mismatch_extreme_summary.csv'));
disp(T);

function spec = make_case(label, controller_type, rbf_comp, rbf_adapt, eso_comp)
spec = struct('label', label, 'controller_type', controller_type, ...
    'rbf_comp', rbf_comp, 'rbf_adapt', rbf_adapt, 'eso_comp', eso_comp);
end

function value = get_metric(m, name)
if isstruct(m) && isfield(m, name) && ~isempty(m.(name))
    value = double(m.(name));
else
    value = NaN;
end
end
