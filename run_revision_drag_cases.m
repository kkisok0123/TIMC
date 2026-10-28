%% 未建模非线性探索：二次气动阻力
% 目的：检验 RBF 的增量是否随"状态相关未建模项"增强而增大。
% 被控对象：F_drag = -cd*|v|*v（世界系），控制器模型完全不知道阻力存在。
% 组别：BS / BS+RBF / BS+ESO / BS+RBF+ESO
% 阻力：cd = 0 / 4 / 12（cd=12 时约 1.1 m/s 下约 14.5 N，约 74% 重量）
%
% 运行：matlab -batch "run_revision_drag_cases"

clear; clc;
root_dir = fileparts(mfilename('fullpath'));
if isempty(root_dir), root_dir = pwd; end
cd(root_dir);

drag_levels = [0, 4, 12];
output_root = fullfile(root_dir, 'revision_runs', 'drag');
if ~exist(output_root, 'dir'), mkdir(output_root); end

cases = build_cases();
rows = {};
run_idx = 0;
total = numel(drag_levels) * numel(cases);

for d_idx = 1:numel(drag_levels)
    cd_val = drag_levels(d_idx);
    for c_idx = 1:numel(cases)
        run_idx = run_idx + 1;
        spec = cases(c_idx);
        label = sprintf('%s_cd%d', spec.label, cd_val);
        fprintf('\n===== [%d/%d] %s (drag cd=%d) =====\n', run_idx, total, spec.label, cd_val);

        override = struct();
        override.experiment = struct('comparison_label', label, ...
            'payload_plant', struct('drag_cd', cd_val));
        if strcmpi(spec.controller_type, 'rbf')
            override.rbf = struct('base_mode', 'backstepping', ...
                'enable_compensation', spec.rbf_comp, 'enable_adaptation', spec.rbf_adapt);
            override.eso = struct('use_compensation', spec.eso_comp);
        end

        options = struct();
        options.controller_param_override = override;
        options.run_output_dir = fullfile(output_root, label);
        options.save_run_data = true;
        options.save_workspace = false;
        options.enable_default_plots = false;

        status = 'ok';
        try
            r = run_load_lifting_case(spec.controller_type, true, options);
            m = compute_revision_metrics(r);
        catch ME
            status = sprintf('failed: %s', ME.message);
            m = struct();
            warning('TIMC:RunFailed', '%s: %s', label, ME.message);
        end
        rows(end+1, :) = {spec.label, cd_val, status, ...
            get_metric(m, 'load_pos_rms'), get_metric(m, 'load_pos_max'), ...
            get_metric(m, 'rot_err_rms'), get_metric(m, 'tension_min_steady')}; %#ok<SAGROW>
    end
end

T = cell2table(rows, 'VariableNames', {'method', 'drag_cd', 'status', 'load_pos_rms', ...
    'load_pos_max', 'rot_err_rms', 'tension_min_steady'});
writetable(T, fullfile(output_root, 'revision_drag_summary.csv'));
disp(T);

% 复合相对 ESO-only 的比值
fprintf('\nComposite / ESO-only ratio by drag level:\n');
for d_idx = 1:numel(drag_levels)
    cd_val = drag_levels(d_idx);
    i_e = strcmp(T.method, 'BS_ESO') & T.drag_cd == cd_val;
    i_c = strcmp(T.method, 'BS_RBF_ESO') & T.drag_cd == cd_val;
    if any(i_e) && any(i_c)
        fprintf('  cd=%2d : %.4f\n', cd_val, ...
            T.load_pos_rms(find(i_c,1)) / T.load_pos_rms(find(i_e,1)));
    end
end

function cases = build_cases()
cases = struct('label', {}, 'controller_type', {}, 'rbf_comp', {}, 'rbf_adapt', {}, 'eso_comp', {});
cases(end+1) = make_case('BS', 'backstepping', false, false, false);
cases(end+1) = make_case('BS_RBF', 'rbf', true, true, false);
cases(end+1) = make_case('BS_ESO', 'rbf', false, false, true);
cases(end+1) = make_case('BS_RBF_ESO', 'rbf', true, true, true);
end

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
