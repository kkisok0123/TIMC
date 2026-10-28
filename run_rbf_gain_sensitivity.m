%% RBF 学习率敏感性（复合方法、标称模型、周期扰动）
% 目的：判断"复合相对 ESO-only 约 10%"是否是默认学习率偏保守造成的。
% 固定其余参数，只改 gama / gama_att。

clear; clc;
root_dir = fileparts(mfilename('fullpath'));
if isempty(root_dir), root_dir = pwd; end
cd(root_dir);

gammas = [5, 100];
output_root = fullfile(root_dir, 'revision_runs', 'gain_sensitivity');
if ~exist(output_root, 'dir'), mkdir(output_root); end

rows = {};
for k = 1:numel(gammas)
    g = gammas(k);
    label = sprintf('BS_RBF_ESO_gamma%d', g);
    fprintf('\n===== gamma = %d =====\n', g);

    override = struct();
    override.experiment = struct('comparison_label', label);
    override.rbf = struct('base_mode', 'backstepping', ...
        'enable_compensation', true, 'enable_adaptation', true, ...
        'adaptation', struct('gama', g, 'gama_att', g));
    override.eso = struct('use_compensation', true);

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
        th = reshape(r.theta_rbf(:), 5, 6);
        theta_norm_max = max(arrayfun(@(c) norm(th(:,c)), 1:6));
    catch ME
        status = sprintf('failed: %s', ME.message);
        m = struct();
        theta_norm_max = NaN;
    end
    rows(end+1, :) = {g, status, get_metric(m, 'load_pos_rms'), ...
        get_metric(m, 'rot_err_rms'), theta_norm_max}; %#ok<SAGROW>
end

T = cell2table(rows, 'VariableNames', {'gamma', 'status', 'load_pos_rms', 'rot_err_rms', 'theta_norm_max'});
writetable(T, fullfile(output_root, 'gain_sensitivity.csv'));
disp(T);

function value = get_metric(m, name)
if isstruct(m) && isfield(m, name) && ~isempty(m.(name))
    value = double(m.(name));
else
    value = NaN;
end
end
