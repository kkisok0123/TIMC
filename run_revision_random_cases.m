function run_revision_random_cases(varargin)
%% 非周期随机扰动实验（实验 B）：4 方法 × 5 种子
%
% 目的：在固定随机种子、统一参数集的非周期（带限随机）扰动下，比较
%       BS / BS+RBF / BS+ESO / BS+RBF+ESO 四种控制器，检查排序是否与
%       已有周期扰动（4 正弦）实验一致。只加扰动模型与批处理，不改控制器、
%       不改指标定义、不重新调参。
%
% 用法：
%   run_revision_random_cases                       % 完整矩阵 4×5
%   run_revision_random_cases('seeds', 101)         % 预试验（只跑 101）
%   run_revision_random_cases('methods', {'BS'})    % 只跑某一方法
%   matlab -batch "run_revision_random_cases"
%
% 输出（默认 output_root = <repo>/revision_runs/random）：
%   <method>_s<seed>/simulation_run_*.mat   run_result（save_workspace=false）
%   <method>_s<seed>/revision_metrics.mat   compute_revision_metrics 结果
%   <method>_s<seed>/disturbance_trace.mat  该种子的随机扰动 trace（可复核）
%   revision_random_summary.csv             主表（规定列）
%   revision_random_detail.csv              附表（全程指标、负张力样本、耗时）
%   revision_random_summary.md              人读版汇总
%
% 约定：
%   - 每个 case 独立初始化（clear + 唯一一次 rng 重置在生成器内部）；
%   - 强制重跑，不复用任何旧 MAT（旧结果来自确定性扰动）；
%   - payload_drag_cd 显式置 0，避免混入未建模阻力项；
%   - 失败（NaN/Inf、位置误差 > 5 m、仿真未跑完）必须如实记录，不删除不截断。

%% ==================== 参数 ====================
opts.seeds = [101 202 303 404 505];
opts.methods = {'BS', 'BS_RBF', 'BS_ESO', 'BS_RBF_ESO'};
opts.amp_pos = 1.0;          % 平移三轴目标 RMS (m/s^2)
opts.amp_att = 0.3;          % 姿态三轴目标 RMS (rad/s^2)
opts.bandwidth_hz = 1.0;     % 低通截止 (Hz)
opts.position_limit = 5.0;   % 失败判据：位置误差上限 (m)
opts.precheck_only = false;
opts.summary_only = false;   % true = 不跑仿真，仅从已有 <case>/revision_metrics.mat 重建汇总
opts.output_root = '';
[opts, unknown] = parse_options(opts, varargin);
if ~isempty(unknown)
    error('TIMC:Random:UnknownOption', 'Unknown option(s): %s', strjoin(unknown, ', '));
end

root_dir = fileparts(mfilename('fullpath'));
if isempty(root_dir), root_dir = pwd; end
cd(root_dir);
if isempty(opts.output_root)
    opts.output_root = fullfile(root_dir, 'revision_runs', 'random');
end
if ~exist(opts.output_root, 'dir'), mkdir(opts.output_root); end

mfilename_this = mfilename;
fprintf('=== TIMC 实验 B：非周期随机扰动 ===\n');
fprintf('方法: %s\n', strjoin(opts.methods, ', '));
fprintf('种子: %s\n', mat2str(opts.seeds));
fprintf('扰动: AR(1) 低通, fc=%.3f Hz, RMS(pos)=%.3f m/s^2, RMS(att)=%.3f rad/s^2\n', ...
    opts.bandwidth_hz, opts.amp_pos, opts.amp_att);
fprintf('输出: %s\n\n', opts.output_root);

cases = build_cases(opts.methods);
total = numel(cases) * numel(opts.seeds);
if total == 0
    error('TIMC:Random:NoCases', 'No case selected.');
end

% 时间网格只用于失败时（未拿到 run_result）重建 trace；正常路径取 run_result 内的 trace
sim_dt = 1e-3;
sim_Tf = 20.0;

if opts.summary_only
    rebuild_summary(opts, cases, opts.seeds, mfilename_this);
    fprintf('已从已有 case 目录重建汇总（未跑仿真）。\n');
    return
end

rows = cell(0, 8);
detail_rows = cell(0, 11);
t_start_all = tic;
run_idx = 0;

for s_idx = 1:numel(opts.seeds)
    seed = opts.seeds(s_idx);
    for c_idx = 1:numel(cases)
        spec = cases(c_idx);
        run_idx = run_idx + 1;
        label = sprintf('%s_s%d', spec.label, seed);
        case_dir = fullfile(opts.output_root, label);
        fprintf('\n===== [%d/%d] %s =====\n', run_idx, total, label);

        % 强制重跑：清空该 case 目录内的旧产物
        reset_case_dir(case_dir);

        override = struct();
        override.experiment = struct( ...
            'comparison_label', label, ...
            'payload_plant', struct('drag_cd', 0), ...
            'disturbance', struct('random', struct( ...
                'enabled', true, ...
                'seed', seed, ...
                'amp_pos', opts.amp_pos, ...
                'amp_att', opts.amp_att, ...
                'bandwidth_hz', opts.bandwidth_hz)));
        if strcmpi(spec.controller_type, 'rbf')
            override.rbf = struct('base_mode', 'backstepping', ...
                'enable_compensation', spec.rbf_comp, 'enable_adaptation', spec.rbf_adapt);
            override.eso = struct('use_compensation', spec.eso_comp);
        end

        options = struct();
        options.controller_param_override = override;
        options.run_output_dir = case_dir;
        options.save_run_data = true;
        options.save_workspace = false;   % 批量时务必 false
        options.enable_default_plots = false;

        case_t0 = tic;
        status = 'ok';
        failure_reason = '';
        m = struct();
        trace = [];
        trace_hash = '';
        try
            r = run_load_lifting_case(spec.controller_type, true, options);
            if ~isfield(r, 'disturbance') || ~isfield(r.disturbance, 'random_trace') || ...
                    isempty(r.disturbance.random_trace)
                error('TIMC:Random:MissingTrace', 'run_result.disturbance.random_trace is missing.');
            end
            trace = r.disturbance.random_trace;
            trace_hash = r.disturbance.random_hash;
            m = compute_revision_metrics(r);

            % 失败判据（按顺序判定，理由写入 status）
            [bad, reason] = diagnose_run(r, m, sim_Tf, opts.position_limit);
            if bad
                status = 'failed';
                failure_reason = reason;
            end
            clear r;
        catch ME
            status = 'failed';
            failure_reason = sprintf('exception: %s', ME.message);
            warning('TIMC:Random:RunFailed', '%s: %s', label, ME.message);
        end

        % 无论成功失败都保留 trace（失败时按同一 seed/参数重建，保证可追溯）
        if isempty(trace)
            t_grid = (0:round(sim_Tf / sim_dt)) * sim_dt;
            trace = make_bandlimited_disturbance(t_grid, seed, struct( ...
                'amp_pos', opts.amp_pos, 'amp_att', opts.amp_att, ...
                'bandwidth_hz', opts.bandwidth_hz));
            trace_hash = trace.hash;
        end
        save(fullfile(case_dir, 'disturbance_trace.mat'), 'trace', '-v7.3');

        if isempty(m)
            m = struct('status', status, 'failure_reason', failure_reason);
        else
            m.status = status;
            if ~isempty(failure_reason), m.failure_reason = failure_reason; end
        end
        elapsed = toc(case_t0);
        m.wall_time_s = elapsed;   % 便于 summary_only 重建时保留耗时列
        save(fullfile(case_dir, 'revision_metrics.mat'), 'm', '-v7.3');

        if strcmp(status, 'ok')
            status_str = 'ok';
        else
            status_str = sprintf('failed: %s', failure_reason);
        end

        rows(end + 1, :) = {spec.label, seed, status_str, ...
            get_metric(m, 'load_pos_rms'), get_metric(m, 'load_pos_max'), ...
            get_metric(m, 'rot_err_rms'), get_metric(m, 'tension_min_steady'), ...
            trace_hash}; %#ok<SAGROW>

        detail_rows(end + 1, :) = {spec.label, seed, status_str, ...
            get_metric(m, 'load_pos_rms_full'), get_metric(m, 'load_pos_max_full'), ...
            get_metric(m, 'rot_err_max'), get_metric(m, 'tension_min'), ...
            get_metric(m, 'tension_negative_samples'), ...
            get_metric(m, 'alloc_force_residual_rms'), ...
            get_metric(m, 'control_step_mean_ms'), elapsed}; %#ok<SAGROW>

        fprintf('  -> %s | pos_rms=%.4f m, pos_max=%.4f m, rot_rms=%.4f rad, T_min=%.3f N, %s | %.1f s\n', ...
            status_str, get_metric(m, 'load_pos_rms'), get_metric(m, 'load_pos_max'), ...
            get_metric(m, 'rot_err_rms'), get_metric(m, 'tension_min_steady'), ...
            trace_hash, elapsed);

        % 每跑完一个 case 就落盘一次，长跑中断也不丢已完成结果
        write_summaries(opts, rows, detail_rows, total, run_idx, ...
            toc(t_start_all), mfilename_this);
    end
end

T = write_summaries(opts, rows, detail_rows, total, run_idx, toc(t_start_all), mfilename_this);
fprintf('\n=== 全部完成：%d/%d case，用时 %.1f 分钟 ===\n', run_idx, total, toc(t_start_all) / 60);
disp(T(:, {'method', 'seed', 'status', 'load_pos_rms', 'load_pos_max', 'rot_err_rms', 'tension_min', 'trace_hash'}));
end

%% ==================== 子函数 ====================

function cases = build_cases(method_labels)
cases = struct('label', {}, 'controller_type', {}, 'rbf_comp', {}, 'rbf_adapt', {}, 'eso_comp', {});
all_cases = [ ...
    make_case('BS', 'backstepping', false, false, false), ...
    make_case('BS_RBF', 'rbf', true, true, false), ...
    make_case('BS_ESO', 'rbf', false, false, true), ...
    make_case('BS_RBF_ESO', 'rbf', true, true, true)];
for k = 1:numel(all_cases)
    if any(strcmp(method_labels, all_cases(k).label))
        cases(end + 1) = all_cases(k); %#ok<AGROW>
    end
end
% 按 method_labels 给定顺序排列
[~, order] = ismember(method_labels, {cases.label});
cases = cases(order(order > 0));
end

function spec = make_case(label, controller_type, rbf_comp, rbf_adapt, eso_comp)
spec = struct('label', label, 'controller_type', controller_type, ...
    'rbf_comp', rbf_comp, 'rbf_adapt', rbf_adapt, 'eso_comp', eso_comp);
end

function reset_case_dir(case_dir)
% 强制重跑：删除该 case 目录下上一轮的产物（不复用旧缓存）
if ~exist(case_dir, 'dir')
    mkdir(case_dir);
    return
end
old_files = dir(fullfile(case_dir, '*.mat'));
for k = 1:numel(old_files)
    delete(fullfile(case_dir, old_files(k).name));
end
end

function rebuild_summary(opts, cases, seeds, caller_name)
% 不跑仿真，只从已有 case 目录（revision_metrics.mat + disturbance_trace.mat）重建汇总表
rows = cell(0, 8);
detail_rows = cell(0, 11);
missing = {};
old_wall = read_old_wall_times(opts.output_root);
for s_idx = 1:numel(seeds)
    seed = seeds(s_idx);
    for c_idx = 1:numel(cases)
        label = sprintf('%s_s%d', cases(c_idx).label, seed);
        case_dir = fullfile(opts.output_root, label);
        metrics_file = fullfile(case_dir, 'revision_metrics.mat');
        trace_file = fullfile(case_dir, 'disturbance_trace.mat');
        if ~exist(metrics_file, 'file') || ~exist(trace_file, 'file')
            missing{end + 1} = label; %#ok<AGROW>
            continue
        end
        S = load(metrics_file); m = S.m;
        S = load(trace_file); trace = S.trace;
        status_str = 'ok';
        if isfield(m, 'status') && ~strcmp(m.status, 'ok')
            status_str = sprintf('failed: %s', m.failure_reason);
        end
        rows(end + 1, :) = {cases(c_idx).label, seed, status_str, ...
            get_metric(m, 'load_pos_rms'), get_metric(m, 'load_pos_max'), ...
            get_metric(m, 'rot_err_rms'), get_metric(m, 'tension_min_steady'), ...
            trace.hash}; %#ok<SAGROW>
        detail_rows(end + 1, :) = {cases(c_idx).label, seed, status_str, ...
            get_metric(m, 'load_pos_rms_full'), get_metric(m, 'load_pos_max_full'), ...
            get_metric(m, 'rot_err_max'), get_metric(m, 'tension_min'), ...
            get_metric(m, 'tension_negative_samples'), ...
            get_metric(m, 'alloc_force_residual_rms'), ...
            get_metric(m, 'control_step_mean_ms'), ...
            wall_time_or_old(m, old_wall, cases(c_idx).label, seed)}; %#ok<SAGROW>
    end
end
if ~isempty(missing)
    warning('TIMC:Random:MissingCases', '未找到 %d 个 case 的产物：%s', ...
        numel(missing), strjoin(missing, ', '));
end
T = write_summaries(opts, rows, detail_rows, numel(cases) * numel(seeds), ...
    numel(rows), 0, caller_name);
disp(T(:, {'method', 'seed', 'status', 'load_pos_rms', 'load_pos_max', 'rot_err_rms', 'tension_min', 'trace_hash'}));
end

function old = read_old_wall_times(output_root)
% 旧版 metrics 里没有 wall_time_s 时，从既有 detail CSV 回填，避免重建时丢耗时列
old = struct('method', {}, 'seed', {}, 'wall', {});
path = fullfile(output_root, 'revision_random_detail.csv');
if ~exist(path, 'file'), return; end
try
    D = readtable(path, 'TextType', 'string');
    if all(ismember({'method', 'seed', 'wall_time_s'}, D.Properties.VariableNames))
        for k = 1:height(D)
            old(end + 1) = struct('method', char(D.method(k)), 'seed', D.seed(k), ...
                'wall', D.wall_time_s(k)); %#ok<AGROW>
        end
    end
catch
    old = struct('method', {}, 'seed', {}, 'wall', {});
end
end

function v = wall_time_or_old(m, old_wall, label, seed)
v = get_metric(m, 'wall_time_s');
if isfinite(v), return; end
v = NaN;
for k = 1:numel(old_wall)
    if strcmp(old_wall(k).method, label) && old_wall(k).seed == seed
        v = old_wall(k).wall;
        return
    end
end
end

function [bad, reason] = diagnose_run(r, m, expected_Tf, position_limit)
% 失败判据：NaN/Inf、位置误差超过 5 m、仿真未跑完
bad = false;
reason = '';

fields = {'load_pos_rms', 'load_pos_max', 'rot_err_rms', 'tension_min_steady'};
for k = 1:numel(fields)
    v = get_metric(m, fields{k});
    if ~isfinite(v)
        bad = true;
        reason = sprintf('non-finite %s', fields{k});
        return
    end
end

if isfield(r, 'time') && isfield(r.time, 't') && ~isempty(r.time.t)
    t_end = r.time.t(end);
    if abs(t_end - expected_Tf) > 1e-9 * max(1, expected_Tf)
        bad = true;
        reason = sprintf('incomplete sim: t_end=%.6f (expected %.6f)', t_end, expected_Tf);
        return
    end
else
    bad = true;
    reason = 'incomplete sim: time vector missing';
    return
end

states = [r.states.X_L(:); r.states.X_UAV(:)];
if any(~isfinite(states))
    bad = true;
    reason = 'non-finite state values';
    return
end

if isfield(m, 'load_pos_max_full') && isfinite(m.load_pos_max_full) && m.load_pos_max_full > position_limit
    bad = true;
    reason = sprintf('position error %.3f m > %.1f m', m.load_pos_max_full, position_limit);
    return
end
end

function T = write_summaries(opts, rows, detail_rows, total, run_idx, elapsed_total, caller_name)
% 主表 + 附表 + 人读版汇总；每个 case 结束后覆盖写一次
T = cell2table(rows, 'VariableNames', {'method', 'seed', 'status', 'load_pos_rms', ...
    'load_pos_max', 'rot_err_rms', 'tension_min', 'trace_hash'});
writetable(T, fullfile(opts.output_root, 'revision_random_summary.csv'));

D = cell2table(detail_rows, 'VariableNames', {'method', 'seed', 'status', ...
    'load_pos_rms_full', 'load_pos_max_full', 'rot_err_max', 'tension_min_all', ...
    'tension_negative_samples', 'alloc_force_residual_rms', 'control_step_mean_ms', ...
    'wall_time_s'});
writetable(D, fullfile(opts.output_root, 'revision_random_detail.csv'));

write_markdown(opts, T, D, total, run_idx, elapsed_total, caller_name);
end

function write_markdown(opts, T, D, total, run_idx, elapsed_total, caller_name)
path = fullfile(opts.output_root, 'revision_random_summary.md');
fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    warning('TIMC:Random:WriteFailed', 'Cannot write %s', path);
    return
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# 实验 B：非周期随机扰动（AR(1) 带限噪声）\n\n');
fprintf(fid, '- 生成脚本：`%s.m`\n', caller_name);
fprintf(fid, '- 扰动：一阶低通白噪声，fc = %.3f Hz，平移 RMS = %.3f m/s²，姿态 RMS = %.3f rad/s²\n', ...
    opts.bandwidth_hz, opts.amp_pos, opts.amp_att);
fprintf(fid, '- 种子：%s；时长/步长 20 s / 1 ms；payload_drag_cd = 0\n', mat2str(opts.seeds));
fprintf(fid, '- 随机扰动与周期扰动互斥（启用随机时不再叠加 4 正弦扰动）\n');
fprintf(fid, '- 进度：%d/%d case，累计 %.1f 分钟 %s\n\n', run_idx, total, elapsed_total / 60, ...
    ternary(run_idx >= total, '（已完成）', '（进行中）'));

fprintf(fid, '## 逐次结果\n\n');
fprintf(fid, '| method | seed | status | load_pos_rms (m) | load_pos_max (m) | rot_err_rms (rad) | tension_min (N) | trace_hash |\n');
fprintf(fid, '|---|---|---|---|---|---|---|---|\n');
for k = 1:height(T)
    fprintf(fid, '| %s | %d | %s | %s | %s | %s | %s | `%s` |\n', ...
        T.method{k}, T.seed(k), esc(T.status{k}), fmt(T.load_pos_rms(k)), ...
        fmt(T.load_pos_max(k)), fmt(T.rot_err_rms(k)), fmt(T.tension_min(k)), T.trace_hash{k});
end
fprintf(fid, '\n');

methods = unique(T.method, 'stable');
fprintf(fid, '## 按方法聚合（5 个种子，仅作初步离散度，不作统计显著性主张）\n\n');
fprintf(fid, '| method | n_ok | n_fail | load_pos_rms mean ± std | load_pos_max mean ± std | rot_err_rms mean ± std | tension_min min |\n');
fprintf(fid, '|---|---|---|---|---|---|---|\n');
for k = 1:numel(methods)
    name = methods{k};
    sel = strcmp(T.method, name);
    ok = sel & strcmp(T.status, 'ok');
    n_ok = sum(ok);
    n_fail = sum(sel) - n_ok;
    fprintf(fid, '| %s | %d | %d | %s | %s | %s | %s |\n', name, n_ok, n_fail, ...
        mean_std(T.load_pos_rms(ok)), mean_std(T.load_pos_max(ok)), ...
        mean_std(T.rot_err_rms(ok)), fmt(min_or_nan(T.tension_min(ok))));
end
fprintf(fid, '\n');

% 排序（按位置 RMS 均值）与复合/ESO 比值
fprintf(fid, '## 排序与关键比值\n\n');
means = nan(numel(methods), 1);
for k = 1:numel(methods)
    sel = strcmp(T.method, methods{k}) & strcmp(T.status, 'ok');
    means(k) = mean_or_nan(T.load_pos_rms(sel));
end
[~, order] = sort(means);
fprintf(fid, '按 `load_pos_rms` 均值升序（越小越好）：\n\n');
for k = 1:numel(order)
    if isfinite(means(order(k)))
        fprintf(fid, '%d. %s — %s\n', k, methods{order(k)}, fmt(means(order(k))));
    end
end
fprintf(fid, '\n');
i_e = strcmp(T.method, 'BS_ESO') & strcmp(T.status, 'ok');
i_c = strcmp(T.method, 'BS_RBF_ESO') & strcmp(T.status, 'ok');
i_r = strcmp(T.method, 'BS_RBF') & strcmp(T.status, 'ok');
if any(i_e) && any(i_c)
    fprintf(fid, '- 复合 / ESO-only（位置 RMS 均值比）：%.4f\n', ...
        mean_or_nan(T.load_pos_rms(i_c)) / mean_or_nan(T.load_pos_rms(i_e)));
end
if any(i_r) && any(i_c)
    fprintf(fid, '- 复合 / RBF-only（位置 RMS 均值比）：%.4f\n', ...
        mean_or_nan(T.load_pos_rms(i_c)) / mean_or_nan(T.load_pos_rms(i_r)));
end
fprintf(fid, '\n## 与周期扰动实验的对照\n\n');
write_periodic_comparison(fid, opts, T);
fprintf(fid, '\n## 失败项\n\n');
failed = T(~strcmp(T.status, 'ok'), :);
if isempty(failed)
    fprintf(fid, '无。\n');
else
    for k = 1:height(failed)
        fprintf(fid, '- %s / seed %d：%s\n', failed.method{k}, failed.seed(k), esc(failed.status{k}));
    end
end
fprintf(fid, '\n## 附表（全程指标与耗时）\n\n');
fprintf(fid, '见 `revision_random_detail.csv`；其中 `load_pos_rms_full` 为全程窗口，`tension_min_all` 为全程最小有符号张力。\n');
end

%% ---------- 小工具 ----------

function write_periodic_comparison(fid, opts, T)
% 与已完成的周期扰动实验对照（revision_runs/drag 的 cd=0，与四组消融同参数）
ref_path = fullfile(fileparts(opts.output_root), 'drag', 'revision_drag_summary.csv');
if ~exist(ref_path, 'file')
    fprintf(fid, '未找到周期扰动基准 `%s`，跳过对照。\n', ref_path);
    return
end
R = readtable(ref_path, 'TextType', 'string');
methods_all = {'BS', 'BS_RBF', 'BS_ESO', 'BS_RBF_ESO'};
ref_pos = nan(1, 4);
for k = 1:4
    idx = find(strcmp(R.method, methods_all{k}) & R.drag_cd == 0, 1);
    if ~isempty(idx)
        ref_pos(k) = R.load_pos_rms(idx);
    end
end
if any(~isfinite(ref_pos))
    fprintf(fid, '周期扰动基准不完整，跳过对照。\n');
    return
end
fprintf(fid, '参考工况：同一参数集、周期扰动（4 正弦，drag cd = 0，即四组消融的"有扰动"一组）。\n');
fprintf(fid, '两类扰动的 RMS 量级不同（周期 d_p 的 RMS 约 2.15 m/s²，随机 d_p 为 1.0 m/s²），');
fprintf(fid, '因此**只比较排序与比值，不比较绝对误差**。\n\n');
fprintf(fid, '| method | 周期扰动 load_pos_rms (m) | 随机扰动 load_pos_rms mean (m) |\n');
fprintf(fid, '|---|---|---|\n');
for k = 1:4
    sel = strcmp(T.method, methods_all{k}) & strcmp(T.status, 'ok');
    fprintf(fid, '| %s | %s | %s |\n', methods_all{k}, fmt(ref_pos(k)), ...
        fmt(mean_or_nan(T.load_pos_rms(sel))));
end
ratio_periodic = ref_pos(4) / ref_pos(3);
sel_e = strcmp(T.method, 'BS_ESO') & strcmp(T.status, 'ok');
sel_c = strcmp(T.method, 'BS_RBF_ESO') & strcmp(T.status, 'ok');
ratio_random = mean_or_nan(T.load_pos_rms(sel_c)) / mean_or_nan(T.load_pos_rms(sel_e));
fprintf(fid, '\n- 复合 / ESO-only：周期 %.4f（%.1f%%），随机 %.4f（%.1f%%）\n', ...
    ratio_periodic, 100 * (1 - ratio_periodic), ratio_random, 100 * (1 - ratio_random));
[~, ord_p] = sort(ref_pos);
[~, ord_r] = sort(arrayfun(@(k) mean_or_nan(T.load_pos_rms(strcmp(T.method, methods_all{k}) & ...
    strcmp(T.status, 'ok'))), 1:4));
fprintf(fid, '- 排序（升序）：周期 %s；随机 %s\n', strjoin(methods_all(ord_p), ' < '), ...
    strjoin(methods_all(ord_r), ' < '));
fprintf(fid, '- 排序一致：%d\n', isequal(ord_p(:), ord_r(:)));
end

function s = fmt(v)
if isnan(v)
    s = 'NaN';
else
    s = sprintf('%.6g', v);
end
end

function s = esc(v)
% 失败原因里可能带 %（如 MatLab 错误信息），写 markdown 前先转义
if isstring(v)
    v = char(v);
end
s = strrep(v, '%', '%%');
end

function s = mean_std(v)
if isempty(v)
    s = 'n/a';
else
    s = sprintf('%.6g ± %.3g', mean(v), std(v));
end
end

function v = mean_or_nan(v)
if isempty(v) || all(~isfinite(v))
    v = NaN;
else
    v = mean(v(isfinite(v)));
end
end

function v = min_or_nan(v)
if isempty(v) || all(~isfinite(v))
    v = NaN;
else
    v = min(v(isfinite(v)));
end
end

function s = ternary(cond, a, b)
if cond
    s = a;
else
    s = b;
end
end

function value = get_metric(m, name)
if isstruct(m) && isfield(m, name) && ~isempty(m.(name)) && isnumeric(m.(name))
    value = double(m.(name));
else
    value = NaN;
end
end

function [opts, unknown] = parse_options(opts, args)
unknown = {};
if mod(numel(args), 2) ~= 0
    error('TIMC:Random:BadOptions', 'Options must be name-value pairs.');
end
for k = 1:2:numel(args)
    name = args{k};
    if ~ischar(name) && ~isstring(name)
        error('TIMC:Random:BadOptions', 'Option name must be a char/string.');
    end
    name = lower(char(name));
    if ~isfield(opts, name)
        unknown{end + 1} = name; %#ok<AGROW>
        continue
    end
    opts.(name) = args{k + 1};
end
end
