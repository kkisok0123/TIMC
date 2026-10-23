%% 汇总图：参数失配下的位置 RMS（对数纵轴）
% 数据来源：revision_runs/mismatch 与 revision_runs/mismatch_extreme 的汇总 CSV

clear; clc;
root_dir = fileparts(mfilename('fullpath'));
if isempty(root_dir), root_dir = pwd; end
cd(root_dir);

f1 = fullfile(root_dir, 'revision_runs', 'mismatch', 'revision_mismatch_summary.csv');
f2 = fullfile(root_dir, 'revision_runs', 'mismatch_extreme', 'mismatch_extreme_summary.csv');
if ~isfile(f1) || ~isfile(f2)
    error('先运行 run_revision_mismatch_cases.m 与 run_revision_mismatch_extreme.m');
end

T1 = readtable(f1, 'VariableNamingRule', 'preserve');
T1 = renamevars(T1, 'case', 'method');
T2 = readtable(f2, 'VariableNamingRule', 'preserve');
T2 = renamevars(T2, 'case', 'method');
T = [T1(:, {'method', 'mass_inertia_scale', 'load_pos_rms'}); ...
     T2(:, {'method', 'mass_inertia_scale', 'load_pos_rms'})];

method_names = {'BS', 'BS_RBF', 'BS_ESO', 'BS_RBF_ESO'};
method_labels = {'Backstepping', 'BS+RBF', 'BS+ESO', 'BS+RBF+ESO'};
styles = {'-o', '-s', '-^', '-d'};
colors = [0.20 0.20 0.20; 0.85 0.45 0.10; 0.10 0.45 0.75; 0.65 0.15 0.15];

scales = unique(T.mass_inertia_scale);

fig = figure('Position', [100 100 900 520], 'Color', 'w');
hold on;
for c = 1:numel(method_names)
    y = nan(size(scales));
    for k = 1:numel(scales)
        idx = strcmp(T.method, method_names{c}) & abs(T.mass_inertia_scale - scales(k)) < 1e-9;
        if any(idx)
            y(k) = T.load_pos_rms(find(idx, 1));
        end
    end
    semilogy(scales, y, styles{c}, 'Color', colors(c,:), 'LineWidth', 1.8, ...
        'MarkerSize', 8, 'MarkerFaceColor', colors(c,:), 'DisplayName', method_labels{c});
end
set(gca, 'YScale', 'log', 'FontSize', 12);
xlabel('True mass / inertia scale (controller keeps nominal model)', 'FontSize', 12);
ylabel('Position RMS (m, log scale)', 'FontSize', 12);
title('Payload tracking error under parameter mismatch (periodic disturbance)', 'FontSize', 13);
grid on; box on;
legend('Location', 'northwest', 'FontSize', 11);
xticks(scales);
xlim([0.45 1.55]);

out = fullfile(root_dir, 'revision_runs', 'revision_mismatch_summary.png');
exportgraphics(fig, out, 'Resolution', 160);
fprintf('Figure saved: %s\n', out);

fprintf('\nComposite / ESO-only ratio:\n');
for k = 1:numel(scales)
    i_e = strcmp(T.method, 'BS_ESO') & abs(T.mass_inertia_scale - scales(k)) < 1e-9;
    i_c = strcmp(T.method, 'BS_RBF_ESO') & abs(T.mass_inertia_scale - scales(k)) < 1e-9;
    if any(i_e) && any(i_c)
        fprintf('  scale %.1f : %.4f\n', scales(k), ...
            T.load_pos_rms(find(i_c,1)) / T.load_pos_rms(find(i_e,1)));
    end
end
