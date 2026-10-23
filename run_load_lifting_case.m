function run_result = run_load_lifting_case(controller_type, with_disturbance, options)
%% 单次仿真入口：设置覆盖项后运行 load_lifting_main 脚本并返回 run_result
%
% 用法：
%   run_result = run_load_lifting_case('rbf', false, struct(...));
%
% options 字段（全部可选）：
%   controller_param_override : 传给 load_lifting_controller_params 的覆盖结构体
%   uav_controller_type       : 无人机内环控制器类型，默认 'backstepping'
%   save_run_data             : 是否保存 MAT 结果，默认 true
%   run_output_dir            : 结果目录，默认 fullfile(pwd,'simulation_runs')
%   enable_default_plots      : 是否绘制主函数默认图，默认 false
%   dt / Tf                   : 仿真步长与时长覆盖
%
% 注意：load_lifting_main.m 是脚本，内部会用 clearvars 清理工作区（保留覆盖项），
% 因此所有选项必须在本函数内先转成脚本认识的变量名。

if nargin < 2 || isempty(with_disturbance)
    with_disturbance = false;
end
if nargin < 3 || isempty(options)
    options = struct();
end
if ~isstruct(options)
    error('TIMC:InvalidOptions', 'options must be a struct.');
end

controller_type_override = controller_type;
with_disturbance_override = logical(with_disturbance);
controller_param_override = get_option(options, 'controller_param_override', []);
uav_controller_type_override = get_option(options, 'uav_controller_type', 'backstepping');
uav_F_type_override = get_option(options, 'uav_F_type', []);
uav_M_type_override = get_option(options, 'uav_M_type', []);
save_run_data = get_option(options, 'save_run_data', true);
run_output_dir = get_option(options, 'run_output_dir', fullfile(pwd, 'simulation_runs'));
enable_default_plots = get_option(options, 'enable_default_plots', false);
simulation_dt_override = get_option(options, 'dt', []);
simulation_Tf_override = get_option(options, 'Tf', []);

clear options;

run('load_lifting_main.m');

if ~exist('run_result', 'var')
    error('TIMC:MissingRunResult', 'load_lifting_main.m did not produce run_result.');
end
end

function value = get_option(options, field_name, default_value)
if isfield(options, field_name) && ~isempty(options.(field_name))
    value = options.(field_name);
else
    value = default_value;
end
end
