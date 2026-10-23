function controller_config = load_current_payload_bsrbf_eso_config(with_disturbance, root_dir)
% Load a payload BS+RBF+ESO controller config from a legacy "current best" txt file.
%
% 返修说明：RBF 已改为"每通道二维输入、单套归一化基函数、固定学习率"，
% ESO 已改为以 Fin/Min 为输入、显式处理重力与姿态欧拉角加速度映射。
% 旧 txt 中与新结构不兼容的字段（6 维中心、未归一化宽度、误差相关学习率、
% 输出裁剪、ESO 输入系数等）不再生效，这里只采纳仍然有意义的字段：
%   allocation.*, backstepping.c1..c4, rbf 的开关/投影半径/学习率/泄漏项,
%   eso 的带宽与补偿比例。

if nargin < 2 || isempty(root_dir)
    root_dir = pwd;
end

controller_config = load_lifting_controller_params('rbf', with_disturbance);

txt_file = get_payload_txt_path(with_disturbance, root_dir);
if ~isfile(txt_file)
    return;
end

data = parse_key_value_txt(txt_file);
ignored = {};

bs_cfg = controller_config.payload_control.backstepping;
if has_key(data, 'backstepping.c1') && has_key(data, 'backstepping.c2') && ...
        has_key(data, 'backstepping.c3') && has_key(data, 'backstepping.c4')
    controller_config.payload_control.backstepping = struct( ...
        'c1', diag(get_numeric_value(data, 'backstepping.c1')), ...
        'c2', diag(get_numeric_value(data, 'backstepping.c2')), ...
        'c3', diag(get_numeric_value(data, 'backstepping.c3')), ...
        'c4', diag(get_numeric_value(data, 'backstepping.c4')));
else
    ignored{end+1} = 'backstepping.c1..c4 (incomplete)';
end

if has_key(data, 'allocation.KsOmega_i')
    controller_config.allocation.KsOmega_i = reshape(get_numeric_value(data, 'allocation.KsOmega_i'), [], 1);
end
if has_key(data, 'allocation.Kq_i')
    controller_config.allocation.Kq_i = reshape(get_numeric_value(data, 'allocation.Kq_i'), [], 1);
end
if has_key(data, 'allocation.alpha_filter')
    controller_config.allocation.alpha_filter = get_scalar_value(data, 'allocation.alpha_filter');
end

% RBF：只采纳与二维统一结构兼容的量
if has_key(data, 'adaptation.gama')
    controller_config.rbf.adaptation.gama = get_scalar_value(data, 'adaptation.gama');
end
if has_key(data, 'adaptation.gama_att')
    controller_config.rbf.adaptation.gama_att = get_scalar_value(data, 'adaptation.gama_att');
end
if has_key(data, 'adaptation.sigma')
    controller_config.rbf.adaptation.sigma = get_scalar_value(data, 'adaptation.sigma');
end
if has_key(data, 'rbf.theta_norm_max')
    controller_config.rbf.theta_norm_max = get_scalar_value(data, 'rbf.theta_norm_max');
end
if has_key(data, 'rbf.enable_compensation')
    controller_config.rbf.enable_compensation = logical(get_scalar_value(data, 'rbf.enable_compensation'));
end
if has_key(data, 'rbf.enable_adaptation')
    controller_config.rbf.enable_adaptation = logical(get_scalar_value(data, 'rbf.enable_adaptation'));
end

% ESO：带宽与补偿比例可沿用，输入系数与姿态映射由代码固定
if has_key(data, 'eso.beta_pos_row1')
    controller_config.eso.beta_pos = repmat(get_numeric_value(data, 'eso.beta_pos_row1'), 3, 1);
end
if has_key(data, 'eso.beta_att_row1')
    controller_config.eso.beta_att = repmat(get_numeric_value(data, 'eso.beta_att_row1'), 3, 1);
end
if has_key(data, 'eso.compensation_scale')
    controller_config.eso.compensation_scale = get_scalar_value(data, 'eso.compensation_scale');
end

legacy_keys = {'rbf.comp_pos', 'rbf.comp_att', 'rbf.b_rbf', 'rbf.theta_max', ...
    'rbf.fn_max', 'rbf.fn_att_max', 'eso.b_pos', 'eso.b_att', ...
    'eso.comp_pos', 'eso.comp_att', 'adaptation.k1', 'adaptation.k1_att', ...
    'adaptation.sigma_z', 'adaptation.sigma_att'};
ignored = [ignored, legacy_keys(has_key_cell(data, legacy_keys))];

if ~isempty(ignored)
    warning('TIMC:LegacyConfigKeys', ...
        'Ignoring %d legacy config key(s) incompatible with the revised controller: %s', ...
        numel(ignored), strjoin(ignored, ', '));
end
end

function txt_file = get_payload_txt_path(with_disturbance, root_dir)
if with_disturbance
    txt_file = fullfile(root_dir, '负载参数_Backstepping+RBF+ESO_有干扰_当前最优.txt');
else
    txt_file = fullfile(root_dir, '负载参数_Backstepping+RBF+ESO_无干扰_当前最优.txt');
end
end

function data = parse_key_value_txt(txt_file)
data = struct();
lines = splitlines(fileread(txt_file));
for idx = 1:numel(lines)
    line = strtrim(lines{idx});
    if isempty(line) || ~contains(line, '=')
        continue;
    end
    parts = split(line, '=', 2);
    key = strtrim(parts{1});
    value = strtrim(parts{2});
    data.(matlab.lang.makeValidName(key)) = value;
end
end

function tf = has_key(data, key)
tf = isfield(data, matlab.lang.makeValidName(key));
end

function mask = has_key_cell(data, keys)
mask = false(1, numel(keys));
for idx = 1:numel(keys)
    mask(idx) = has_key(data, keys{idx});
end
end

function value = get_numeric_value(data, key, default_value)
if nargin < 3
    default_value = [];
end
sanitized_key = matlab.lang.makeValidName(key);
if isfield(data, sanitized_key)
    value = str2num(data.(sanitized_key)); %#ok<ST2NM>
else
    value = default_value;
end
end

function value = get_scalar_value(data, key, default_value)
if nargin < 3
    default_value = [];
end
numeric_value = get_numeric_value(data, key, default_value);
if isempty(numeric_value)
    error('Missing payload txt key: %s', key);
end
value = numeric_value(1);
end
