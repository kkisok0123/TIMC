function controller_config = load_current_payload_bsrbf_eso_config(with_disturbance, root_dir)
% Load the current best payload BS+RBF+ESO controller config.

if nargin < 2 || isempty(root_dir)
    root_dir = pwd;
end

txt_file = get_payload_txt_path(with_disturbance, root_dir);
if isfile(txt_file)
    controller_config = load_payload_config_from_txt(txt_file, with_disturbance);
    return;
end

if with_disturbance
    controller_config = load_best_comparison_config(root_dir, true);
else
    controller_config = load_best_comparison_config(root_dir, false);
end
end

function txt_file = get_payload_txt_path(with_disturbance, root_dir)
if with_disturbance
    txt_file = fullfile(root_dir, '负载参数_Backstepping+RBF+ESO_有干扰_当前最优.txt');
else
    txt_file = fullfile(root_dir, '负载参数_Backstepping+RBF+ESO_无干扰_当前最优.txt');
end
end

function controller_config = load_payload_config_from_txt(txt_file, with_disturbance)
data = parse_key_value_txt(txt_file);
base_rbf = load_lifting_controller_params('rbf', with_disturbance, struct('uav_controller_type', 'rise'), ...
    struct('control_flags', struct('disable_saved_overrides', true)));
base_bs = load_lifting_controller_params('backstepping', with_disturbance, struct('uav_controller_type', 'rise'), ...
    struct('control_flags', struct('disable_saved_overrides', true)));

controller_config = struct();
controller_config.controller_type = 'rbf';
controller_config.with_disturbance = with_disturbance;
controller_config.allocation = base_rbf.allocation;
controller_config.allocation.KsOmega_i = reshape(get_numeric_value(data, 'allocation.KsOmega_i'), [], 1);
controller_config.allocation.Kq_i = reshape(get_numeric_value(data, 'allocation.Kq_i'), [], 1);
controller_config.allocation.alpha_filter = get_scalar_value(data, 'allocation.alpha_filter');

controller_config.payload_control = struct();
controller_config.payload_control.backstepping = struct( ...
    'c1', diag(get_numeric_value(data, 'backstepping.c1')), ...
    'c2', diag(get_numeric_value(data, 'backstepping.c2')), ...
    'c3', diag(get_numeric_value(data, 'backstepping.c3')), ...
    'c4', diag(get_numeric_value(data, 'backstepping.c4')));

rbf = base_rbf.rbf;
rbf.base_mode = get_string_value(data, 'rbf.base_mode', 'backstepping');
rbf.enable_compensation = logical(get_scalar_value(data, 'rbf.enable_compensation', 1));
rbf.enable_adaptation = logical(get_scalar_value(data, 'rbf.enable_adaptation', 1));
rbf.compensation_scale = get_scalar_value(data, 'rbf.compensation_scale', 1.0);
if has_key(data, 'rbf.comp_pos')
    rbf.compensation_scale_pos = get_scalar_value(data, 'rbf.comp_pos');
else
    rbf.compensation_scale_pos = rbf.compensation_scale;
end
if has_key(data, 'rbf.comp_att')
    rbf.compensation_scale_att = get_scalar_value(data, 'rbf.comp_att');
else
    rbf.compensation_scale_att = rbf.compensation_scale;
end
rbf.b_rbf = reshape(get_numeric_value(data, 'rbf.b_rbf'), [], 1);
if has_key(data, 'rbf.theta_init_pos_unique')
    theta_pos = get_scalar_value(data, 'rbf.theta_init_pos_unique');
else
    theta_pos = get_scalar_value(data, 'rbf.theta_rbf_init_pos_unique');
end
if has_key(data, 'rbf.theta_init_att_unique')
    theta_att = get_scalar_value(data, 'rbf.theta_init_att_unique');
else
    theta_att = get_scalar_value(data, 'rbf.theta_rbf_init_att_unique');
end
theta_init = base_rbf.rbf.theta_rbf_init(:);
theta_init(1:15) = theta_pos;
theta_init(16:30) = theta_att;
rbf.theta_rbf_init = theta_init;
rbf.theta_max = get_scalar_value(data, 'rbf.theta_max');
rbf.theta_norm_max = get_scalar_value(data, 'rbf.theta_norm_max');
if has_key(data, 'rbf.fn_max')
    rbf.output_limits.fn_max = get_scalar_value(data, 'rbf.fn_max');
end
if has_key(data, 'rbf.fn_att_max')
    rbf.output_limits.fn_att_max = get_scalar_value(data, 'rbf.fn_att_max');
end
if has_key(data, 'adaptation.gama')
    rbf.adaptation.gama = get_scalar_value(data, 'adaptation.gama');
    rbf.adaptation.gama_z = get_scalar_value(data, 'adaptation.gama_z');
    rbf.adaptation.gama_att = get_scalar_value(data, 'adaptation.gama_att', rbf.adaptation.gama_att);
    rbf.adaptation.k1 = get_scalar_value(data, 'adaptation.k1');
    rbf.adaptation.k1_att = get_scalar_value(data, 'adaptation.k1_att', rbf.adaptation.k1_att);
    rbf.adaptation.sigma = get_scalar_value(data, 'adaptation.sigma');
    rbf.adaptation.sigma_z = get_scalar_value(data, 'adaptation.sigma_z');
    rbf.adaptation.sigma_att = get_scalar_value(data, 'adaptation.sigma_att', rbf.adaptation.sigma_att);
    rbf.adaptation.Q_i = diag(get_numeric_value(data, 'adaptation.Q_i_diag'));
    rbf.adaptation.Q_att = diag(get_numeric_value(data, 'adaptation.Q_att_diag', diag(rbf.adaptation.Q_att).'));
    rbf.adaptation.kp_pos_xy = get_scalar_value(data, 'adaptation.kp_pos_xy');
    rbf.adaptation.kv_pos_xy = get_scalar_value(data, 'adaptation.kv_pos_xy');
    rbf.adaptation.kp_pos_z = get_scalar_value(data, 'adaptation.kp_pos_z');
    rbf.adaptation.kv_pos_z = get_scalar_value(data, 'adaptation.kv_pos_z');
    rbf.adaptation.kp_att = get_scalar_value(data, 'adaptation.kp_att');
    rbf.adaptation.kv_att = get_scalar_value(data, 'adaptation.kv_att');
end
controller_config.rbf = rbf;

controller_config.eso = base_rbf.eso;
controller_config.eso.use_compensation = logical(get_scalar_value(data, 'eso.use_compensation', 1));
controller_config.eso.compensation_scale = get_scalar_value(data, 'eso.compensation_scale', 1.0);
controller_config.eso.compensation_scale_pos = get_scalar_value(data, 'eso.compensation_scale_pos', get_scalar_value(data, 'eso.comp_pos', controller_config.eso.compensation_scale));
controller_config.eso.compensation_scale_att = get_scalar_value(data, 'eso.compensation_scale_att', get_scalar_value(data, 'eso.comp_att', controller_config.eso.compensation_scale));
controller_config.eso.beta_pos = repmat(get_numeric_value(data, 'eso.beta_pos_row1'), 3, 1);
controller_config.eso.beta_att = repmat(get_numeric_value(data, 'eso.beta_att_row1'), 3, 1);
controller_config.eso.b_pos = reshape(get_numeric_value(data, 'eso.b_pos'), [], 1);
controller_config.eso.b_att = reshape(get_numeric_value(data, 'eso.b_att'), [], 1);
end

function controller_config = load_best_comparison_config(root_dir, with_disturbance)
comparison_root = fullfile(root_dir, 'payload_backstepping_rbf_eso_comparison');
files = dir(fullfile(comparison_root, '**', 'payload_controller_param_set.mat'));
if isempty(files)
    error('No payload comparison param sets found under %s', comparison_root);
end

best_idx = [];
best_time = -inf;
for idx = 1:numel(files)
    file_path = fullfile(files(idx).folder, files(idx).name);
    try
        S = load(file_path, 'payload_controller_param_set');
        cfg = S.payload_controller_param_set.backstepping.controller_config;
        if cfg.with_disturbance ~= with_disturbance
            continue;
        end
        if files(idx).datenum > best_time
            best_time = files(idx).datenum;
            best_idx = idx;
        end
    catch
    end
end

if isempty(best_idx)
    error('No matching payload comparison config found for with_disturbance=%d', with_disturbance);
end

S = load(fullfile(files(best_idx).folder, files(best_idx).name), 'payload_controller_param_set');
controller_config = S.payload_controller_param_set.backstepping_rbf_eso.controller_config;
end

function controller_config = build_payload_bsrbf_eso_config(candidate, with_disturbance)
base_rbf = load_lifting_controller_params('rbf', with_disturbance, struct('uav_controller_type', 'rise'), ...
    struct('control_flags', struct('disable_saved_overrides', true)));
base_bs = load_lifting_controller_params('backstepping', with_disturbance, struct('uav_controller_type', 'rise'), ...
    struct('control_flags', struct('disable_saved_overrides', true)));
load_lifting_parameters;
global m_L b1_L b2_L b3_L

controller_config = struct();
controller_config.controller_type = 'rbf';
controller_config.with_disturbance = with_disturbance;
controller_config.allocation = base_rbf.allocation;
controller_config.allocation.KsOmega_i = base_rbf.allocation.KsOmega_i(:) .* candidate.ks_scale(:);
controller_config.allocation.Kq_i = base_rbf.allocation.Kq_i(:) .* candidate.kq_scale(:);
controller_config.allocation.alpha_filter = min(max(base_rbf.allocation.alpha_filter + candidate.alpha_delta, 0.05), 0.90);

bs = base_bs.payload_control.backstepping;
controller_config.payload_control = struct();
controller_config.payload_control.backstepping = struct( ...
    'c1', diag(diag(bs.c1) .* candidate.bs_c1_scale(:)), ...
    'c2', diag(diag(bs.c2) .* candidate.bs_c2_scale(:)), ...
    'c3', diag(diag(bs.c3) .* candidate.bs_c3_scale(:)), ...
    'c4', diag(diag(bs.c4) .* candidate.bs_c4_scale(:)));

rbf = base_rbf.rbf;
theta_init = rbf.theta_rbf_init(:);
theta_init(1:15) = theta_init(1:15) * candidate.theta_pos_scale;
theta_init(16:30) = theta_init(16:30) * candidate.theta_att_scale;
rbf.base_mode = 'backstepping';
rbf.enable_compensation = true;
rbf.enable_adaptation = true;
rbf.compensation_scale = candidate.rbf_comp_scale;
rbf.compensation_scale_pos = candidate.rbf_comp_scale;
rbf.compensation_scale_att = get_candidate_field(candidate, 'rbf_comp_scale_att', candidate.rbf_comp_scale);
rbf.b_rbf = rbf.b_rbf(:) * candidate.rbf_width_scale;
rbf.theta_rbf_init = theta_init;
rbf.theta_max = rbf.theta_max * candidate.theta_max_scale;
rbf.theta_norm_max = rbf.theta_norm_max * candidate.theta_norm_scale;
rbf.output_limits.fn_max = rbf.output_limits.fn_max * candidate.fn_max_scale;
rbf.output_limits.fn_att_max = rbf.output_limits.fn_att_max * candidate.fn_att_max_scale;
adapt = rbf.adaptation;
adapt.gama = adapt.gama * candidate.gamma_scale;
adapt.gama_z = adapt.gama_z * candidate.gamma_z_scale;
adapt.gama_att = adapt.gama_att * get_candidate_field(candidate, 'gamma_att_scale', 1.0);
adapt.k1 = adapt.k1 * candidate.adapt_k1_scale;
adapt.k1_att = adapt.k1_att * get_candidate_field(candidate, 'adapt_k1_att_scale', 1.0);
adapt.sigma = adapt.sigma * candidate.sigma_scale;
adapt.sigma_z = adapt.sigma_z * candidate.sigma_z_scale;
adapt.sigma_att = adapt.sigma_att * get_candidate_field(candidate, 'sigma_att_scale', 1.0);
adapt.Q_i = adapt.Q_i * candidate.Q_scale;
adapt.Q_att = adapt.Q_att * get_candidate_field(candidate, 'Q_att_scale', 1.0);
adapt.kp_pos_xy = adapt.kp_pos_xy * candidate.kp_pos_xy_scale;
adapt.kv_pos_xy = adapt.kv_pos_xy * candidate.kv_pos_xy_scale;
adapt.kp_pos_z = adapt.kp_pos_z * candidate.kp_pos_z_scale;
adapt.kv_pos_z = adapt.kv_pos_z * candidate.kv_pos_z_scale;
adapt.kp_att = adapt.kp_att * candidate.kp_att_scale;
adapt.kv_att = adapt.kv_att * candidate.kv_att_scale;
rbf.adaptation = adapt;
controller_config.rbf = rbf;

omega_pos = base_rbf.eso.beta_pos(1, 1) / 3 * candidate.eso_pos_bandwidth_scale;
omega_att = base_rbf.eso.beta_att(1, 1) / 3 * candidate.eso_att_bandwidth_scale;
controller_config.eso = struct();
controller_config.eso.beta_pos = [3 * omega_pos * ones(3, 1), 3 * omega_pos^2 * ones(3, 1), omega_pos^3 * ones(3, 1)];
controller_config.eso.b_pos = [1 / m_L; 1 / m_L; 1 / m_L];
controller_config.eso.beta_att = [3 * omega_att * ones(3, 1), 3 * omega_att^2 * ones(3, 1), omega_att^3 * ones(3, 1)];
controller_config.eso.b_att = [1 / b1_L; 1 / b2_L; 1 / b3_L];
controller_config.eso.use_compensation = true;
controller_config.eso.compensation_scale = candidate.eso_comp_scale;
controller_config.eso.compensation_scale_pos = candidate.eso_comp_scale;
controller_config.eso.compensation_scale_att = get_candidate_field(candidate, 'eso_comp_scale_att', candidate.eso_comp_scale);
end

function value = get_candidate_field(candidate, field_name, default_value)
if isfield(candidate, field_name) && ~isempty(candidate.(field_name))
    value = candidate.(field_name);
else
    value = default_value;
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

function value = get_string_value(data, key, default_value)
sanitized_key = matlab.lang.makeValidName(key);
if isfield(data, sanitized_key)
    value = char(string(data.(sanitized_key)));
else
    value = default_value;
end
end
