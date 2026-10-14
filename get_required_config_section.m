function section = get_required_config_section(root_config, field_path, context_label)
% Return a nested config section or raise a clear error.

if nargin < 3 || isempty(context_label)
    context_label = 'Controller';
end

if nargin < 2 || isempty(field_path)
    error('%s requires a non-empty config field path.', context_label);
end

if ~iscell(field_path)
    field_path = {field_path};
end

section = root_config;
resolved_path = cell(1, numel(field_path));

for idx = 1:numel(field_path)
    field_name = field_path{idx};
    resolved_path{idx} = field_name;

    if ~isstruct(section) || ~isfield(section, field_name) || isempty(section.(field_name))
        error(['%s missing required config field "%s". ' ...
               'Load parameters via load_lifting_controller_params.m and pass controller_config through the payload controller interface.'], ...
              context_label, strjoin(resolved_path, '.'));
    end

    section = section.(field_name);
end
end
