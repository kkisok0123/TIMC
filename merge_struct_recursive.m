function base = merge_struct_recursive(base, override)
% Recursively merge struct fields from override into base.

if isempty(override)
    return;
end

override_fields = fieldnames(override);
for idx = 1:numel(override_fields)
    field_name = override_fields{idx};
    override_value = override.(field_name);

    if isstruct(override_value) && isfield(base, field_name) && isstruct(base.(field_name)) && ...
            isscalar(override_value) && isscalar(base.(field_name))
        base.(field_name) = merge_struct_recursive(base.(field_name), override_value);
    else
        base.(field_name) = override_value;
    end
end
end
