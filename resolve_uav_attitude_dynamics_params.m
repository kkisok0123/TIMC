function [a_p, b_p, inertia_diag] = resolve_uav_attitude_dynamics_params(controller_config)
% Resolve the UAV attitude dynamics coefficients used by the M controllers.

global Jx_p Jy_p Jz_p
global a1_p a2_p a3_p
global b1_p b2_p b3_p
global l_i

inertia_diag = [Jx_p; Jy_p; Jz_p];
arm_length = l_i;
a_p = [a1_p; a2_p; a3_p];
b_p = [b1_p; b2_p; b3_p];

if nargin < 1 || isempty(controller_config) || ~isstruct(controller_config)
    return;
end

if isfield(controller_config, 'uav_model') && isstruct(controller_config.uav_model)
    uav_model = controller_config.uav_model;

    if isfield(uav_model, 'arm_length') && ~isempty(uav_model.arm_length)
        arm_length = uav_model.arm_length;
    end

    if isfield(uav_model, 'inertia_diag') && ~isempty(uav_model.inertia_diag)
        inertia_diag = reshape(uav_model.inertia_diag, 3, 1);
    end

    if isfield(uav_model, 'a_p') && ~isempty(uav_model.a_p) && ...
            isfield(uav_model, 'b_p') && ~isempty(uav_model.b_p)
        a_p = reshape(uav_model.a_p, 3, 1);
        b_p = reshape(uav_model.b_p, 3, 1);
    else
        [a_p, b_p] = compute_ab_from_inertia(inertia_diag, arm_length);
    end
end
end
