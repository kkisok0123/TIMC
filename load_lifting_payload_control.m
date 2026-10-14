function [Fd, Md, e_RL, e_Ome_L, e_p_L, e_v_L, RL] = load_lifting_payload_control(x_L, Des_Att, Des_pL, RL_d, controller_config)
%% Legacy payload controller entry point.
% This function is kept as a thin wrapper so older callers still route
% through the unified PD parameter loading path.

if nargin < 5 || isempty(controller_config)
    error(['load_lifting_payload_control requires controller_config from ' ...
           'load_lifting_controller_params.m.']);
end

[Fd, Md, e_RL, e_Ome_L, e_p_L, e_v_L, RL] = ...
    load_lifting_payload_control_pd(x_L, Des_Att, Des_pL, RL_d, controller_config);
end
