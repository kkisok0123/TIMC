function [Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, theta_rbf, fn, fn_att, h_rbf_all, eso_state] = ...
    load_lifting_payload_control_backstepping_rbf_eso(x_L, Des_Att, Des_pL, RL_d, theta_rbf_prev, dt, x_L_prev, with_disturbance, controller_config, eso_state_prev, u_prev)
% Backstepping + RBF + ESO payload controller wrapper.

if nargin < 9 || isempty(controller_config)
    controller_config = struct();
end
if ~isfield(controller_config, 'rbf') || ~isstruct(controller_config.rbf)
    controller_config.rbf = struct();
end
controller_config.rbf.base_mode = 'backstepping';

[Fd, Md, e_RL, e_Ome_L, e_pL, e_vL, theta_rbf, fn, fn_att, h_rbf_all, eso_state] = ...
    load_lifting_payload_control_rbf(x_L, Des_Att, Des_pL, RL_d, theta_rbf_prev, dt, x_L_prev, ...
                                     with_disturbance, controller_config, eso_state_prev, u_prev);
end
