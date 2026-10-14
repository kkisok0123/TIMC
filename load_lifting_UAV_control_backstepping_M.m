function M_plant = load_lifting_UAV_control_backstepping_M(x_att, x_ome, angled_d, Omegaid_d, Omegaid_d_dot, controller_config, q_num)
%% 无人机姿态反步控制器：计算控制力矩 M_plant

[a_p, b_p] = resolve_uav_attitude_dynamics_params(controller_config);
bi_inv = diag(1 ./ b_p);

if nargin < 7 || isempty(q_num)
    q_num = 1;
end

cfg = get_required_config_section(controller_config, {'uav_control', 'uav_M_backstepping'}, 'UAV backstepping M controller');
c1 = pick_gain_matrix(cfg.c1, q_num);
c2 = pick_gain_matrix(cfg.c2, q_num);

% Inputs Omegaid_d/Omegaid_d_dot are desired BODY rates and derivatives.
[T, T_dot, eta_dot] = zyx_euler_kinematics(x_att, x_ome);
[Td, Td_dot, eta_d_dot] = zyx_euler_kinematics(angled_d, Omegaid_d);
eta_d_ddot = Td_dot * Omegaid_d + Td * Omegaid_d_dot;
z1 = atan2(sin(x_att - angled_d), cos(x_att - angled_d));
z1_dot = eta_dot - eta_d_dot;
z2 = z1_dot + c1 * z1;
eta_ddot_cmd = eta_d_ddot - c1 * z1_dot - z1 - c2 * z2;
ome_cross = [x_ome(2) * x_ome(3); x_ome(1) * x_ome(3); x_ome(1) * x_ome(2)];
coriolis_term = diag(a_p) * ome_cross;
M_plant = bi_inv * (T \ (eta_ddot_cmd - T_dot * x_ome) - coriolis_term);

end

function gain = pick_gain_matrix(raw_gain, q_num)
if ndims(raw_gain) == 3
    gain = raw_gain(:, :, q_num);
else
    gain = raw_gain;
end
end
