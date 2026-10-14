function [Fd, e_PosL, e_VelL] = load_lifting_UAV_control_backstepping_F(x_i, DesPos, controller_config, q_num)
% UAV position backstepping controller.

global g m_p e3

if nargin < 4 || isempty(q_num)
    q_num = 1;
end

Pos_L = [x_i(7); x_i(9); x_i(11)];
Vel_L = [x_i(8); x_i(10); x_i(12)];

PosL_d = DesPos(:, 1);
PosL_d_dot = DesPos(:, 2);
PosL_d_2dot = zeros(3, 1);

e_PosL = Pos_L - PosL_d;
e_VelL = Vel_L - PosL_d_dot;

cfg = get_required_config_section(controller_config, {'uav_control', 'uav_F_backstepping'}, 'UAV backstepping F controller');
c3 = pick_gain_matrix(cfg.c3, q_num);
c4 = pick_gain_matrix(cfg.c4, q_num);

z3 = Pos_L - PosL_d;
z3_dot = Vel_L - PosL_d_dot;
alpha2_dot = PosL_d_2dot - c3 * z3_dot;
z4 = z3_dot + c3 * z3;

Fd = m_p(q_num) * (-z3 + g * e3 + alpha2_dot - c4 * z4);

end

function gain = pick_gain_matrix(raw_gain, q_num)
if isempty(raw_gain)
    gain = [];
elseif ndims(raw_gain) == 3
    gain = raw_gain(:, :, q_num);
else
    gain = raw_gain;
end
end
