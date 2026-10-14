% Global parameter setup for the load-lifting simulation.
global m_L g m_p1 m_p2 m_p3 m_p4 m_p
global J_L l_L
global Jx_L Jy_L Jz_L
global a1_L a2_L a3_L
global b1_L b2_L b3_L
global J1_p J2_p J3_p J_p
global Jx_p Jy_p Jz_p
global a1_p a2_p a3_p l_i
global b1_p b2_p b3_p l_plant
global Quadrotor_Num
global load_length load_width load_height

dt = 0.001;
Tf = 20;

Quadrotor_Num = 3;
m_p_base = 2;
l_i = 1;

% UAV parameters
m_p = repmat(m_p_base, 1, 4);
m_p1 = m_p(1);
m_p2 = m_p(2);
m_p3 = m_p(3);
m_p4 = m_p(4);

l_plant = ones(1, Quadrotor_Num);

J_p_diag = [0.082, 0.0845, 0.1377];
Jx_p = J_p_diag(1);
Jy_p = J_p_diag(2);
Jz_p = J_p_diag(3);

J_p = zeros(3, 3, Quadrotor_Num);
J_single_p = diag(J_p_diag);
J1_p = J_single_p;
J2_p = J_single_p;
J3_p = J_single_p;
for i = 1:Quadrotor_Num
    J_p(:, :, i) = J_single_p;
end

% Payload parameters
m_L = 2;
g = 9.81;
l_L = 1;
load_length = 1.0;
load_width = 0.8;
load_height = 0.2;

J_L_values = [0.113333, 0.173333, 0.273333];
J_L = diag(J_L_values);
Jx_L = J_L_values(1);
Jy_L = J_L_values(2);
Jz_L = J_L_values(3);

J_p_array = [Jx_p, Jy_p, Jz_p];
[a_p, b_p] = calculate_ab_params(J_p_array, l_i);
a1_p = a_p(1);
a2_p = a_p(2);
a3_p = a_p(3);
b1_p = b_p(1);
b2_p = b_p(2);
b3_p = b_p(3);

J_L_array = [Jx_L, Jy_L, Jz_L];
[a_L_tmp, b_L_tmp] = calculate_ab_params(J_L_array, l_L);
a1_L = a_L_tmp(1);
a2_L = a_L_tmp(2);
a3_L = a_L_tmp(3);
b1_L = b_L_tmp(1);
b2_L = b_L_tmp(2);
b3_L = b_L_tmp(3);

function [a_array, b_array] = calculate_ab_params(J_array, L)
    a_array = zeros(1, 3);
    b_array = zeros(1, 3);
    for idx = 1:3
        j1 = mod(idx, 3) + 1;
        j2 = mod(idx + 1, 3) + 1;
        a_array(idx) = (J_array(j1) - J_array(j2)) / J_array(idx);
        if idx == 3
            b_array(idx) = 1 / J_array(idx);
        else
            b_array(idx) = L / J_array(idx);
        end
    end
end
