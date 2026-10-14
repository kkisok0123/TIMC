function [Fd, Md, e_RL, e_OmeL, e_PosL, e_VelL, RL] = load_lifting_payload_control_backstepping(x_L, DesAtt, DesPos, RL_d, controller_config)
% Shared nominal backstepping controller. z is UP, F world N, M body N m.
% State: [roll,p,pitch,q,yaw,r,x,vx,y,vy,z,vz]; p,q,r are BODY rates.
% DesAtt = [Euler angles, Euler angle rates, Euler angle accelerations].
model = get_required_config_section(controller_config, {'payload_model'}, 'Payload nominal model');
bs = get_required_config_section(controller_config, {'payload_control','backstepping'}, 'Payload backstepping');
J = model.inertia;
eta = x_L([1,3,5]); omega = x_L([2,4,6]);
[T, T_dot, eta_dot] = zyx_euler_kinematics(eta, omega);
RL = eul2rotm_ZYX(eta);
e_RL = 0.5 * vee(RL_d' * RL - RL' * RL_d);
Td = zyx_euler_kinematics(DesAtt(:,1), zeros(3,1));
omega_d = Td \ DesAtt(:,2);
e_OmeL = omega - RL' * RL_d * omega_d;
e_eta = atan2(sin(eta - DesAtt(:,1)), cos(eta - DesAtt(:,1)));
e_eta_dot = eta_dot - DesAtt(:,2);
z_R = e_eta_dot + bs.c1 * e_eta;
eta_ddot_cmd = DesAtt(:,3) - bs.c1 * e_eta_dot - e_eta - bs.c2 * z_R;
% eta_ddot = T_dot*omega + T*J^(-1)*(M - omega x J*omega).
Md = J * (T \ (eta_ddot_cmd - T_dot * omega)) + cross(omega, J * omega);
e_PosL = x_L([7,9,11]) - DesPos(:,1);
e_VelL = x_L([8,10,12]) - DesPos(:,2);
z_p = e_VelL + bs.c3 * e_PosL;
% Existing unit K0 retained; no new gain tuning or controller redesign.
Fd = model.mass * ([0;0;model.gravity] + DesPos(:,3) ...
    - bs.c3 * e_VelL - e_PosL - bs.c4 * z_p);
end
