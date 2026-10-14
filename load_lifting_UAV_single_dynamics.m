function [X_dot] = load_lifting_UAV_single_dynamics(x,U)

global m_p1 g
global a1_p a2_p a3_p
global b1_p b2_p b3_p

X_dot    = zeros(12,1);

Ux = cos(x(1))*sin(x(3))*cos(x(5))+sin(x(1))*sin(x(5));
Uy = cos(x(1))*sin(x(3))*sin(x(5))-sin(x(1))*cos(x(5));

% 状态顺序: [phi; phi_dot; theta; theta_dot; psi; psi_dot; x; x_dot; y; y_dot; z; z_dot]
% 姿态动力学
X_dot(1) = x(2);
X_dot(2) = x(4)*x(6)*a1_p+ U(2)*b1_p;
X_dot(3) = x(4);
X_dot(4) = x(2)*x(6)*a2_p+ U(3)*b2_p;
X_dot(5) = x(6);
X_dot(6) = x(2)*x(4)*a3_p+ U(4)*b3_p;
% 位置动力学 (x, y, z)
X_dot(7) = x(8);   % x_dot
X_dot(8)= U(1)/m_p1*Ux;  % x_ddot
X_dot(9)= x(10);   % y_dot
X_dot(10)= U(1)/m_p1*Uy;  % y_ddot
X_dot(11) = x(12);  % z_dot
X_dot(12) = U(1)/m_p1*(cos(x(1))*cos(x(3)))-g;  % z_ddot
end

