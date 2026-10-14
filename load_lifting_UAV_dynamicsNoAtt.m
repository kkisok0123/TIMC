function [X_dot] = load_lifting_UAV_dynamicsNoAtt(x, Us, M_plant)
global g  m_p1
global a1_p a2_p a3_p
global b1_p b2_p b3_p 
X_dot = zeros(12,1);
X_dot(1) = x(2);
X_dot(2) = x(4)*x(6)*a1_p + M_plant(1)*b1_p;
X_dot(3) = x(4);
X_dot(4) = x(2)*x(6)*a2_p + M_plant(2)*b2_p;
X_dot(5) = x(6);
X_dot(6) = x(2)*x(4)*a3_p + M_plant(3)*b3_p;
Att = [x(1); x(3); x(5)];
R = eul2rotm_ZYX(Att);
% Us = F_plant * R(:,3);

X_dot(7)  = x(8); 
X_dot(8)  = Us(1)/m_p1;
X_dot(9)  = x(10); 
X_dot(10) = Us(2)/m_p1;
X_dot(11) = x(12); 
% 向上为正，重力向下，所以是-g
X_dot(12) = Us(3)/m_p1 - g;
end 