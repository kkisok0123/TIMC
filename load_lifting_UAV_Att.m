function [X_dot] = load_lifting_UAV_Att(x, M_plant)
global a1_p a2_p a3_p
global b1_p b2_p b3_p

X_dot = zeros(6,1);
% 无人机的位姿信息
X_dot(1) = x(2);
X_dot(2) = x(4) * x(6) * a1_p + M_plant(1) * b1_p;
X_dot(3) = x(4);
X_dot(4) = x(2) * x(6) * a2_p + M_plant(2) * b2_p;
X_dot(5) = x(6);
X_dot(6) = x(2) * x(4) * a3_p + M_plant(3) * b3_p;

end 