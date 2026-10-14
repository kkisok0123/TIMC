function R = eul2rotm_ZYX(angles)
%% 欧拉角转旋转矩阵（ZYX顺序）
% 输入参数：
%   angles: [roll; pitch; yaw]，单位：弧度
% 输出参数：
%   R: 3x3旋转矩阵
%
% 说明：
%   按照 ZYX 顺序（先绕Z轴旋转yaw，再绕Y轴旋转pitch，最后绕X轴旋转roll）
%   进行右乘得到旋转矩阵：R = Rz(yaw) * Ry(pitch) * Rx(roll)

roll  = angles(1);
pitch = angles(2);
yaw   = angles(3);

% 基本旋转矩阵
Rz = [ cos(yaw), -sin(yaw), 0;
       sin(yaw),  cos(yaw), 0;
       0,         0,        1 ];

Ry = [ cos(pitch),  0, sin(pitch);
       0,           1, 0;
      -sin(pitch),  0, cos(pitch) ];

Rx = [ 1, 0,         0;
       0, cos(roll), -sin(roll);
       0, sin(roll),  cos(roll) ];

% 按 ZYX 顺序做右乘
% R = Rz(yaw)*Ry(pitch)*Rx(roll)
R = Rz * Ry * Rx;

end



