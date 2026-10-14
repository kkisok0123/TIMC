function S = skew(v)
    % v 为 3x1 向量, S 则为 3x3 反对称矩阵,满足 S*w = v x w
    S = [   0,  -v(3),  v(2);
         v(3),     0,   -v(1);
        -v(2),   v(1),    0   ];
end