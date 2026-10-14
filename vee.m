function v = vee(S)
    % S 是 3x3 的斜对称矩阵
    % 返回 v, 满足 S = skew(v)
    % 即 S = v^wedge, 而 v = S^vee
    v = [S(3,2); 
         S(1,3);
         S(2,1)];
end