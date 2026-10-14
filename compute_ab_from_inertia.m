function [a_p, b_p] = compute_ab_from_inertia(inertia_diag, arm_length)
% Compute rigid-body a/b coefficients from the inertia diagonal.

if nargin < 2 || isempty(arm_length)
    arm_length = 1;
end

J = reshape(inertia_diag, 3, 1);
J = max(J, 1.0e-9);

a_p = [ ...
    (J(2) - J(3)) / J(1); ...
    (J(3) - J(1)) / J(2); ...
    (J(1) - J(2)) / J(3)];

b_p = [ ...
    arm_length / J(1); ...
    arm_length / J(2); ...
    1 / J(3)];
end
