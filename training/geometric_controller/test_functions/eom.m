function Xdot = eom(t, X, k, param, trajhandle, controlhandle)

e3 = [0, 0, 1]';
m = param.m;
J = param.J;

[~, v, R, W, ~, ~] = split_to_states(X);
param.R = R;
desired = trajhandle(t,param);
[f, M, ei_dot, eI_dot, ~, ~] = controlhandle(X, desired, k, param);

xdot = v;
vdot = param.g * e3 ...
    - f / m * R * e3 + param.x_delta / m;
Wdot = J \ (-hat(W) * J * W + M + param.R_delta);
Rdot = R * hat(W);
% x_delta and R_delta: disturnbance inputs

Xdot=[xdot; vdot; Wdot; reshape(Rdot,9,1); ei_dot; eI_dot];
end