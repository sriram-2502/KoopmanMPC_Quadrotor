function Xdot = eom_mpc(t, X, U, param, mass_change_flag)
e3 = [0, 0, 1]';
m = param.m;
J = param.J;

if mass_change_flag
    % add weight at t = 0
    if t>=0
        m = 1.5*m;
        J = 1.5*J;
    end
    % remove weight at t = 1
    if t>2
        m = param.m;
        J = param.J;
    end
end
        

% set to ei; eI = zeros(3,1) because it does not affect the
% dynamics
ei=zeros(3,1); eI=zeros(3,1);
X = [X;ei;eI];
[~, v, R, W, ~, ~] = split_to_states(X);
f = U(1);
M = U(2:4);

xdot = v;
vdot = param.g * e3 ...
    - f / m * R * e3 + param.x_delta / m;
Wdot = J \ (-hat(W) * J * W + M + param.R_delta);
Rdot = R * hat(W);
% x_delta and R_delta: disturnbance inputs

Xdot=[xdot; vdot; Wdot; reshape(Rdot,9,1)];
end