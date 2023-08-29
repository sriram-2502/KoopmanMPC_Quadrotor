function basis = get_basis(X, n)
% Input
% X         : states
% n         : number of basis
% output
% basis     : basis functions for SO(3) dynamics

%% extract states
wRb = reshape(X(10:18),[3,3]); % wRb - body frame to inertial frame
wb = X(7:9); % wb - body frame
wb_hat = hat_map(wb);

%% build observables [R';z1,z1;...]
% zn = R*w_hat^n
basis = [];
Z = wRb;
for i = 1:n
    Z = Z*wb_hat;
    basis = [basis; vectorize(Z)];
end

% Geometric basis
% X(1:18) => [x; dx; w; R(:)]
% removing last six states, ei, eI, so that they dont affect the dynamics
basis = [X(1:18); basis];