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
% add basis for CBF constrains
% % cbf_basis => [x1^2; x1*x2; x1*x3; x2^2; x2*x3; x3^2]
% cbf_basis = unique(kron(X(1:3),X(1:3)),'stable');
% cbf_basis => [x1^2; x2^2; x3^2]
cbf_basis = X(1:3).^2;
basis = [X(1:18); cbf_basis; basis];