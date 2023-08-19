function basis = get_basis(X, n)
% Input
% X         : states
% n         : number of basis
% output
% basis     : basis functions for SO(3) dynamics

%% extract states
wRb = reshape(X(7:15),[3,3]); % wRb - body frame to inertial frame
wb = X(16:18); % wb - body frame
wb_hat = hat_map(wb);

%% build observables [R';z1,z1;...]
% zn = R*w_hat^n
basis = [];
Z = wRb;
for i = 1:n
    Z = Z*wb_hat;
    basis = [basis; vectorize(Z)];
end
% basis = [vectorize(R'); vectorize(wb_hat'); basis];
q = RotToQuat(wRb);
basis = [X(1:6); q; wb; basis];
