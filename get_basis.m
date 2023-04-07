function basis = get_basis(X, n)
% Input
% X         : states
% n         : number of basis
% output
% basis     : basis functions for SO(3) dynamics

%% extract states
R = reshape(X(7:15),[3,3]);
wb = reshape(X(16:18),[3,1]);
wb_hat = hat_map(wb);

%% build observables (add as necessary)
Z = R;
for i = 1:n
    Z = Z*wb_hat;
    basis = [basis; vectorize(Z)];
end
basis = [vectorize(R'); basis];
