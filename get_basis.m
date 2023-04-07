function basis = get_basis(X)
% Input
% X - states
% output
% basis - basis functions for SRB dynamics

%% extract states
R = reshape(X(7:15),[3,3]);
wb = reshape(Xt(16:18),[3,1]);
wb_hat = hat_map(wb);

%% build observables (add as necessary)
z1 = R*wb_hat;
z2 = z1*wb_hat;
z3 = z2*wb_hat;

basis = [vectorize(z1), vectorize(z2), vectorize(z3)];
