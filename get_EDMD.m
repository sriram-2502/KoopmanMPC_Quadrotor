function EDMD = get_EDMD(X, U, n_basis)
% function to generate A B matrices using EDMD
% Inputs
% X             : state trajecotries of n traj of length m
% U             : random control inputs of 
% n_basis       : Number of basis used to generate EDMD
% Outputs
% EDMD.Z        : Lifted states 
% size of Z     : (15+9*n_basis) x (n x m)
% EDMD.K        : Augmented K matrix where K = [A B]
% size of K     : (15+9*n_basis) x (n x m)
% EDMD.A        : Lifted A matrix
% size of A     : (15+9*n_basis) x n
% EDMD.B        : Lifted B matrix
% size of B     : (15+9*n_basis) x m

%% generate basis from data
Z = []; 
% collect basis for n control inputs 
% Zn = [Zt_1; Zt_2; ... Zt_n]
for i = 1:length(X)
    x = X(:,i);
    basis = get_basis(x,n_basis);
    z = [x(1:3); x(4:6); basis];
    Z = [Z, z];
end

%% get A B matrices
% Z2 = AZ1 + BU = [A B]*Z1_aug => Z2 = K*Z1_aug
Z1 = Z(:,1:end-1);
U1 = U(:,1:end-1);
Z1_aug = [Z1; U1];
Z2 = Z(:,2:end);

% EDMD using least squares 
% reference: https://www.sciencedirect.com/science/article/pii/S0167278919306086
% S Klaus (2019) Data driven approx of the Koopman Generator
m = size(Z,2); % avg over number of data points
A = (Z2*Z1_aug')./m;
G = (Z1_aug*Z1_aug')./m;

EDMD.K = A*pinv(G);
EDMD.A = EDMD.K(:,1:size(Z1,1));
EDMD.B = EDMD.K(:,size(Z1,1)+1:end);
EDMD.Z = Z;

