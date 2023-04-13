function EDMD = get_EDMD(X1, X2, U1, n_basis, t_traj)
% function to generate A B matrices using EDMD
% Inputs
% X1            : state trajecotries of n traj of length 1:m-1
% X2            : state trajecotries of n traj of length 2:m
% U1            : random control inputs for n traj of length 1:m-1 
% n_basis       : Number of basis used to generate EDMD
% t_traj        : vector of timestamps for each trajectory
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
Z1 = []; Z2 = [];
% collect basis for n control inputs 
% Zn = [Zt_1; Zt_2; ... Zt_n]
for i = 1:length(X1)
    x1 = X1(:,i); x2 = X2(:,i);
    basis1 = get_basis(x1,n_basis);
    basis2 = get_basis(x2,n_basis);
    z1 = [x1(1:3); x1(4:6); basis1];
    z2 = [x2(1:3); x2(4:6); basis2];
    Z1 = [Z1, z1]; Z2 = [Z2, z2];
end

%% get A B matrices
% Z2 = AZ1 + BU = [A B]*Z1_aug => Z2 = K*Z1_aug
Z1_aug = [Z1; U1];

% EDMD using least squares 
% reference: https://www.sciencedirect.com/science/article/pii/S0167278919306086
% S Klaus (2019) Data driven approx of the Koopman Generator
m = size(Z1,2); % avg over number of data points
A = (Z2*Z1_aug')./m;
G = (Z1_aug*Z1_aug')./m;

% mapping matrix X = CZ 
C = zeros(24,size(Z1,1)); 
C(1:24,1:24)=eye(24); 

EDMD.K = A*pinv(G);
EDMD.A = EDMD.K(:,1:size(Z1,1));
EDMD.B = EDMD.K(:,size(Z1,1)+1:end);
EDMD.C = C;
EDMD.Z1 = Z1;
EDMD.Z2 = Z2;

