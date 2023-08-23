function EDMD = get_EDMD(X1, X2, U1, n_basis)
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
    z1 = get_basis(x1,n_basis);
    z2 = get_basis(x2,n_basis);
    Z1 = [Z1, z1]; Z2 = [Z2, z2];
end

%% get A B matrices
% Z2 = AZ1 + BU = [A B]*Z1_aug => Z2 = K*Z1_aug
Z1_aug = [Z1; U1];

% EDMD using least squares 
% reference: https://www.sciencedirect.com/science/article/pii/S0167278919306086
% S Klaus (2019) Data driven approx of the Koopman Generator
n1 = size(Z1,1); n2 = size(U1,1); % size of K: n1 X n2
m = size(Z1,2); % avg over number of data points
A = (Z2*Z1_aug')./m;
G = (Z1_aug*Z1_aug')./m;
EDMD.Inter_A = A;
EDMD.Inter_G = G;

% mapping matrix X = CZ 
% X_PID: [x; dx; q; wb] => 13 X 1
C = zeros(13,size(Z1,1)); 
C(1:13,1:13)=eye(13); 

% K*G = A
EDMD.K = A*pinv(G);
EDMD.A = EDMD.K(:,1:size(Z1,1));
EDMD.B = EDMD.K(:,size(Z1,1)+1:end);
EDMD.C = C;
EDMD.Z1 = Z1;
EDMD.Z2 = Z2;

%% using lsqminnorm
% Z2 = AZ1 + BU = [A B]*Z1_aug => Z2 = K*Z1_aug => Z2'=Z1_Aug'*K' (b = A*x)
% Z1_aug = [Z1; U1];
% AA = Z1_aug';
% bb = Z2';
% K_top = lsqminnorm(AA, bb);
% K_top = lsqminnorm(G', A');
% EDMD.K_minnorm = K_top';
% K*G = A => G'*K' = A' (size K_top = n2 X n1)
% K_i_row*G = A_i_row => G'*K_i_row' = A_i_row'
K_minnorm = zeros(n1,n1+n2);
for i=1:n1
    K_i_row_top = lsqminnorm(G', A(i,:)');%G'\A(i,:)';%
    K_minnorm(i,:) = K_i_row_top';
end

EDMD.K_minnorm = round(K_minnorm,3);
EDMD.A_minnorm = round(EDMD.K_minnorm(:,1:size(Z1,1)),3);
EDMD.B_minnorm = round(EDMD.K_minnorm(:,size(Z1,1)+1:end),3);

%% using cvx
% n = size(Z1,1); m = size(U1,1);
% cvx_begin quiet
% variable K(n1,n1+n2)
% % Z2 = AZ1 + BU = [A B]*Z1_aug => Z2 = K*Z1_aug
% minimize norm(Z2-K*Z1_aug)% + 0.1*norm(K)
% cvx_end
% EDMD.K_cvx = roound(K,4);