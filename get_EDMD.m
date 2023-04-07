function EDMD = get_EDMD(n_basis)
% function to generate A B matrices using EDMD
% Inputs
% n_basis       : Number of basis used to generate EDMD
% Outputs
% EDMD.Z        : Lifted states of n traj of length m 
% size of Z     : (15+9*n_basis) x (n x m)
% EDMD.U        : control inputs corresponding Z 
% size of U     : 4 x (n x m))
% EDMD.K        : Augmented K matrix where K = [A B]
% size of K     : (15+9*n_basis) x (n x m)
% EDMD.A        : Lifted A matrix
% size of A     : (15+9*n_basis) x n
% EDMD.B        : Lifted B matrix
% size of B     : (15+9*n_basis) x m

%% generate random inputs for U
% generate n random inputs for 4 thrusters
% use multivariate random nomral distribution since inputs are correlated
n = 100;
mu = [0;0;0;0];
Sigma = diag([100;100;100;100]);
U_rnd = mvnrnd(mu,Sigma,n);

%% simulate random inputs for 50s to get trajectories
params = get_params();

% states X = [x dx R wb]'
% initial condition is from rest (from ground)
x0 = [0;0;0]; dx0 = [0;0;0];
R0 = eye(3); wb0 = [0;0;0];
X0 = [x0;dx0;R0(:);wb0];

X_rnd = []; 
t_span = 0:0.01:5;
for i=1:n
    % get control
    U = U_rnd(i,:)';
    % make sure net force U(1) > mg
    U(1) = params.mass*params.g/4 + U(1);
    
    % simulate Ode
    [t,X] = ode45(@(t,X)dynamics_SRB(t,X,U,params),t_span,X0);

    % collect data
    X_rnd = [X_rnd,X']; % [X(t1), X(t2), ..., X(tn)] stacked for each control input n
end

%% generate basis from data
Z = []; 
% collect basis for n control inputs 
% Zn = [Zt_1; Zt_2; ... Zt_n]
for i = 1:length(X_rnd)
    X = X_rnd(:,i);
    basis = get_basis(X,n_basis);
    z = [X(1:3); X(4:6); basis];
    Z = [Z, z];
end

% stack U as constant along each trajectory for n control inputs
U = [];
for i = 1:n    
    U = [U, repmat(U_rnd(i,:)',1,length(t_span))];
end

%% get A B matrices
% Z2 = AZ1 + BU = [A B]*Z1_aug => Z2 = K*Z1_aug
Z1 = Z(:,1:end-1);
U1 = U(:,1:end-1);
Z1_aug = [Z1; U1];
Z2 = Z(:,2:end);

EDMD.K = Z2*pinv(Z1_aug);
EDMD.A = EDMD.K(:,1:size(Z1,1));
EDMD.B = EDMD.K(:,size(Z1,1)+1:end);
EDMD.Z = Z;
EDMD.U = U;

