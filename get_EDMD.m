function EDMD = get_EDMD()
% function to generate A B matrices using EDMD
%% generate random inputs for U
% generate n random inputs for 4 thrusters
% use multivariate random nomral distribution since inputs are correlated
n = 10;
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
t_span = 0:0.01:2;
for i=1:n
    % get control
    U = U_rnd(i,:)';
    % make sure net force U(1) > mg
    U(1) = params.mass*params.g/4 + U(1);
    
    % simulate Ode
    [t,X] = ode45(@(t,X)dynamics_SRB(t,X,U,params),t_span,X0);

    % collect data
    X_rnd = [X_rnd;X']; % [X(t1), X(t2), ..., X(tn)] stacked for each control input n
end

%% generate basis from data
Zt = [];Zn = []; U = [];

% collect basis for n control inputs 
% Zn = [Zt_1; Zt_2; ... Zt_n]
for i = 1:n
    Zt = []; 
    % collect basis along each traj
    % Zt = [z(t1), z(t2), ... z(tn)]
    for j = 1:length(t_span)
        X = X_rnd(i:i*length(X0),j);
        basis = get_basis(X);
        z = [X(1:3); X(4:6); basis];
        Zt = [Zt, z];
    end
    Zn = [Zn; Zt];
    % stack U as constant along each trajectory for n control inputs
    U = [U; repmat(U_rnd(1,:)',1,length(t_span))];
end

%% get A B matrices
% Z2 = AZ1 + BU = [A B]*Z1_aug => Z2 = K*Z1_aug
Z1 = Zn(:,1:end-1);
U1 = U(:,1:end-1);
Z1_aug = [Z1; U1];
Z2 = Zn(:,2:end);

EDMD.K = Z2*pinv(Z1_aug);
EDMD.A = EDMD.K(:,1:size(Z1,1));
EDMD.B = EDMD.K(:,size(Z1,1)+1:end);
EDMD.Z = Zn;
EDMD.U = U;

