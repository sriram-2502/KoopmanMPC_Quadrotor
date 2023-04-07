function EDMD = get_EDMD()

%% generate data
params = get_params();

% states X = [x dx R wb]'
x0 = [0;0;0]; dx0 = [0;0;0];
R0 = eye(3); wb0 = [0;0;0];
X0 = [x0,dx0,R0,wb0];

% generate n random inputs for 4 thrusters
% todo scale inputs to match geometric control paper
n = 100;
mu = [0;0;0;0];
Sigma = diag([1;1;1;1]);
U_rnd = mvnrnd(mu,Sigma,n);

% simulate random inputs for 50s to get trajectories
X_rnd = [];
t_span = 0:0.1:50;
for i=1:n
    U = U_rnd(i,:);
    [t,X] = ode45(@(t,X)dynamics_SRB(t,X0,U,params),t_span,X0);
    X_rnd = [X_rnd, X];
end

EDMD = X_rnd;