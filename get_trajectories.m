function [X, U] = get_trajectories(X0, n_control,t_traj, show_plot)
% function to random trajectories
% Inputs
% X0                : initial condition
% n_control         : number of random control inputs
% t_traj            : trajectory length for each control input 
% outputs
% X                 : random trajectories generated with constant control for
% each trajectory
% size of X         : 18 * (n_control x len(t_traj))
% U                 : random control inputs for each trajectory
% size of U         : 4 * (n_control x len(t_traj))

%% get robot params
params = get_params();

%% generate random inputs for U
% generate n random inputs for 4 thrusters
net_weight = params.mass*params.g;

% use multivariate random nomral distribution since inputs are correlated
mu = [10;10;10;10]; % ensures flight
Sigma = diag([1;1;1;1]);
U_rnd = mvnrnd(mu,Sigma,n_control);

% get U as uniform random distribution
% a = 10; b = 20;
% U_rnd = a + (b-a).*rand(n_control,4);

% add the net weight of the robot to ensure it flies up 
U_rnd(:,1) = U_rnd(:,1) + net_weight;

% straight line traj
U_rnd(:,2:end) = 0;

%% simulate random inputs for 50s to get trajectories
% states X = [x dx R wb]'
% initial condition is from rest (from ground)
X = []; 
t_span = t_traj;

for i=1:n_control
    % get control
    U = U_rnd(i,:)';

    % simulate ode
    [t,x] = ode45(@(t,X)dynamics_SRB(t,X,U,params),t_span,X0);
    
    if(show_plot)
        figure(1)
        plot3(x(:,1),x(:,2),x(:,3)); hold on;
        grid on; axis square
        title('training trajectories for EDMD')
    end

    % collect data
    X = [X,x']; % [X(t1), X(t2), ..., X(tn)] stacked for each control input n
end

% stack U as constant along each trajectory for n control inputs
U = [];
for i = 1:n_control    
    U = [U, repmat(U_rnd(i,:)',1,length(t_span))];
end