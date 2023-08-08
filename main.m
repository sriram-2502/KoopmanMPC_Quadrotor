clc; clear; close all;

%set default figure properties
set(0,'DefaultLineLineWidth',2)
set(0,'defaultfigurecolor',[1 1 1])

% fix random seed
seed = 1;
rng(seed);

% import functions
addpath dynamics edmd mpc utils training
addpath(genpath('training'))

%% get robot parameters
params = get_params();
% set params
show_plot = true;
params.use_casadi = false;

dt = 1e-3;
t_span = 0.1; % in (s)

%% generate random data
% generate random data starting from same initial condition
% each traj is generated using contant control input
% generate random control inputs to get different trajectories
x0 = [0;0;0]; dx0 = [0;0;0];
R0 = eye(3); wb0 = [0.1;0;0];
X0 = [x0;dx0;R0(:);wb0];

% % get traj for hover
% traj_param.traj_type = 'hover';
% traj_param.height = [1,2,3,4]; traj_param.n_traj = length(traj_param.height); 
% [X, U, X1, X2, U1, U2, traj_len] = get_pid_trajectories(traj_param);

% get traj for circle
traj_param.traj_type = 'circle';
traj_param.radius = [1, 2]; traj_param.n_traj = length(traj_param.radius);
traj_param.direction = 1;
[X, U, X1, X2, U1, U2, traj_len] = get_pid_trajectories(traj_param);

% get traj for random control input
% n_control = 100; % number of random controls to apply
% t_traj = 0:dt:t_span; % traj length to simulate (s)
% show_plot = true; 
% flag = 'train';
% [X, U, X1, X2, U1] = get_rnd_trajectories(X0,n_control,t_traj,show_plot,flag);

%% get EDMD matrices
n_basis = 3; % n=3 works best
EDMD = get_EDMD(X1, X2, U1, n_basis);
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;
EDMD.n_basis = n_basis;

%% check prediction on the training distrubution as || Z2 - (AZ1 + BU1) ||_mse
Z2_predicted = A*Z1 + B*U1;
X_ref=[]; X_pred=[];
for i = 1:length(Z2_predicted)
    X_ref = [X_ref, EDMD.C*Z2(:,i)];
    X_pred = [X_pred, EDMD.C*Z2_predicted(:,i)];
end
show_plot = true;
RMSE_training = rmse(X_pred,X_ref,traj_len,show_plot)


%% evaluate EDMD prediction
show_plot = false;
X_eval = eval_EDMD(X0,dt,t_span,EDMD,n_basis,show_plot);

%% do MPC
% MPC parameters
params.predHorizon = 10;
%params.Tmpc = 1e-3;
params.simTimeStep = 1e-3;

dt_sim = params.simTimeStep;

% simulation time
params.SimTimeDuration = 1.2;  % [sec]
params.MAX_ITER = floor(params.SimTimeDuration/ params.simTimeStep);

% get reference trajectory (desired)
n_control = 1; % number of random controls to apply
% t_traj = 0:params.Tmpc:10; % traj length to simulate (s)
t_traj = 0:1e-3:10;
show_plot = false;
flag = 'mpc';
[X_ref] = get_rnd_trajectories(X0,n_control,t_traj,show_plot,flag);

% get lifted states
Z_ref = [];
X_ref = X_ref(:,2:end);
for i = 1:length(X_ref) % compare n+1 timesteps
    x_des = X_ref(:,i);
    basis = get_basis(x_des,n_basis);
    z = [x_des(1:3); x_des(4:6); basis];
    Z_ref = [Z_ref,z];
end

% get lift desired states for constant reference
% Z_ref = []; Xf = []; 
% xf = [0.38;-1.2;3.6]; dxf = [0;0;0];
% Rf = eye(3); wbf = [0;0;0];
% Xff = [xf;dxf;Rf(:);wbf];
% for i = 1:MAX_ITER+N % compare n+1 timesteps
%     x_des = Xff;
%     Xf = [Xf,x_des];
%     basis = get_basis(x_des,n_basis);
%     z = [x_des(1:3); x_des(4:6); basis];
%     Z_ref = [Z_ref,z];
% end
% X_ref = Xf;

basis = get_basis(X0,n_basis);
Z0 = [X0(1:3); X0(4:6); basis];

mpc = sim_MPC(EDMD,Z0,Z_ref,X_ref,params);

%% plots
% parse each state for plotting
x_ref=[]; dx_ref = []; theta_ref =[]; wb_ref=[];
x_mpc=[]; dx_mpc = []; theta_mpc =[]; wb_mpc=[];
X_ref = []; X = [];
for i = 1:length(mpc.t)
    x_ref = [x_ref, mpc.X_ref(i,1:3)']; 
    dx_ref = [dx_ref, mpc.X_ref(i,4:6)']; 
    R_ref = reshape(mpc.X_ref(i,7:15),[3,3]);
    theta_ref = [theta_ref, vee_map(logm(R_ref))]; 
    wb_ref = [wb_ref, mpc.X_ref(i,16:18)']; 

    x_mpc = [x_mpc, mpc.X(i,1:3)']; 
    dx_mpc = [dx_mpc, mpc.X(i,4:6)']; 
    R_mpc = reshape(mpc.X(i,7:15),[3,3]); %todo make R positive def in EDMD
    theta_mpc = [theta_mpc, vee_map(logm(R_mpc))]; 
    wb_mpc = [wb_mpc, mpc.X(i,16:18)']; 
end

X_ref.x = x_ref; X_ref.dx = dx_ref;
X_ref.theta = theta_ref; X_ref.wb = wb_ref;

X.x = x_mpc; X.dx = dx_mpc;
X.theta = theta_mpc; X.wb = wb_mpc;

figure(1);
subplot(2,4,5)
plot3(x_ref(1,:), x_ref(2,:), x_ref(3,:)); hold on
plot3(x_mpc(1,:), x_mpc(2,:), x_mpc(3,:), '--'); hold on
grid on; box on; axis square;
xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
axes.LineWidth=2;
lgd = legend('reference','MPC');
lgd.Location = 'northoutside';
lgd.NumColumns = 1;

%%
%state traj plots
fig_num = 10; flag = 'mpc';
state_plots(fig_num,mpc.t,X,X_ref,flag)
fig_num = 20;

% control polts
control_plots(fig_num,mpc.t,mpc.U)