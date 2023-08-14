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

%% get parameters
mpc_params = get_params();
% set params
show_plot = false;
mpc_params.use_casadi = false;

%% generate trajectory data using nominal controller
traj_params.traj_type = 'circle';%'hover';%'circle';
% parameter: 
% hover:-> height
% circle:-> radius
% line:-> end point
traj_params.params = [1,1.1,1.2,1.3];  
traj_params.n_traj = length(traj_params.params);

[T, X, U, X1, X2, U1, U2, traj_params] = get_pid_trajectories(traj_params,show_plot);

%% get EDMD matrices
n_basis = 3; % n=3 works best
EDMD = get_EDMD(X1, X2, U1, n_basis);
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;
EDMD.n_basis = n_basis;

%% One timestep prediction
% Z2_predicted = A*Z1 + B*U1;
% X_ref=[]; X_pred=[];
% for i = 1:length(Z2_predicted)
%     X_ref = [X_ref, EDMD.C*Z2(:,i)];
%     X_pred = [X_pred, EDMD.C*Z2_predicted(:,i)];
% end
% RMSE_training = rmse(X_pred,X_ref,traj_params.traj_len,show_plot);

%% evaluate EDMD prediction for n timesteps
show_plot = true;
X_eval = eval_EDMD_pid(X,U,traj_params,EDMD,n_basis,show_plot);

%% do MPC
% MPC parameters
mpc_params.predHorizon = 2;
%params.Tmpc = 1e-3;
mpc_params.simTimeStep = 1e-2;

dt_sim = mpc_params.simTimeStep;

% simulation time
mpc_params.SimTimeDuration = 1;  % [sec]
mpc_params.MAX_ITER = floor(mpc_params.SimTimeDuration/ mpc_params.simTimeStep);

% get reference trajectory (desired)
% n_control = 1; % number of random controls to apply
% % t_traj = 0:params.Tmpc:10; % traj length to simulate (s)
% t_traj = 0:1e-3:10;
% show_plot = false;
% flag = 'mpc';
% [X_ref] = get_rnd_trajectories(X0,n_control,t_traj,show_plot,flag);

show_plot = false;
traj_params.params = 1;  
traj_params.n_traj = length(traj_params.params);
[T, X_ref, U, X1, X2, U1, U2, traj_params] = get_pid_trajectories(traj_params,show_plot);

% get lifted states
Z_ref = [];
X_ref = X_ref(:,2:end);
for i = 1:length(X_ref) % compare n+1 timesteps
    x_des = X_ref(:,i);
    basis = get_basis(x_des,n_basis);
    z = [x_des(1:3); x_des(4:6); basis];
    Z_ref = [Z_ref,z];
end

X0 = X_ref(:,1);
basis = get_basis(X0,n_basis);
Z0 = [X0(1:3); X0(4:6); basis];

mpc = sim_MPC(EDMD,Z0,Z_ref,X_ref,mpc_params);

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

%state traj plots
fig_num = 10; flag = 'mpc';
state_plots(fig_num,mpc.t,X,X_ref,flag)
fig_num = 20;

% control polts
control_plots(fig_num,mpc.t,mpc.U)

% figure(1);
% subplot(2,4,5)
% plot3(x_ref(1,:), x_ref(2,:), x_ref(3,:)); hold on
% plot3(x_mpc(1,:), x_mpc(2,:), x_mpc(3,:), '--'); hold on
% grid on; box on; axis square;
% xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
% ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
% zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
% axes = gca; set(axes,'FontSize',15);
% axes.LineWidth=2;