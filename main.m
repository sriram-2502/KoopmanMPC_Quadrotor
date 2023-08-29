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


%% generate trajectory data using nominal controller
traj_params.traj_type = 'line';%'circle';%'lissajous';
show_plot = false;
noise_flag = false;
trun_traj = false; % if true, need to set the indices inside get_geometric_trajectories
% parameter: 
% hover:-> height
% circle:-> radius
% line:-> end point
traj_params.params = linspace(0.5,2.5,10);  
traj_params.n_traj = length(traj_params.params);
[T, X, U, X1, X2, U1, U2, traj_params] = get_geometric_trajectories(traj_params,show_plot,noise_flag,trun_traj);

%% get EDMD matrices
n_basis = 3; % n=3 works best
EDMD = get_EDMD(X1, X2, U1, n_basis);
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;
EDMD.n_basis = n_basis;

%% evaluate EDMD prediction for n timesteps
show_plot = true;
noise_flag = false;
trun_traj = false;
traj_params.params = 1.5;  
traj_params.n_traj = length(traj_params.params);
[T, X, U, X1, X2, U1, U2, traj_params] = get_geometric_trajectories(traj_params,show_plot,noise_flag,trun_traj);
EDMD.eval_horizon = 2000; % 'eval_horizon' step prediction
X_eval = eval_EDMD_pid(X,U,traj_params,EDMD,show_plot);

%% do MPC
% MPC parameters
noise_flag = false;
params = get_params(noise_flag);
% set params
params.use_casadi = false;
params.predHorizon = 15;
params.simTimeStep = 1e-3;
dt_sim = params.simTimeStep;

% simulation time
params.SimTimeDuration = 3;  % [sec]
params.MAX_ITER = floor(params.SimTimeDuration/ params.simTimeStep);

% get reference trajectory
show_plot = false;
trun_traj = false;
traj_params.params = 1.5;  
traj_params.n_traj = length(traj_params.params);
[T, X_ref_mpc, U, X1, X2, U1, U2, traj_params] = get_geometric_trajectories(traj_params,show_plot,noise_flag,trun_traj);

% reduce number of refernce points in the trajectory
% hold_for = 5;%mpc_params.predHorizon;
% remove_num = mod(size(X_ref,2),hold_for);
% X_ref_hold = zeros(size(X_ref,1),size(X_ref,2)-remove_num);
% for i=hold_for:hold_for:size(X_ref,2)
%     X_ref_hold(:,i-(hold_for-1):i) = repmat(X_ref(:,i),1,hold_for);
% end

% get lifted states
Z_ref = [];
X0 = X_ref_mpc(:,1);
X_ref_mpc = X_ref_mpc(:,2:end);
% X_ref = X_ref_hold(:,2:end);
for i = 1:length(X_ref_mpc) % compare n+1 timesteps
    x_des = X_ref_mpc(:,i);
    z = get_basis(x_des,n_basis);
    Z_ref = [Z_ref,z];
end
Z0 = get_basis(X0,n_basis);
mpc = sim_MPC(EDMD,Z0,Z_ref,X_ref_mpc,params);
mpc
%% MPC plots
% parse each state for plotting
close all
x_ref=[]; dx_ref = []; theta_ref =[]; wb_ref=[];
x_mpc=[]; dx_mpc = []; theta_mpc =[]; wb_mpc=[];
X_ref_plot = []; X_mpc = []; t_plot = [];
skip_idx=100; 
for i = 1:skip_idx:length(mpc.t)
    x_ref = [x_ref, mpc.X_ref(i,1:3)'];
    dx_ref = [dx_ref, mpc.X_ref(i,4:6)'];
    bRw_ref = reshape(mpc.X_ref(i,10:18),[3,3]);
    theta_ref = [theta_ref, vee_map(logm(bRw_ref))];
    wb_ref = [wb_ref, mpc.X_ref(i,7:9)'];

    % for prediction
    x_mpc = [x_mpc, mpc.X(i,1:3)'];
    dx_mpc = [dx_mpc, mpc.X(i,4:6)'];
    bRw = reshape(mpc.X(i,10:18),[3,3]);
    theta_mpc = [theta_mpc, vee_map(logm(bRw))];
    wb_mpc = [wb_mpc, mpc.X(i,7:9)'];

    t_plot = [t_plot, mpc.t(i)];
end

X_ref_plot.x = x_ref; X_ref_plot.dx = dx_ref;
X_ref_plot.theta = theta_ref; X_ref_plot.wb = wb_ref;

X_mpc.x = x_mpc; X_mpc.dx = dx_mpc;
X_mpc.theta = theta_mpc; X_mpc.wb = wb_mpc;

%state traj plots
fig_num = 10; flag = 'mpc';
state_plots(fig_num,t_plot,X_mpc,X_ref_plot,flag)
fig_num = 20;

% control polts
control_plots(fig_num,mpc.t,mpc.U)

figure(1);
plot3(x_ref(1,:), x_ref(2,:), x_ref(3,:)); hold on
plot3(x_mpc(1,:), x_mpc(2,:), x_mpc(3,:), '--'); hold on
grid on; box on; axis square;
xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
axes.LineWidth=2;