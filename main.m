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
traj_params.traj_type = 'line';%'hover';%'line';
% parameter: 
% hover:-> height
% circle:-> radius
% line:-> end point
traj_params.params = linspace(0.5,5,10);  
traj_params.n_traj = length(traj_params.params);
flag='training';
[T, X, U, X1, X2, U1, U2, traj_params] = get_pid_trajectories(traj_params,show_plot,flag);

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
flag='eval';
traj_params.params = 1.5;  
traj_params.n_traj = length(traj_params.params);
[T, X, U, X1, X2, U1, U2, traj_params] = get_pid_trajectories(traj_params,show_plot,flag);
X_eval = eval_EDMD_pid(X,U,traj_params,EDMD,n_basis,show_plot);

%% do MPC
% MPC parameters
mpc_params.predHorizon = 10;
mpc_params.simTimeStep = 1e-3;
dt_sim = mpc_params.simTimeStep;

% simulation time
mpc_params.SimTimeDuration = 2;  % [sec]
mpc_params.MAX_ITER = floor(mpc_params.SimTimeDuration/ mpc_params.simTimeStep);

% get reference trajectory
show_plot = false;
traj_params.params = 1.5;  
traj_params.n_traj = length(traj_params.params);
flag='mpc';
[T, X_ref_mpc, U, X1, X2, U1, U2, traj_params] = get_pid_trajectories(traj_params,show_plot,flag);

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

mpc = sim_MPC(EDMD,Z0,Z_ref,X_ref_mpc,n_basis,mpc_params);

%% plots
% parse each state for plotting
x_ref=[]; dx_ref = []; theta_ref =[]; wb_ref=[];
x_mpc=[]; dx_mpc = []; theta_mpc =[]; wb_mpc=[];
X_ref_plot = []; X_mpc = []; t_plot = [];
skip_idx=10; 
for i = 1:skip_idx:length(mpc.t)
    x_ref = [x_ref, mpc.X_ref(i,1:3)'];
    dx_ref = [dx_ref, mpc.X_ref(i,4:6)'];
    q_ref = mpc.X_ref(i,7:10)';
    bRw_ref = QuatToRot(q_ref);
    [roll_ref,pitch_ref,yaw_ref] = RotToRPY_ZXY(bRw_ref);
    theta_ref = [theta_ref, [roll_ref,pitch_ref,yaw_ref]'];
    wb_ref = [wb_ref, mpc.X_ref(i,11:13)']; 

    % for prediction
    x_mpc = [x_mpc, mpc.X(i,1:3)'];
    dx_mpc = [dx_mpc, mpc.X(i,4:6)'];
    q_mpc = mpc.X(i,7:10)';
    bRw_mpc = QuatToRot(q_mpc);
    [roll_mpc,pitch_mpc,yaw_mpc] = RotToRPY_ZXY(bRw_mpc);
    theta_mpc = [theta_mpc, [roll_mpc,pitch_mpc,yaw_mpc]'];
    wb_mpc = [wb_mpc, mpc.X(i,11:13)']; 

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