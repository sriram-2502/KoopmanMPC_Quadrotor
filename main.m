clc; clear; close all;
set(0,'DefaultLineLineWidth',2) %linewidh on plots
set(0,'defaultfigurecolor',[1 1 1])
seed = 1;
rng(seed);
show_plot = true;

%% get robot parameters
params = get_params();
dt = 1e-3;
t_span = 0.1; % in (s)

%% generate random data
% generate random data starting from same initial condition
% each traj is generated using contant control input
% generate random control inputs to get different trajectories
x0 = [0;0;0]; dx0 = [0;0;0];
R0 = eye(3); wb0 = [0;0;0];
X0 = [x0;dx0;R0(:);wb0];

n_control = 100; % number of random controls to apply
t_traj = 0:dt:t_span; % traj length to simulate (s)
[X, U, X1, X2, U1, U2] = get_trajectories(X0,n_control,t_traj,show_plot);

%% get EDMD matrices
n_basis = 1;
EDMD = get_EDMD(X1, X2, U1, n_basis, t_traj);
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;

%% check prediction on the training distrubution as || Z2 - (AZ1 + BU1) ||_mse
Z2_predicted = A*Z1 + B*U1;
Z2_error = Z2(:) - Z2_predicted(:);
Z2_mse_training = sqrt(mean(Z2_error.^2))/sqrt(mean(Z2(:).^2))

%% evaluate EDMD prediction
eval_EDMD(X0,dt,t_span,EDMD,n_basis,show_plot)

%% get QP matrices

