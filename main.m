clc; clear; close all;

%% get robot parameters
params = get_params();

%% generate random data
n_control = 10; % number of random controls to apply
t_traj = 0:0.01:5; % traj length to simulate (s)
[X, U] = get_trajectories(n_control,t_traj);

%% get EDMD matrices
n_basis = 1;
EDMD = get_EDMD(X, U, n_basis);
A = EDMD.A;
B = EDMD.B;
Z = EDMD.Z;
U = EDMD.U;

%% check prediction on the same distrubution as || Z2 - (AZ1 + BU1) ||_mse
Z1 = Z(:,1:end-1);
U1 = U(:,1:end-1);
Z2 = Z(:,2:end);
Z2_predicted = A*Z1 + B*U1;
Z2_error = Z2 - Z2_predicted;
Z2_mse = sqrt(mean(Z2_error(:).^2))