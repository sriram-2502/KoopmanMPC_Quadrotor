clc; clear; close all;
% seed = 10;
% rng(seed);

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
show_plot = true;
[X, U, X1, X2, U1, U2] = get_trajectories(X0,n_control,t_traj,show_plot);

%% get EDMD matrices
n_basis = 1;
EDMD = get_EDMD(X1, X2, U1, n_basis, t_traj);
A = EDMD.A;
B = EDMD.B;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;

%% check prediction on the training distrubution as || Z2 - (AZ1 + BU1) ||_mse
Z2_predicted = A*Z1 + B*U1;
Z2_error = Z2 - Z2_predicted;
Z2_mse_training = sqrt(mean(Z2_error(:).^2))/sqrt(mean(Z2(:).^2))

%% check prediction on new control input
n_control = 1; % number of random controls to apply
t_traj = 0:dt:t_span; % traj length to simulate (s)
[X, U, X1, X2] = get_trajectories(X0,n_control,t_traj,show_plot);

% get z0
basis = get_basis(X0,n_basis);
z0 = [X0(1:3); X0(4:6); basis];

% propagate z0 for prediction
n_prediction = 100; % 10 timesteps
z = z0; Z_pred = [z0]; % add initial
for i = 1:n_prediction
    z_next = A*z + B*U(:,i);
    z = z_next;
    Z_pred = [Z_pred,z];
end

% get Z2 for comparion
Z_true = [];
for i = 1:n_prediction+1 % compare n+1 timesteps
    x = X(:,i);
    basis = get_basis(x,n_basis);
    z = [x(1:3); x(4:6); basis];
    Z_true = [Z_true,z];
end

Z2_error = Z_true - Z_pred;
Z2_mse_prediction = sqrt(mean(Z2_error(:).^2))/sqrt(mean(Z_true(:).^2))

%% plots
figure(2)
sz = 50;
subplot(2,2,1)
scatter3(Z_true(1,1), Z_true(2,1), Z_true(3,1),sz,'kx'); hold on
plot3(Z_true(1,:), Z_true(2,:), Z_true(3,:),'b'); hold on
plot3(Z_pred(1,:), Z_pred(2,:), Z_pred(3,:), 'r--'); hold on
grid on; axis square;
legend('initial','true','predicted')

subplot(2,2,2)
scatter3(Z_true(1,1), Z_true(2,1), Z_true(3,1),sz,'kx'); hold on
plot3(Z_true(1,:), Z_true(2,:), Z_true(3,:),'b'); hold on
grid on; axis square;
legend('initial','true')

subplot(2,2,4)
scatter3(Z_pred(1,1), Z_pred(2,1), Z_pred(3,1),sz,'kx'); hold on
plot3(Z_pred(1,:), Z_pred(2,:), Z_pred(3,:), 'r--'); hold on
grid on; axis square
legend('initial','predicted')
