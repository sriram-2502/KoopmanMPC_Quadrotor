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
show_plot = false;
[X, U, X1, X2, U1] = get_trajectories(X0,n_control,t_traj,show_plot);

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

%% do MPC
% MPC parameters
% params.predHorizon = 15;
% params.Tmpc = 1/50;
% params.simTimeStep = 1/200;

params.predHorizon = 5;
params.Tmpc = 1/50;
params.simTimeStep = 1/100;

dt_sim = params.simTimeStep;
N = params.predHorizon;

% simulation time
SimTimeDuration = 1;  % [sec]
MAX_ITER = floor(SimTimeDuration/dt_sim);

% get reference trajectory (desired)
n_control = 1; % number of random controls to apply
t_traj = 0:dt_sim:N; % traj length to simulate (s)
show_plot = false;
[X_ref] = get_trajectories(X0,n_control,t_traj,show_plot);

% get lifted states
Z_ref = []; Xf = [];
xf = [0.1;0.1;0.1]; dx0 = [0.1;0.1;0.1];
R0 = eye(3); wb0 = [0.1;0.1;0.1];
Xff = [xf;dx0;R0(:);wb0];
for i = 1:length(X_ref) % compare n+1 timesteps
    x_des = X_ref(:,i);
    basis = get_basis(x_des,n_basis);
    z = [x_des(1:3); x_des(4:6); basis];
    Z_ref = [Z_ref,z];
end

% get lift desired states for constant reference
% Z_ref = []; Xf = [];
% xf = [0.1;0.1;0.1]; dx0 = [0.1;0.1;0.1];
% R0 = eye(3); wb0 = [0.1;0.1;0.1];
% Xff = [xf;dx0;R0(:);wb0];
% for i = 1:length(X_ref) % compare n+1 timesteps
%     x_des = Xff;
%     Xf = [Xf,x_des];
%     basis = get_basis(x_des,n_basis);
%     z = [x_des(1:3); x_des(4:6); basis];
%     Z_ref = [Z_ref,z];
% end

% get lifted states at t=0
basis = get_basis(X0,n_basis);
Z0 = [X0(1:3); X0(4:6); basis];

% --- logging ---
tstart = 0;
tend = dt_sim;

[tout,Xout,Uout,Xdout] = deal([]);

% --- simulation ----
%h_waitbar = waitbar(0,'Calculating...');
tic
Z = Z0;
for ii = 1:MAX_ITER
    % --- time vector ---
    %t_ = dt_sim * (ii-1) + params.Tmpc * (0:N-1);

    %% --- MPC ----      
    %form QP using explicit matrices
    z_ref = Z_ref(:,ii:ii+N-1);
    [f, G, A, b] = get_QP(EDMD,Z,z_ref,N,params);
    % solve QP using quadprog     
    [zval] = quadprog(G,f,[],[],[],[],[],[]);
    Ut = zval(1:4)
    %Ut(1) = Ut(1) + params.mass*params.g; 

    %% --- simulate without any external disturbances ---
    %parse true states from lifted states
    Xt = C*Z;
    x = Xt(1:3); dx = Xt(4:6); 
    R = reshape(Xt(7:15),[3,3]);
    wb_hat = reshape(Xt(16:24),[3,3]); % body frame
    wb = vee_map(wb_hat);
    Xt = [x;dx;R(:);wb;];

    [t,X] = ode45(@(t,X)dynamics_SRB(t,X,Ut,params),[tstart,tend],Xt);
    
    %% --- update ---
    Xt = X(end,:)';
    % get lifted states at t=0
    basis = get_basis(Xt,n_basis);
    Z = [Xt(1:3); Xt(4:6); basis];

    tstart = tend;
    tend = tstart + dt_sim;
    
    %% --- log ---  
    lent = length(t(2:end));
    tout = [tout;t(2:end)];
    Xout = [Xout;X(2:end,:)];
    Uout = [Uout;repmat(Ut',[lent,1])];
    Xdout = [Xdout;repmat(X_ref(:,ii)',[lent,1])];

    %waitbar(ii/MAX_ITER,h_waitbar,'Calculating...');
end
%close(h_waitbar)
%fprintf('Calculation Complete!\n')
toc

%% plots
% parse each state for plotting
x_ref=[]; dx_ref = []; theta_ref =[]; wb_ref=[];
x_mpc=[]; dx_mpc = []; theta_mpc =[]; wb_mpc=[];

for i = 1:length(tout)
    x_ref = [x_ref, Xdout(i,1:3)'];
    dx_ref = [dx_ref, Xdout(i,4:6)'];
    R_ref = reshape(Xdout(i,7:15),[3,3]);
    theta_ref = [theta_ref, vee_map(logm(R_ref))];
    wb_ref = [wb_ref, Xdout(i,16:18)'];

    x_mpc = [x_mpc, Xout(i,1:3)'];
    dx_mpc = [dx_mpc, Xout(i,4:6)'];
    R_mpc = reshape(Xout(i,7:15),[3,3]); %todo make R positive def in EDMD
    theta_mpc = [theta_mpc, vee_map(logm(R_mpc))];
    wb_mpc = [wb_mpc, Xout(i,16:18)'];
end


%% plots
figure(2);
subplot(6,3,[10,13])
plot3(x_ref(1,:), x_ref(2,:), x_ref(3,:)); hold on
plot3(x_mpc(1,:), x_mpc(2,:), x_mpc(3,:), '--'); hold on
grid on; box on; axis square;
xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
axes.LineWidth=2;
lgd = legend('true','predicted');
lgd.Location = 'north';
lgd.NumColumns = 2;

%linear states
subplot(6,3,2)
plot(tout, x_ref(1,:)); hold on;
plot(tout, x_mpc(1,:),'--'); hold on;
axes = gca;
set(axes,'FontSize',15);
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$x$','FontSize',20, 'Interpreter','latex')
box on; axes.LineWidth=2;

subplot(6,3,5)
plot(tout, x_ref(2,:)); hold on; 
plot(tout, x_mpc(2,:),'--'); hold on;
ylabel('$y$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,3,8)
plot(tout, x_ref(3,:)); hold on;
plot(tout, x_mpc(3,:),'--'); hold on;
ylabel('$z$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,3,11)
plot(tout, dx_ref(1,:)); hold on;
plot(tout, dx_mpc(1,:),'--'); hold on;
ylabel('$v_x$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,3,14)
plot(tout, dx_ref(2,:)); hold on;
plot(tout, dx_mpc(2,:),'--'); hold on;
ylabel('$v_y$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,3,17)
plot(tout, dx_ref(3,:)); hold on; 
plot(tout, dx_mpc(3,:),'--'); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$v_z$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

%angular states
subplot(6,3,3)
plot(tout, theta_ref(1,:)); hold on; 
plot(tout, theta_mpc(1,:),'--'); hold on;
ylabel('$\theta$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,3,6)
plot(tout, theta_ref(2,:)); hold on; 
plot(tout, theta_mpc(2,:),'--'); hold on;
ylabel('$\phi$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,3,9)
plot(tout, theta_ref(3,:)); hold on; 
plot(tout, theta_mpc(3,:),'--'); hold on;
ylabel('$\psi$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,3,12)
plot(tout, wb_ref(1,:)); hold on;
plot(tout, wb_mpc(1,:),'--'); hold on;
ylabel('$\omega_x$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,3,15)
plot(tout, wb_ref(2,:)); hold on;
plot(tout, wb_mpc(2,:),'--'); hold on;
ylabel('$\omega_y$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,3,18)
plot(tout, wb_ref(3,:)); hold on;
plot(tout, wb_mpc(3,:),'--'); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$\omega_z$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

