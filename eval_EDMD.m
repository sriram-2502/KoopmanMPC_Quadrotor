function X = eval_EDMD(X0,dt,t_span,EDMD,n_basis,show_plot)
%% get EDMD matrices
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;

%% check prediction on new control input
n_control = 1; % number of random controls to apply
t_traj = 0:dt:t_span; % traj length to simulate (s)
[X, U, X1, X2] = get_rnd_trajectories(X0,n_control,t_traj,show_plot);

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

Z2_error = Z_true(:) - Z_pred(:);
Z2_mse_prediction = sqrt(mean(Z2_error.^2))/sqrt(mean(Z_true(:).^2))

%% get true states from lifted states
t_pred = (0:n_prediction) * dt;
X_true=[]; X_pred=[];
for i = 1:length(t_pred)
    X_true = [X_true, C*Z_true(:,i)];
    X_pred = [X_pred, C*Z_pred(:,i)];
    
end

% parse each state for plotting
x_true=[]; dx_true = []; theta_true =[]; wb_true=[];
x_pred=[]; dx_pred = []; theta_pred =[]; wb_pred=[];
for i = 1:length(t_pred)
    x_true = [x_true, X_true(1:3,i)];
    dx_true = [dx_true, X_true(4:6,i)];
    R_true = reshape(X_true(7:15,i)',[3,3]);
    theta_true = [theta_true, vee_map(logm(R_true))];
    wb_hat_true = reshape(X_true(16:24,i),[3,3]);
    wb_true = [wb_true, vee_map(wb_hat_true)];

    x_pred = [x_pred, X_pred(1:3,i)];
    dx_pred = [dx_pred, X_pred(4:6,i)];
    R_pred = reshape(X_pred(7:15,i)',[3,3]); %todo make R positive def in EDMD
    theta_pred = [theta_pred, vee_map(logm(R_pred))];
    wb_hat_pred = reshape(X_pred(16:24,i),[3,3]);
    wb_pred = [wb_pred, vee_map(wb_hat_pred)];
end


%% plots
figure(1); hold on;
%subplot(6,3,[10,13])
subplot(2,4,2)
%scatter3(Z_true(1,1), Z_true(2,1), Z_true(3,1),sz,'kx'); hold on
plot3(x_true(1,:), x_true(2,:), x_true(3,:)); hold on
plot3(x_pred(1,:), x_pred(2,:), x_pred(3,:), '--'); hold on
grid on; box on; axis square;
xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
axes.LineWidth=2;
lgd = legend('true','predicted');
lgd.Location = 'north';
lgd.NumColumns = 2;

%% trajjectory plots
figure(2)
%linear states
subplot(6,4,1)
plot(t_traj, x_true(1,:)); hold on;
plot(t_traj, x_pred(1,:),'--'); hold on;
axes = gca;
set(axes,'FontSize',15);
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$x$','FontSize',20, 'Interpreter','latex')
box on; axes.LineWidth=2;

subplot(6,4,5)
plot(t_traj, x_true(2,:)); hold on; 
plot(t_traj, x_pred(2,:),'--'); hold on;
ylabel('$y$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,4,9)
plot(t_traj, x_true(3,:)); hold on;
plot(t_traj, x_pred(3,:),'--'); hold on;
ylabel('$z$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,4,13)
plot(t_traj, dx_true(1,:)); hold on;
plot(t_traj, dx_pred(1,:),'--'); hold on;
ylabel('$v_x$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,4,17)
plot(t_traj, dx_true(2,:)); hold on;
plot(t_traj, dx_pred(2,:),'--'); hold on;
ylabel('$v_y$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,4,21)
plot(t_traj, dx_true(3,:)); hold on; 
plot(t_traj, dx_pred(3,:),'--'); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$v_z$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

%angular states
subplot(6,4,2)
plot(t_traj, theta_true(1,:)); hold on; 
plot(t_traj, theta_pred(1,:),'--'); hold on;
ylabel('$\theta$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,4,6)
plot(t_traj, theta_true(2,:)); hold on; 
plot(t_traj, theta_pred(2,:),'--'); hold on;
ylabel('$\phi$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,4,10)
plot(t_traj, theta_true(3,:)); hold on; 
plot(t_traj, theta_pred(3,:),'--'); hold on;
ylabel('$\psi$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,4,14)
plot(t_traj, wb_true(1,:)); hold on;
plot(t_traj, wb_pred(1,:),'--'); hold on;
ylabel('$\omega_x$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,4,18)
plot(t_traj, wb_true(2,:)); hold on;
plot(t_traj, wb_pred(2,:),'--'); hold on;
ylabel('$\omega_y$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;

subplot(6,4,22)
plot(t_traj, wb_true(3,:)); hold on;
plot(t_traj, wb_pred(3,:),'--'); hold on;
xlabel('$t$ (s)','FontSize',20, 'Interpreter','latex')
ylabel('$\omega_z$','FontSize',20, 'Interpreter','latex')
axes = gca; set(axes,'FontSize',15);
box on; axes.LineWidth=2;