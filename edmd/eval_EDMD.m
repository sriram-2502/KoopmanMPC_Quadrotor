function X = eval_EDMD(X0,dt,t_span,EDMD,n_basis,show_plot)
%% get EDMD matrices
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;

%% check prediction on new control input
n_control = 50; % number of random controls to apply
t_traj = 0:dt:t_span; % traj length to simulate (s)
x_err = 0; dx_err = 0;
theta_err = 0; wb_err = 0;
for i = 1:n_control
    flag = 'val';
    [X, U, X1, X2] = get_rnd_trajectories(X0,n_control,t_traj,show_plot,flag);
    
    % get z0
    basis = get_basis(X0,n_basis);
    z0 = [X0(1:3); X0(4:6); basis];
    
    % propagate z0 for prediction
    Z_pred = [];
    n_prediction = 100; % 100 timesteps
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

    X_ref=[]; X=[];
    for i = 1:length(Z_true)
        X_ref = [X_ref, EDMD.C*Z_true(:,i)];
        X = [X, EDMD.C*Z_pred(:,i)];
        
    end

    % get rmse for each control
    RMSE_i = rmse(X,X_ref);

    % sum rmse for each control
    x_err =  [x_err, RMSE_i.x];
    dx_err =  [dx_err, RMSE_i.dx];
    theta_err =  [theta_err, RMSE_i.theta];
    wb_err =  [wb_err, RMSE_i.wb];

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
    X1 = []; X2 = [];
    for i = 1:length(t_pred)
        x_true = [x_true, X_true(1:3,i)];
        dx_true = [dx_true, X_true(4:6,i)];
        R_true = reshape(X_true(7:15,i),[3,3])'; % get R from lifted states R.T
        theta_true = [theta_true, vee_map(logm(R_true))]; % get wb_hat from lifted states wb_hat.T
        wb_hat_true = reshape(X_true(16:24,i),[3,3])';
        wb_true = [wb_true, vee_map(wb_hat_true)];
    
        x_pred = [x_pred, X_pred(1:3,i)];
        dx_pred = [dx_pred, X_pred(4:6,i)];
        R_pred = reshape(X_pred(7:15,i)',[3,3]); %todo make R positive def in EDMD
        theta_pred = [theta_pred, vee_map(logm(R_pred))];
        wb_hat_pred = reshape(X_pred(16:24,i),[3,3]);
        wb_pred = [wb_pred, vee_map(wb_hat_pred)];
    end
    
    X2.x = x_true; X2.dx = dx_true;
    X2.theta = theta_true; X2.wb = wb_true;
    
    X1.x = x_pred; X1.dx = dx_pred;
    X1.theta = theta_pred; X1.wb = wb_pred;
    
    %% plots
    colors = colororder;
    blue = colors(1,:);
    red = colors(2,:);
    figure(1); hold on;
    subplot(2,4,2)
    plot3(x_true(1,:), x_true(2,:), x_true(3,:),'color',blue); hold on
    plot3(x_pred(1,:), x_pred(2,:), x_pred(3,:), 'color',red,'linestyle','--'); hold on
    grid on; box on; axis square;
    xlabel('$x_1$','FontSize',20, 'Interpreter','latex')
    ylabel('$x_2$','FontSize',20, 'Interpreter','latex')
    zlabel('$x_3$','FontSize',20, 'Interpreter','latex')
    axes = gca; set(axes,'FontSize',15);
    axes.LineWidth=2;
    lgd = legend('true','predicted');
    lgd.Location = 'north';
    lgd.NumColumns = 2;
end
% avg rmse for n_controls
RMSE_val.x = [mean(x_err), std(x_err)];
RMSE_val.dx = [mean(dx_err), std(dx_err)];
RMSE_val.theta = [mean(theta_err), std(theta_err)];
RMSE_val.wb = [mean(wb_err), std(wb_err)];
RMSE_val

% state traj plots
% fig_num = 30; flag = 'edmd';
% state_plots(fig_num,t_traj,X1,X2,flag)