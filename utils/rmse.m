function RMSE = rmse(X,X_ref,traj_len,show_plot)

% parse each state
x_ref=[]; dx_ref = []; theta_ref =[]; wb_ref=[];
x=[]; dx = []; theta =[]; wb=[];

for i = 1:length(X)
    % for reference
    x_ref = [x_ref, X_ref(1:3,i)];
    dx_ref = [dx_ref, X_ref(4:6,i)];
    R_true = reshape(X_ref(7:15,i)',[3,3]);
    theta_ref = [theta_ref, vee_map(logm(R_true))];
    wb_hat_true = reshape(X_ref(16:24,i),[3,3]);
    wb_ref = [wb_ref, vee_map(wb_hat_true)];

    % for prediction
    x = [x, X(1:3,i)];
    dx = [dx, X(4:6,i)];
    R = reshape(X(7:15,i)',[3,3]); %todo make R positive def in EDMD
    theta = [theta, vee_map(logm(R))];
    wb_hat_pred = reshape(X(16:24,i),[3,3]);
    wb = [wb, vee_map(wb_hat_pred)];
end


%% plots
traj_len = traj_len - 1; % since trajectory for EDMD is smaller by 1 time unit
traj_len = [0; traj_len]; % add 0 to get consistant trajectory lengths
if show_plot
for j=1:length(traj_len)-1
    iter = 1:traj_len(j+1);
    start_idx = sum(traj_len(1:j))+1;
    end_idx = sum(traj_len(1:j+1));
    figure
    subplot(6,2,1)
    plot(iter,x_ref(1,start_idx:end_idx),iter,x(1,start_idx:end_idx),'--','LineWidth',2);
    axes1 = gca;
    box(axes1,'on');
    set(axes1,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$x$",'interpreter','latex', 'FontSize', 20)

    subplot(6,2,3)
    plot(iter,x_ref(2,start_idx:end_idx),iter,x(2,start_idx:end_idx),'--','LineWidth',2);
    axes3 = gca;
    box(axes3,'on');
    set(axes3,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$y$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,5)
    plot(iter,x_ref(3,start_idx:end_idx),iter,x(3,start_idx:end_idx),'--','LineWidth',2);
    axes5 = gca;
    box(axes5,'on');
    set(axes5,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$z$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,7)
    plot(iter,dx_ref(1,start_idx:end_idx),iter,dx(1,start_idx:end_idx),'--','LineWidth',2);
    axes7 = gca;
    box(axes7,'on');
    set(axes7,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$\dot{x}$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,9)
    plot(iter,dx_ref(2,start_idx:end_idx),iter,dx(2,start_idx:end_idx),'--','LineWidth',2);
    axes9 = gca;
    box(axes9,'on');
    set(axes9,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$\dot{y}$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,11)
    plot(iter,dx_ref(3,start_idx:end_idx),iter,dx(3,start_idx:end_idx),'--','LineWidth',2);
    axes11 = gca;
    box(axes11,'on');
    set(axes11,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    xlabel("$\textrm{iterations}$",'interpreter','latex', 'FontSize', 20)
    ylabel("$\dot{z}$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,2)
    plot(iter,theta_ref(1,start_idx:end_idx),iter,theta(1,start_idx:end_idx),'--','LineWidth',2);
    axes2 = gca;
    box(axes2,'on');
    set(axes2,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$\theta_x$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,4)
    plot(iter,theta_ref(2,start_idx:end_idx),iter,theta(2,start_idx:end_idx),'--','LineWidth',2);
    axes4 = gca;
    box(axes4,'on');
    set(axes4,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$\theta_y$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,6)
    plot(iter,theta_ref(3,start_idx:end_idx),iter,theta(3,start_idx:end_idx),'--','LineWidth',2);
    axes6 = gca;
    box(axes6,'on');
    set(axes6,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$\theta_z$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,8)
    plot(iter,wb_ref(1,start_idx:end_idx),iter,wb(1,start_idx:end_idx),'--','LineWidth',2);
    axes8 = gca;
    box(axes8,'on');
    set(axes8,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$\omega_x$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,10)
    plot(iter,wb_ref(2,start_idx:end_idx),iter,wb(2,start_idx:end_idx),'--','LineWidth',2);
    axes10 = gca;
    box(axes10,'on');
    set(axes10,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    ylabel("$\omega_y$",'interpreter','latex', 'FontSize', 20)
    
    subplot(6,2,12)
    plot(iter,wb_ref(3,start_idx:end_idx),iter,wb(3,start_idx:end_idx),'--','LineWidth',2);
    axes12 = gca;
    box(axes12,'on');
    set(axes12,'FontSize',15,'LineWidth',2);
    xlim([1,traj_len(j+1)])
    xlabel("$\textrm{iterations}$",'interpreter','latex', 'FontSize', 20)
    ylabel("$\omega_z$",'interpreter','latex', 'FontSize', 20)
end
end

%% rmse error
x_error = x_ref(:) - x(:);
x_mse = sqrt(mean(x_error.^2))/length(X);
RMSE.x = x_mse;

dx_error = dx_ref(:) - dx(:);
dx_mse = sqrt(mean(dx_error.^2))/length(X);
RMSE.dx = dx_mse;

theta_error = theta_ref(:) - theta(:);
theta_mse = sqrt(mean(theta_error.^2))/length(X);
RMSE.theta = theta_mse;

wb_error = wb_ref(:) - wb(:);
wb_mse = sqrt(mean(wb_error.^2))/length(X);
RMSE.wb = wb_mse;

