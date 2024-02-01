function RMSE = rmse(X_pred,X_true,traj_len,show_plot)

% parse each state
x_true=[]; dx_true = []; theta_true =[]; wb_true=[];
x_pred=[]; dx_pred = []; theta_pred =[]; wb_pred=[];

for i = 1:size(X_pred,2)
    % for reference
    x_true = [x_true, X_true(1:3,i)];
    dx_true = [dx_true, X_true(4:6,i)];
    bRw_ref = reshape(X_true(10:18,i),[3,3]);
    theta_true = [theta_true, vee_map(logm(bRw_ref))];
    wb_true = [wb_true, X_true(7:9,i)]; 

    % for prediction
    x_pred = [x_pred, X_pred(1:3,i)];
    dx_pred = [dx_pred, X_pred(4:6,i)];
    bRw = reshape(X_pred(10:18,i),[3,3]);
    theta_pred = [theta_pred, vee_map(logm(bRw))];
    wb_pred = [wb_pred, X_pred(7:9,i)];
end

x_err = []; dx_err = []; theta_err = []; wb_err = [];

%% get rmse for each trajectory
traj_len = traj_len; 
traj_len = [0; traj_len]; % add 0 to get consistant trajectory lengths
for j=1:length(traj_len)-1
    iter = 1:traj_len(j+1);
    start_idx = sum(traj_len(1:j))+1;
    end_idx = sum(traj_len(1:j+1));

    % parse states
    x_true_j = x_true(:,start_idx:end_idx);
    dx_true_j = dx_true(:,start_idx:end_idx);
    theta_true_j = theta_true(:,start_idx:end_idx);
    wb_true_j = wb_true(:,start_idx:end_idx);

    % for prediction
    x_pred_j = x_pred(:,start_idx:end_idx);
    dx_pred_j = dx_pred(:,start_idx:end_idx);
    theta_pred_j = theta_pred(:,start_idx:end_idx);
    wb_pred_j = wb_pred(:,start_idx:end_idx);

    % get rmse for each trajectory
%     x_error = x_true_j(:) - x_pred_j(:);
    x_error = x_true_j(:,end) - real(x_pred_j(:,end));
%     x_mse = sqrt(mean(x_error.^2))/length(X_pred);
    x_mse = norm(x_error);
    x_pcent = mean(abs(x_error)./abs(x_true_j(:,end))*100);
    RMSE.x = x_mse;%x_pcent; %

%     dx_error = dx_true_j(:) - dx_pred_j(:);
%     dx_mse = sqrt(mean(dx_error.^2))/length(X_pred);
    dx_error = dx_true_j(:,end) - real(dx_pred_j(:,end));
    dx_mse = norm(dx_error);
    dx_pcent = mean(abs(dx_error)./abs(dx_true_j(:,end))*100);
    RMSE.dx = dx_mse;%dx_pcent;% 

%     theta_error = theta_true_j(:) - theta_pred_j(:);
%     theta_mse = sqrt(mean(theta_error.^2))/length(X_pred);
    theta_error = theta_true_j(:,end) - real(theta_pred_j(:,end));
    theta_mse = norm(theta_error);
    theta_pcent = mean(abs(theta_error)./abs(theta_true_j(:,end))*100);
    RMSE.theta = theta_mse;%theta_pcent;% 

%     wb_error = wb_true_j(:) - wb_pred_j(:);
%     wb_mse = sqrt(mean(wb_error.^2))/length(X_pred);
    wb_error = wb_true_j(:,end) - real(wb_pred_j(:,end));
    wb_mse = norm(wb_error);
    wb_pcent = mean(abs(wb_error)./abs(wb_true_j(:,end))*100);
    RMSE.wb = wb_mse;%wb_pcent;% 

    % sum rmse for each control
    x_err =  [x_err, RMSE.x];
    dx_err =  [dx_err, RMSE.dx];
    theta_err =  [theta_err, RMSE.theta];
    wb_err =  [wb_err, RMSE.wb];
    
    % plot the trajectories if show_plot = true
    if show_plot
        figure
        subplot(6,2,1)
        plot(iter,x_true(1,start_idx:end_idx),iter,x_pred(1,start_idx:end_idx),'--','LineWidth',2);
        axes1 = gca;
        box(axes1,'on');
        set(axes1,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$x$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,3)
        plot(iter,x_true(2,start_idx:end_idx),iter,x_pred(2,start_idx:end_idx),'--','LineWidth',2);
        axes3 = gca;
        box(axes3,'on');
        set(axes3,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$y$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,5)
        plot(iter,x_true(3,start_idx:end_idx),iter,x_pred(3,start_idx:end_idx),'--','LineWidth',2);
        axes5 = gca;
        box(axes5,'on');
        set(axes5,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$z$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,7)
        plot(iter,dx_true(1,start_idx:end_idx),iter,dx_pred(1,start_idx:end_idx),'--','LineWidth',2);
        axes7 = gca;
        box(axes7,'on');
        set(axes7,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$\dot{x}$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,9)
        plot(iter,dx_true(2,start_idx:end_idx),iter,dx_pred(2,start_idx:end_idx),'--','LineWidth',2);
        axes9 = gca;
        box(axes9,'on');
        set(axes9,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$\dot{y}$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,11)
        plot(iter,dx_true(3,start_idx:end_idx),iter,dx_pred(3,start_idx:end_idx),'--','LineWidth',2);
        axes11 = gca;
        box(axes11,'on');
        set(axes11,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        xlabel("$\textrm{iterations}$",'interpreter','latex', 'FontSize', 20)
        ylabel("$\dot{z}$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,2)
        plot(iter,theta_true(1,start_idx:end_idx),iter,theta_pred(1,start_idx:end_idx),'--','LineWidth',2);
        axes2 = gca;
        box(axes2,'on');
        set(axes2,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$\theta_x$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,4)
        plot(iter,theta_true(2,start_idx:end_idx),iter,theta_pred(2,start_idx:end_idx),'--','LineWidth',2);
        axes4 = gca;
        box(axes4,'on');
        set(axes4,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$\theta_y$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,6)
        plot(iter,theta_true(3,start_idx:end_idx),iter,theta_pred(3,start_idx:end_idx),'--','LineWidth',2);
        axes6 = gca;
        box(axes6,'on');
        set(axes6,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$\theta_z$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,8)
        plot(iter,wb_true(1,start_idx:end_idx),iter,wb_pred(1,start_idx:end_idx),'--','LineWidth',2);
        axes8 = gca;
        box(axes8,'on');
        set(axes8,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$\omega_x$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,10)
        plot(iter,wb_true(2,start_idx:end_idx),iter,wb_pred(2,start_idx:end_idx),'--','LineWidth',2);
        axes10 = gca;
        box(axes10,'on');
        set(axes10,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        ylabel("$\omega_y$",'interpreter','latex', 'FontSize', 20)

        subplot(6,2,12)
        plot(iter,wb_true(3,start_idx:end_idx),iter,wb_pred(3,start_idx:end_idx),'--','LineWidth',2);
        axes12 = gca;
        box(axes12,'on');
        set(axes12,'FontSize',15,'LineWidth',2);
        xlim([1,traj_len(j+1)])
        xlabel("$\textrm{iterations}$",'interpreter','latex', 'FontSize', 20)
        ylabel("$\omega_z$",'interpreter','latex', 'FontSize', 20)

        legend('actual','predicted')
    end
end

% avg rmse for all trajectories
RMSE_avg.x = [mean(x_err), std(x_err)];
RMSE_avg.dx = [mean(dx_err), std(dx_err)];
RMSE_avg.theta = [mean(theta_err), std(theta_err)];
RMSE_avg.wb = [mean(wb_err), std(wb_err)];
RMSE_avg


