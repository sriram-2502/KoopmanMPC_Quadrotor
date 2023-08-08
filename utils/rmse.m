function RMSE = rmse(X,X_ref)

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

iter = 1:(size(X,2));
figure
subplot(6,2,1)
plot(iter,x_ref(1,:),iter,x(1,:),'--');
ylabel('x')

subplot(6,2,2)
plot(iter,x_ref(2,:),iter,x(2,:),'--');
ylabel('y')

subplot(6,2,3)
plot(iter,x_ref(3,:),iter,x(3,:),'--');
ylabel('z')

subplot(6,2,4)
plot(iter,dx_ref(1,:),iter,dx(1,:),'--');
ylabel('x_dot')

subplot(6,2,5)
plot(iter,dx_ref(2,:),iter,dx(2,:),'--');
ylabel('y_dot')

subplot(6,2,6)
plot(iter,dx_ref(3,:),iter,dx(3,:),'--');
ylabel('z_dot')

subplot(6,2,7)
plot(iter,theta_ref(1,:),iter,theta(1,:),'--');
ylabel('roll')

subplot(6,2,8)
plot(iter,theta_ref(2,:),iter,theta(2,:),'--');
ylabel('pitch')

subplot(6,2,9)
plot(iter,theta_ref(3,:),iter,theta(3,:),'--');
ylabel('yaw')

subplot(6,2,10)
plot(iter,wb_ref(1,:),iter,wb(1,:),'--');
ylabel('w_x')

subplot(6,2,11)
plot(iter,wb_ref(2,:),iter,wb(2,:),'--');
ylabel('w_y')

subplot(6,2,12)
plot(iter,wb_ref(3,:),iter,wb(3,:),'--');
ylabel('w_z')

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

