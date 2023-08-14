function RMSE = rmse(X,X_ref)

% parse each state
x_ref=[]; dx_ref = []; theta_ref =[]; wb_ref=[];
x=[]; dx = []; theta =[]; wb=[];

for i = 1:length(X)
    x_ref = [x_ref, X_ref(1:3,i)];
    dx_ref = [dx_ref, X_ref(4:6,i)];
    R_true = reshape(X_ref(7:15,i)',[3,3]);
    theta_ref = [theta_ref, vee_map(logm(R_true))];
    wb_hat_true = reshape(X_ref(16:24,i),[3,3]);
    wb_ref = [wb_ref, vee_map(wb_hat_true)];

    x = [x, X(1:3,i)];
    dx = [dx, X(4:6,i)];
    R = reshape(X(7:15,i)',[3,3]); %todo make R positive def in EDMD
    theta = [theta, vee_map(logm(R))];
    wb_hat_pred = reshape(X(16:24,i),[3,3]);
    wb = [wb, vee_map(wb_hat_pred)];
end

x_error = x_ref(:) - x(:);
x_mse = sqrt(mean(x_error.^2))/sqrt(mean(x_ref(:).^2));
RMSE.x = x_mse;

dx_error = dx_ref(:) - dx(:);
dx_mse = sqrt(mean(dx_error.^2))/sqrt(mean(dx_ref(:).^2));
RMSE.dx = dx_mse;

theta_error = theta_ref(:) - theta(:);
theta_mse = sqrt(mean(theta_error.^2))/sqrt(mean(theta_ref(:).^2));
RMSE.theta = theta_mse;

wb_error = wb_ref(:) - wb(:);
wb_mse = sqrt(mean(wb_error.^2))/sqrt(mean(wb_ref(:).^2));
RMSE.wb = wb_mse;

