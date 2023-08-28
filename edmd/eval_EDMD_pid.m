function X = eval_EDMD_pid(X,U,traj_params,EDMD,n_basis,show_plot)
%% get EDMD matrices
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;

%% check prediction on new control input
X_ref=[]; X_pred=[];
traj_len = traj_params.traj_len;
t_len = traj_len - 1; % since trajectory for EDMD is smaller by 1 time unit
t_len = [0; t_len]; % add 0 to get consistant trajectory lengths
traj_params.eval_traj_len = 50*ones(size(traj_len));% part of the trajectory used for evaluation

%loop for each traj
for j=1:length(t_len)-1
    start_idx = sum(t_len(1:j))+1 +500;
    
    % get z0
    X0 = X(:,start_idx);
    z0 = get_basis(X0,n_basis);
    
    % propagate z0 for prediction
    Z_pred = [];
    n_prediction = traj_params.eval_traj_len(j); % 100 timesteps
    z = z0; Z_pred = [z0]; % add initial
    for i = start_idx:start_idx+n_prediction-1 % -1 to for n_prediction number of evaluations
        z_next = A*z + B*U(:,i); % Incomplete: get U as state-feedback
        z = z_next;
        Z_pred = [Z_pred,z];
    end
    
    % get Z2 for comparion
    Z_true = [];
    for i = start_idx:start_idx+n_prediction % n_prediction+1 number of evaluations
        x = X(:,i);
        z = get_basis(x,n_basis);
        Z_true = [Z_true,z];
    end

    X_ref_j=[]; X_pred_j=[];
    for i = 1:size(Z_true,2)
        X_ref_j = [X_ref_j, EDMD.C*Z_true(:,i)];
        X_pred_j = [X_pred_j, EDMD.C*Z_pred(:,i)];
    end
    
    % combine all the trajectories togather
    X_ref = [X_ref, X_ref_j];
    X_pred = [X_pred, X_pred_j];
    
end
traj_params.eval_traj_len = traj_params.eval_traj_len+1;
RMSE = rmse(X_pred,X_ref,traj_params.eval_traj_len,show_plot) % 

