function X = eval_EDMD_pid(traj_param,EDMD,n_basis,show_plot)
%% get EDMD matrices
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;

%% Extract trajectory parameters


%% check prediction on new control input
[T, X, U, X1, X2, U1, U2, traj_len] = get_pid_trajectories(traj_param);
X_ref=[]; X_pred=[];

t_len = traj_len - 1; % since trajectory for EDMD is smaller by 1 time unit
t_len = [0; t_len]; % add 0 to get consistant trajectory lengths
eval_traj_len = 100*ones(size(traj_len));%floor(traj_len./50); % part of the trajectory used for evaluation
for j=1:length(t_len)-1
    start_idx = sum(t_len(1:j))+1;
    
    % get z0
    X0 = X(:,start_idx);
    basis = get_basis(X0,n_basis);
    z0 = [X0(1:3); X0(4:6); basis];
    
    % propagate z0 for prediction
    Z_pred = [];
    n_prediction = eval_traj_len(j); % 100 timesteps
    z = z0; Z_pred = [z0]; % add initial
    for i = start_idx:start_idx+n_prediction
        z_next = A*z + B*U(:,i);
        z = z_next;
        Z_pred = [Z_pred,z];
    end
    
    % get Z2 for comparion
    Z_true = [];
    for i = start_idx:start_idx+n_prediction+1 % compare n+1 timesteps
        x = X(:,i);
        basis = get_basis(x,n_basis);
        z = [x(1:3); x(4:6); basis];
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

RMSE = rmse(X_pred,X_ref,eval_traj_len,show_plot)

