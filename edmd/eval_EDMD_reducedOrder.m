function X = eval_EDMD_reducedOrder(X,U,traj_params,EDMD,show_plot)
%% get EDMD matrices
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
Z1 = EDMD.Z1;
Z2 = EDMD.Z2;

%% Model order reduction using Eigenvectors
[r_eVecs,eVals,l_eVecs] = eig(A);
% [r_eVecs_real, eVals_real_r] = cdf2rdf(r_eVecs, eVals);
[l_eVecs_real, eVals_real_l] = cdf2rdf(l_eVecs, eVals);
[~,sorted_indices] = sort(real(diag(eVals)),'descend');
eVals_sorted = eVals_real_l(sorted_indices,sorted_indices);
% r_eVecs_sorted = r_eVecs(:,sorted_indices);
l_eVecs_sorted = l_eVecs_real(:,sorted_indices);

reduced_model_size = 12;%size(A,2);%       % checking with full model
W_top_reduced = l_eVecs_sorted(:,1:reduced_model_size)';
A_tilde = eVals_sorted(1:reduced_model_size,1:reduced_model_size);
B_tilde = W_top_reduced*B;
C_tilde = C*pinv(W_top_reduced);


%% check prediction on new control input
X_true=[]; X_pred=[];
traj_len = traj_params.traj_len;
t_len = traj_len - 1; % since trajectory for EDMD is smaller by 1 time unit
t_len = [0; t_len]; % add 0 to get consistant trajectory lengths
eval_traj_len = EDMD.eval_horizon*ones(size(traj_len));% part of the trajectory used for evaluation

%loop for each traj
for j=1:length(t_len)-1
    start_idx = sum(t_len(1:j))+1;
    
    % get psi0
    X0 = X(:,start_idx);
    z0 = get_basis(X0,EDMD.n_basis);
    psi0 = W_top_reduced*z0;
    
    % propagate psi0 for prediction
    psi_pred = [];
    n_prediction = eval_traj_len(j); 
    psi = psi0; Psi_pred = [psi0]; % add initial
    for i = start_idx:start_idx+n_prediction-1 % -1 to for n_prediction number of evaluations
        psi_next = A_tilde*psi + B_tilde*U(:,i); % Incomplete: get U as state-feedback
        psi = psi_next;
        Psi_pred = [Psi_pred,psi];
    end
    
    % get Z2 for comparion
    Psi_true = [];
    for i = start_idx:start_idx+n_prediction % n_prediction+1 number of evaluations
        x = X(:,i);
        z = get_basis(x,EDMD.n_basis);
        psi = W_top_reduced*z;
        Psi_true = [Psi_true,psi];
    end

    X_true_j=[]; X_pred_j=[];
    for i = 1:size(Psi_true,2)
        X_true_j = [X_true_j, C_tilde*Psi_true(:,i)];
        X_pred_j = [X_pred_j, C_tilde*Psi_pred(:,i)];
    end
    
    % combine all the trajectories togather
    X_true = [X_true, X_true_j];
    X_pred = [X_pred, X_pred_j];
    
end
eval_traj_len = eval_traj_len+1;
RMSE = rmse(X_pred,X_true,eval_traj_len,show_plot) % 

