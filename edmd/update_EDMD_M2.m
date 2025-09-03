function EDMD = update_EDMD_M2(EDMD,X_buffer,U_buffer)

Z_online = [];
for i=1:size(X_buffer,2)
    Z_online = [Z_online, get_basis(X_buffer(:,i),EDMD.n_basis)];
end

Z1_online = Z_online(:,1:end-1);
Z2_online = Z_online(:,2:end);
U1_online = U_buffer(:,1:end-1);

A_old = EDMD.A; B_old = EDMD.B;
Z1_online_aug = [Z1_online; U1_online];

Z2_pred_online = A_old*Z1_online + B_old*U1_online;
D2_online = Z2_online - Z2_pred_online;
norm(D2_online)

data_per_update = size(D2_online,2);

shuffle_order = randperm(size(Z1_online_aug,2));
Z1_online_aug = Z1_online_aug(:,shuffle_order);
D2_online = D2_online(:,shuffle_order);

% % psuedo inverse (no regularization)
% Inter_M1 = D2_online*Z1_online_aug'/data_per_update;        % A_update from EDMD
% Inter_M2 = Z1_online_aug*Z1_online_aug'/data_per_update;    % G_update from EDMD
% delta_K = Inter_M1*pinv(Inter_M2);

% % with regularization (YALMIP)
% lam = 0.5;
% delta_K=sdpvar(size(EDMD.K,1),size(EDMD.K,2));
% Objective =  (1-lam)*norm(D2_online - delta_K*Z1_online_aug) ...
%     - lam*norm(delta_K);
% Constraints = [];
% opt = sdpsettings('solver','quadprog','verbose',0,'cachesolvers',1);
% sol1 = optimize(Constraints,Objective,opt);
% delta_K = value(delta_K);
% % U = round(U,4);
% fprintf('error objective = %f\n', double(Objective));

% with regularization (cvx)
lam = 0.5;
cvx_begin quiet
variable delta_K(size(EDMD.K,1),size(EDMD.K,2))
minimize ((1-lam)*norm(D2_online - delta_K*Z1_online_aug) + lam*norm(delta_K))
cvx_end

EDMD.K = EDMD.K + delta_K;
EDMD.A = EDMD.K(:,1:size(Z1_online,1));
EDMD.B = EDMD.K(:,size(Z1_online,1)+1:end);

A_old = EDMD.A; B_old = EDMD.B;
Z1_online_aug = [Z1_online; U1_online];

Z2_pred_online = A_old*Z1_online + B_old*U1_online;
D2_online = Z2_online - Z2_pred_online;
norm(D2_online)

EDMD.update_flag = true;