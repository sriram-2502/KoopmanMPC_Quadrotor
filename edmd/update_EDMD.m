function EDMD = update_EDMD(EDMD,X_buffer,U_buffer)

Z_online = [];
for i=1:size(X_buffer,2)
    Z_online = [Z_online, get_basis(X_buffer(:,i),EDMD.n_basis)];
end

Z1_online = Z_online(:,1:end-1);
Z2_online = Z_online(:,2:end);
U_online = U_buffer(:,1:end-1);

data_per_update = size(X_buffer,2);

A_old = EDMD.Inter_A; G_old = EDMD.Inter_G;

Z1_online_aug = [Z1_online; U_online];

shuffle_order = randperm(size(Z1_online_aug,2));
Z1_online_aug = Z1_online_aug(:,shuffle_order);
Z2_online = Z2_online(:,shuffle_order);


A_update = Z2_online*Z1_online_aug'/data_per_update;
G_update = Z1_online_aug*Z1_online_aug'/data_per_update;

update_weight = 0.0;
A_hat = (1-update_weight)*A_old + update_weight*(A_update);
G_hat = (1-update_weight)*G_old + update_weight*(G_update);
EDMD.Inter_A = A_hat;
EDMD.Inter_G = G_hat;

EDMD.K_online = A_hat*pinv(G_hat);
EDMD.A_online = EDMD.K_online(:,1:size(Z1_online,1));
EDMD.B_online = EDMD.K_online(:,size(Z1_online,1)+1:end);

EDMD.update_flag = true;