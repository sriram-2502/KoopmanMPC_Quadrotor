function EDMD = update_EDMD(EDMD,X_buffer,U_buffer)
Z_online = [];
for i=1:size(X_buffer,2)
    Z_online = [Z_online, get_basis(X_buffer(:,i),EDMD.n_basis)];
end

Z1_online = Z_online(:,1:end-1);
Z2_online = Z_online(:,2:end);
U_online = U_buffer(:,1:end-1);

data_per_update = size(X_buffer,2);

total_data = size(X_buffer,2);
A_old = EDMD.Inter_A; G_old = EDMD.Inter_G;

psiZ_tilde = Z2_online;
psiZ = [Z1_online; U_online];

A_update = psiZ_tilde*psiZ'/data_per_update;
G_update = psiZ*psiZ'/data_per_update;

lam = 0.25;
A_hat = (1-lam)*A_old + lam*(A_update);
G_hat = (1-lam)*G_old + lam*(G_update);
EDMD.Inter_A = A_hat;
EDMD.Inter_G = G_hat;

EDMD.K_online = A_hat*pinv(G_hat);
EDMD.A_online = EDMD.K_online(:,1:size(Z1_online,1));
EDMD.B_online = EDMD.K_online(:,size(Z1_online,1)+1:end);

