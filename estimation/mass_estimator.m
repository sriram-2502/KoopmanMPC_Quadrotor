% new function struture
function mass_estimate = mass_estimator(X_data, U_data, T_data, mass_change_flag)
% old function structure
% function [mass_estimate, mass_estimate_edmd] = mass_estimator(dt_sim, X_data, X_plus_data, U_data, EDMD)

%% estimation using original dynamics
params = get_params(0); % 0: for no noise
g = params.g;
e3 = [0;0;1];
% v_dots_approximated-> not very accurate
% v_dots_approximated =  (X_data(4:6,:) - X_plus_data(4:6,:))./dt_sim;
% v_dot expected := g * e3 - f / m * R * e3
% m_actual = 2;
v_dots_expected_noisy = [];

f_R_e3 = [];
for i=1:size(X_data,2)
    f_i = U_data(1,i);
    R_i = reshape(X_data(10:18,i), 3, 3);
    f_R_e3 = [f_R_e3, f_i*R_i*e3];
    
%     v_dot_mean_i = g*e3 - 1/m_actual * f_i*R_i*e3;
    X_dot_mean_i = eom_mpc(T_data(:,i), X_data(:,i), U_data(:,i),...
        params, mass_change_flag);
    v_dot_mean_i = X_dot_mean_i(4:6,:);
    v_dot_stdev_i = abs(v_dot_mean_i/20);
    v_dot_expected_noisy_i = normrnd(v_dot_mean_i,v_dot_stdev_i);
    v_dots_expected_noisy = [v_dots_expected_noisy, v_dot_expected_noisy_i];
end
g_e3 = params.g*repmat(e3,1,size(v_dots_expected_noisy,2));

% f*R*e3 = (g*e3 - v_dot)*m => d=C*m

% x = lsqlin(C,d,A,b,Aeq,beq,lb,ub) => least squares for: 
% ||C*x-d||^2 s.t
% A*x <= b;
% A_eq*x = b_eq;
% lb <= x <= ub;

C = g_e3 - v_dots_expected_noisy;
d = f_R_e3;
lb = [];

individual_mass_estimates = d./C;
% 
% mass_estimate_1 = lsqlin(C(1,:)',d(1,:)',[],[],[],[],lb,[]);
% mass_estimate_2 = lsqlin(C(2,:)',d(2,:)',[],[],[],[],lb,[]);
% mass_estimate_3 = lsqlin(C(3,:)',d(3,:)',[],[],[],[],lb,[]);
mass_estimate = mean(individual_mass_estimates,2);

%% estimation using EDMD
% not accurate enough
% Z_1 = []; Z_2 = [];
% % collect basis for n control inputs 
% % Zn = [Zt_1; Zt_2; ... Zt_n]
% for i = 1:size(X_plus_data,2)-1
%     x1 = X_plus_data(:,i); x2 = X_plus_data(:,i+1);
%     z1 = get_basis(x1,EDMD.n_basis);
%     z2 = get_basis(x2,EDMD.n_basis);
%     Z_1 = [Z_1, z1]; Z_2 = [Z_2, z2];
% end
%  % Z_2 = EDMD.A*Z_1 + EDMD.B(:,1:4)*U_data(1:4,:) + EDMD.B(:,5)*mass(:)
%  A_Z_1_data = EDMD.A*Z_1;
%  B_U_data_1_4 = EDMD.B(:,1:4)*U_data(1:4,1:end-1);
%  % Z_data_plus - A_Z_data - B_U_data_1_4 = EDMD.B(:,5)*mass(:)
%  Z_diff = Z_2 - A_Z_1_data - B_U_data_1_4;
%  mass_edmd_all = Z_diff./EDMD.B(:,5);
%  mass_estimate_edmd = mean(mass_edmd_all,2);

end