clc; clear; close all;

params = get_params();
EDMD = get_EDMD();
A = EDMD.A;
B = EDMD.B;
Z = EDMD.Z;
U = EDMD.U;

% check prediction on the same distrubution as || Z2 - (AZ1 + BU1) ||_mse
Z1 = Z(:,1:end-1);
U1 = U(:,1:end-1);
Z2 = Z(:,2:end);
Z2_predicted = A*Z1 + B*U1;
Z2_error = Z2 - Z2_predicted;
Z2_mse = sqrt(mean(Z2_error(:).^2))