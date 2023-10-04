function [F, G, A_ineq, b_ineq, EDMD] = get_QP(EDMD,Z,Z_ref,N,mpc)
% Inputs
% Z (current states - lifted)
% EDMD (structure with EDMD matrices A B and C)
% Z_des (desired states - lifted)
% N is prediction horizon                   : scalar
% params (struct of parameters)

% Augments
% Q_hat (augmented Q)                       : n(N) x n(N)
% R_hat (augmented R)                       : p(N) x p(N)
% A_hat (augmented A)                       : n(N) x n(N)
% B_hat (augmented B)                       : n(N) x p(N)

% Outputs
% G (augmented cost)                        : p(N) x p(N)
% F (augmented cost)                        : p(N) x 1
% augmented cost                            : 1/2 * X^T * G * X + X^T * F * x0
% A_ineq (augmented ineq constraint A)      : (p+m)(N) x (p+m)(N)
% b_ineq (augmented ineq constraint b)      : 2(p+m)(N) x 1
% augmented ineq constraint                 : A_ineq * X <= b_ineq

%% get system matrices
A = EDMD.A;
B = EDMD.B;

% update rule for EDMD
error_threshold = 1e-4; % tuning parameter
if EDMD.update_flag && size(mpc.X_buffer,2)==mpc.buffer_size % dont calculate error if X_buffer is empty
    error = norm(mpc.X_buffer(1:3,:) - mpc.X_ref_buffer(1:3,:));%/size(mpc.X_buffer,2);
    if error > error_threshold
        EDMD = update_EDMD_M2(EDMD,mpc.X_buffer,mpc.U_buffer);
        A = EDMD.A;
        B = EDMD.B;
    end
end
C = EDMD.C;
n = size(A,2); % state dimension columns (n x n)

%% define costs 

if EDMD.mass_as_input
    % when mass is considered as input/disturbance
    Qx = diag([1e5;1e5;1e5]);
    Qv = diag([1e5;1e5;1e5]);
    Qw = 1e5*eye(3);
    Qr = 1e5*eye(9);
    Q_i = 0*eye(size(Z,1));
    Q_i(1:18,1:18) = blkdiag(Qx, Qv, Qw, Qr);
    P = Q_i; % terminal cost

    R_i = diag([1e-1;1e2;1e2;1e2;1e-1]);
else
    % when mass is NOT considered as input/disturbance
    Qx = diag([1e5;1e5;1e5]);
    Qv = diag([1e5;1e5;1e5]);
    Qw = 1e5*eye(3);
    Qr = 1e5*eye(9);
    Q_i = 0*eye(size(Z,1));
    Q_i(1:18,1:18) = blkdiag(Qx, Qv, Qw, Qr);
    P = Q_i; % terminal cost

    R_i = diag([1e-1;1e2;1e2;1e2]);
end

%% Build QP Matrices
A_hat = zeros(n*N,n);
B_hat = []; % set max value for number of columns
Q_hat = []; R_hat = [];
A_ineq = []; b_ineq = [];

% iterate rows to get A_hat and Q_hat
for i = 1:N 
    A_hat((i-1)*n+1:i*n,:) = A^i;
    Q_hat = blkdiag(Q_hat, Q_i);
end
Q_hat(end-n+1:end,end-n+1:end) = P;

if isempty(mpc.estimated_mass)
    param = get_params(0);
    m_est = param.m;
elseif size(mpc.estimated_mass,2)<=50
%     m_est = round(mean(mpc.estimated_mass(1,:)),4);
    m_est = round(mean(mpc.estimated_mass(3,:)),4);
else
%     m_est = round(mean(mpc.estimated_mass(1,end-10:end)),4);
    m_est = round(mean(mpc.estimated_mass(3,end-50:end)),4);
end
% iterate columns to get B_hat and R_hat
for i = 1:N 
    %% Augmented Cost
    a = [zeros(i*n,n); eye(n); A_hat(1:end-i*n,:)];
    B_hat = [B_hat, a(n+1:end,:)*B];
    R_hat = blkdiag(R_hat, R_i);
    
    %% Augmented inequality constraint
        
    if EDMD.mass_as_input
        %A_ineq_i = repmat([-1;1],4,1);
        A_ineq_i = kron(eye(5),[-1;1]);
        A_ineq = blkdiag(A_ineq, A_ineq_i);

        % set lower and upper bounds for control inputs
        if EDMD.mass_inv_as_param
            u_lb = -1e4.*[2;0.05;0.05;0.05; -1e-4*1./m_est];
            u_ub = 1e4.*[2;0.05;0.05;0.05; 1e-4*1./m_est];
        else
            u_lb = -1e4.*[2;0.05;0.05;0.05; -1e-4*m_est];
            u_ub = 1e4.*[2;0.05;0.05;0.05; 1e-4*m_est];
        end
    else
        %A_ineq_i = repmat([-1;1],4,1);
        A_ineq_i = kron(eye(4),[-1;1]);
        A_ineq = blkdiag(A_ineq, A_ineq_i);

        % set lower and upper bounds for control inputs
        u_lb = -10.*[0;0.05;0.05;0.05];
        u_ub = 10.*[2;0.05;0.05;0.05];
    end
    
%     b_ineq_i = [-u_lb; u_ub];
%     b_ineq_i = repmat(b_ineq_i,4,1);
    
    b_ineq_i = [-u_lb, u_ub]';
    b_ineq_i = b_ineq_i(:);
    b_ineq = [b_ineq; b_ineq_i];

end
R_N = eye(size(B)); % terminal cost
p = size(R_N,2); % get last columns
%R_hat(end-n+1:end,end-p+1:end) = R_N;

% Augmented cost: 1/2 * U^T * G * U + U^T * F
G = 2*(R_hat + B_hat'*Q_hat*B_hat);
y = Z_ref(:);
F = 2*B_hat'*Q_hat*(A_hat*Z-y);
%F = 2*B_hat'*Q_hat*(A_hat*Z);

end