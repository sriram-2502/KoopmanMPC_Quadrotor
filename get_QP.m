function [F, G, A_ineq, b_ineq] = get_QP(EDMD,Z,Z_ref,N,params)
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
% define friction constraints 
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
n = size(A,2); % state dimension columns (n x n)

%% get current and desired states
% get X from lifted states Z = [x dx R' w' Rw Rw^2 ...]
X = C*Z;
x = X(1:3); dx = X(4:6); 
R = reshape(X(7:15),[3,3]);
theta = vee_map(logm(R')); 
wb_hat = reshape(X(16:24),[3,3]); % body frame
wb = vee_map(wb_hat');
X_cur = [x;dx;theta;wb]; 

for i =1:N
    X_ref = C*Z_ref(:,i);
    x_ref = X_ref(1:3); dx_ref = X_ref(4:6);
    theta_ref =[ ]; wb_ref = [];
    R_ref = reshape(X_ref(7:15),[3,3]);
    theta_ref = [theta_ref, vee_map(logm(R_ref'))]; 
    wb_hat_ref = reshape(X_ref(16:24),[3,3]); % body frame
    wb_ref = [wb_ref, vee_map(wb_hat_ref')];
end
X_ref = [x_ref;dx_ref;theta_ref;wb_ref];

%% define costs 
Qx = diag([1e4;1e4;1e2]);
Qv = diag([1e4;1e4;1e2]);
Qa = 1e2*eye(9);
Qw = 1e2*eye(9);
Q_i = 0*eye(size(Z,1));
Q_i(1:24,1:24) = blkdiag(Qx, Qv, Qa, Qw);

P = Q_i; % terminal cost

R_i = diag([1e-6;1e0;1e0;1e0]);

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

% iterate columns to get B_hat and R_hat
for i = 1:N 
    %% Augmented Cost
    a = [zeros(i*n,n); eye(n); A_hat(1:end-i*n,:)];
    B_hat = [B_hat, a(n+1:end,:)*B];
    R_hat = blkdiag(R_hat, R_i);
    
    %% Augmented inequality constraint
    
    %A_ineq_i = repmat([-1;1],4,1);
    A_ineq_i = kron(eye(4),[-1;1]);
    A_ineq = blkdiag(A_ineq, A_ineq_i);
    % set lower and upper bounds for control inputs
    u_lb = -50; u_ub = 50; % best working
    %u_lb = -40; u_ub = 40; % best working

    b_ineq_i = [-u_lb; u_ub];
    %b_ineq_i = [-u_lb; u_ub;-10;10;-10;10;-10;10];
    b_ineq_i = repmat(b_ineq_i,4,1);
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