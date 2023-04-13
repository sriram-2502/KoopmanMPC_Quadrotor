function [zval] = casadi_MPC(EDMD,Z_cur,Z_ref,N,params)
import casadi.*
dT = params.Tmpc;
grav = -9.81;
A = EDMD.A;
B = EDMD.B;
C = EDMD.C;
n_states = 12;
n_lifted = size(Z_cur,1);

%% get current and desired states = [p p_dot theta omega gravity]
% get X from lifted states Z = [x dx R' w' Rw Rw^2 ...]
X = C*Z_cur;
x = X(1:3); dx = X(4:6); 
R = reshape(X(7:15),[3,3]);
theta = vee_map(logm(R)); 
wb_hat = reshape(X(16:24),[3,3]); % body frame
wb = vee_map(wb_hat);
X_cur = [x;dx;theta;wb]; 

% desired states
for i =1:N
    X_ref = C*Z_ref(:,i);
    x_ref = X_ref(1:3); dx_ref = X_ref(4:6);
    theta_ref =[ ]; wb_ref = [];
    R_ref = reshape(X_ref(7:15),[3,3]);
    theta_ref = [theta_ref, vee_map(logm(R_ref))]; 
    wb_hat_ref = reshape(X_ref(16:24),[3,3]); % body frame
    wb_ref = [wb_ref, vee_map(wb_hat_ref)];
end
X_ref = [x_ref;dx_ref;theta_ref;wb_ref];

%% get dynamics function
% get augmented states and control for the horizon
% get time varying controls U_i for a horizon
U_i = SX.sym('Ui',4*N); % vector for N horizon

% build state matrix for N horizon
Z = SX.sym('Z',n_lifted,(N+1)); 
Z(:,1) = Z_cur; % initial state
for k = 1:N  
    % build X as a function of U using dynamics
    Z(:,k+1) = A*Z(:,k) + B*U_i(k:k+N-1);
end

%% get objective function and constraints
obj = 0; % Objective function
g = [];  % constraints function vector
args = struct; % constraint values
args.ubg = []; args.lbg = [];

% weights Q and R for obj
Qx = 1e6*eye(3); Qv = 1e6*eye(3);
Qa = 1.5e6*eye(9); Qw = 1e6*eye(9);
Q = blkdiag(Qx, Qv, Qa, Qw);

for k=1:N
    % get x an u for each step in pred horizon
    Z_i = Z(:,k);
    X_i = C*Z_i;

    X_ref = C*Z_ref(:,k);
    u_i = U_i(k:k+N-1);% u for n feets in contact

    % get R matrix based on time varying B matrix
    R_i = 1e-1*eye(size(B,2));
    
    % compute objective function
    obj = obj+(X_i-X_ref)'*Q*(X_i-X_ref) + u_i'*R_i*u_i;
    
    % set max values of u
    u_lb = -1.5 * 1000;
    u_ub = 1.5 * 1000;

    A_ineq_i = [-1;1];
    A_ineq_i = kron(eye(4),A_ineq_i);
    b_ineq_i = [-u_lb; u_ub];

    % friction constraints g_i <= b_i
    g = [g; A_ineq_i*u_i];
    
    % form ineq values
    b_i = b_ineq_i;
    b_i_ub = repmat(b_i,4,1);
    args.lbg = -inf;
    args.ubg = [args.ubg; b_i_ub];   
end

% set up NLP problem
optim_var = U_i;

qp_prob = struct('f', obj, 'x', optim_var, 'g', g);

% set solver options 
% check qpoases user manual
% https://www.coin-or.org/qpOASES/doc/3.0/manual.pdf
opts = struct;
opts.error_on_fail = false;
opts.printLevel = 'none';

% set up solver
solver = qpsol('solver', 'qpoases', qp_prob, opts);

% find optimal solution
sol = solver('lbg',[], 'ubg', []); 
%sol = solver('lbg', args.lbg, 'ubg', args.ubg); 
%sol = solver('x0', args.x0, 'lbg', args.lbg, 'ubg', args.ubg);    

zval = full(sol.x);
zval = zval(1:4);

end