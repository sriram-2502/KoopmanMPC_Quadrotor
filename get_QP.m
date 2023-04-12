function [F, G, A_ineq, b_ineq] = get_QP(EDMD, X,Xd,Ud,idx,N,params)
% Inputs
% Xd (desired states)
% Ud (desired foot forces)
% idx (index of feet in contact over horizon)
% N is prediction horizon                   : scalar
% params (dict of parameters)

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
    g = -9.81;
    dT = params.Tmpc;
    
    A = EDMD.A;
    B = EDMD.B;
    C = eye(size(A));
    sys=ss(A,B,C,0);
    [A, ~, C] = ssdata(c2d(sys,dT)); %discretize A
    n = size(A,2); % state dimension columns (n x n)

    %% get current and desired states
    % states = [p p_dot theta omega gravity]
    x = X(1:3,:); dx = X(4:6,:);
    R = reshape(X(7:15,:),[3,3]);
    theta = veeMap(logm(R)); 
    wb_hat = X(16:24,:); % body frame
    wb = vee_map(wb_hat);
    X_cur = [x;dx;theta;wb]; 

    xd = Xd(1:3,:); vd = Xd(4:6,:);
    thetad = []; wd = Xd(16:18,:); % body frame
    wd_world = [];
    X_des = [xd;vd];

    %% define costs 
    Qx = 1e6*eye(3);
    Qv = 1e6*eye(3);
    Qa = 1e6*eye(3);
    Qw = 1e6*eye(3);
    Q_i = blkdiag(Qx, Qv, Qa, Qw, 1e-5);
    P = Q_i; % terminal cost
    
    %% Build QP Matrices
    A_hat = zeros(n*N,n);
    B = []; b = [];
    B_hat = []; % set max value for number of columns
    Q_hat = []; R_hat = [];
    A_ineq = []; b_ineq = [];
    
    % iterate rows to get A_hat and Q_hat
    for i = 1:N 
        A_hat((i-1)*n+1:i*n,:) = A^i;
        Q_hat = blkdiag(Q_hat, Q_i);

        % get euler angles for each horizon
        Rd = reshape(Xd(7:15,i),[3,3]);
        thetad = [thetad, veeMap(logm(Rd))];   
        wd_world =  [wd_world, Rd*wd(:,i)];
    end
    Q_hat(end-n+1:end,end-n+1:end) = P;
    X_des = [X_des;thetad;wd_world;g.*ones(1,N)]; 

    % iterate columns to get B_hat and R_hat
    for i = 1:N 
        %% Augmented Cost
        B_i = get_B(X,Xd,idx,i,params);
        sys=ss(A,B_i,C,0);
        [~, B_i, ~] = ssdata(c2d(sys,dT)); %discretize B

        a = [zeros(i*n,n); eye(n); A_hat(1:end-i*n,:)];
        B_hat = [B_hat, a(n+1:end,:)*B_i];
        R_i = 1e1*eye(size(B_i,2));
        R_hat = blkdiag(R_hat, R_i);
        
        %% Augmented inequality constraint
        % get number of feet in contact
        num_feet_contact = length(nonzeros(idx(:,i)));
        
        A_ineq_i = [-1  0 -mu;...
                     1  0 -mu;...
                     0  -1 -mu;...
                     0  1 -mu;...
                     0  0  -1;...
                     0  0  1];
        A_ineq_i = kron(eye(num_feet_contact),A_ineq_i);
        A_ineq = blkdiag(A_ineq, A_ineq_i);
        % set max values of fi_z
        Fzd = Ud([3 6 9 12],i);
        fi_z_lb = 0 * max(Fzd);
        fi_z_ub = 1.5 * max(Fzd);
        
        b_ineq_i = [0; 0; 0; 0; -fi_z_lb; fi_z_ub];
        b_ineq_i = repmat(b_ineq_i,num_feet_contact,1);
        b_ineq = [b_ineq; b_ineq_i];

    end
    R_N = eye(size(B_i)); % terminal cost
    p = size(R_N,2); % get last columns
    %R_hat(end-n+1:end,end-p+1:end) = R_N;
    
    % Augmented cost: 1/2 * U^T * G * U + U^T * F
    G = 2*(R_hat + B_hat'*Q_hat*B_hat);
    y = reshape(X_des,[],1);
    F = 2*B_hat'*Q_hat*(A_hat*X_cur-y);
    
end