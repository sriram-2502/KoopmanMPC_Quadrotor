function mpc = sim_MPC(EDMD,Z0,Z_ref,X_ref,mpc_params)
% --- logging ---
dt_sim = mpc_params.simTimeStep;
N = mpc_params.predHorizon;
tstart = 0;
tend = dt_sim;
params = sys_params;

[tout,Xout,Uout,Xdout] = deal([]);

%% --- simulation ----
h_waitbar = waitbar(0,'Calculating...');
tic
Z = Z0;
for ii = 1:mpc_params.MAX_ITER
    % --- time vector ---
    %t_ = dt_sim * (ii-1) + params.Tmpc * (0:N-1);

    %% --- MPC ----      
    %form QP using explicit matrices
    z_ref = Z_ref(:,ii:ii+N-1);

    if(mpc_params.use_casadi)
        [zval] = casadi_MPC(EDMD,Z,Z_ref,N,mpc_params);
    else
        tic
        [f, G, A, b] = get_QP(EDMD,Z,z_ref,N,mpc_params);
        % solve QP using quadprog     
        [zval] = quadprog(G,f,A,b,[],[],[],[]);
        toc
    end

    Ut = zval(1:4);

    %% --- simulate without any external disturbances ---
    %parse true states from lifted states
    Xt = EDMD.C*Z;
    x = Xt(1:3); dx = Xt(4:6); 
    wRb = reshape(Xt(7:15),[3,3]);
    theta = vee_map(logm(wRb'));
    wb_hat = reshape(Xt(16:24),[3,3]); % body frame
    wb = vee_map(wb_hat');
    Xt = [x;dx;wRb(:);wb];
    
    % use dynamics_SRB for ode45
    quad_params = sys_params;
    [t,X] = ode45(@(t,s) dynamics_SRB(t, s, Ut, quad_params),[tstart,tend],Xt);
    
    % use pid dynamics for ode45
%     % Xt = [x; dx in world frame; quartornions; body frame angular
%     % velocities]
%     bRw = wRb';
%     Rot = RPYtoRot_ZXY(theta(1),theta(2),theta(3));
%     q = RotToQuat(Rot);
%     Xt = [x;dx;q;wb;]; % Xt for pid simulation (in world frame)
% 
%     [t,X_pid] = ode45(@(t,s) quadEOM_readonly(t, s, Ut(1), Ut(2:end), params),[tstart,tend],Xt);
%     X = parse_edmd(t,X_pid);
    
    %% --- update ---
    Xt = X(end,:)'; % Xt for EDMD (in body frame)
    % get lifted states at t=0
    basis = get_basis(Xt,EDMD.n_basis);
    Z = [Xt(1:3); Xt(4:6); basis];

    tstart = tend;
    tend = tstart + dt_sim;
    
    %% --- log ---  
    lent = length(t(2:end));
    tout = [tout;t(2:end)];
    Xout = [Xout;X(2:end,:)];
    Uout = [Uout;repmat(Ut',[lent,1])];
    Xdout = [Xdout;repmat(X_ref(:,ii)',[lent,1])];

    waitbar(ii/mpc_params.MAX_ITER,h_waitbar,'Calculating...');
end
F = findall(0,'type','figure','tag','TMWWaitbar');
delete(F);
fprintf('Calculation Complete!\n')
toc

mpc.t = tout;
mpc.X = Xout;
mpc.U = Uout;
mpc.X_ref = Xdout;