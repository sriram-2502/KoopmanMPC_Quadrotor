function mpc = sim_MPC(EDMD,Z0,Z_ref,X_ref,n_basis,mpc_params)
% --- logging ---
dt_sim = mpc_params.simTimeStep;
N = mpc_params.predHorizon;
tstart = 0;
tend = dt_sim;
quad_params = sys_params;
quad_params.flag='mpc';
[tout,Xout,Uout,Xdout] = deal([]);

%% --- simulation ----
h_waitbar = waitbar(0,'Calculating...');
tic
Z = Z0;
mpc.X_buffer = [];
mpc.U_buffer = [];
mpc.X_ref_buffer = [];
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
        [f, G, A, b, EDMD] = get_QP(EDMD,Z,z_ref,N,mpc,mpc_params);
        % solve QP using quadprog     
        options = optimoptions('quadprog','MaxIterations',1e4);
        [zval,f_val] = quadprog(G,f,A,b,[],[],[],[],[],options);
        toc
    end

    Ut = zval(1:4)

    %% --- simulate without any external disturbances ---
    % parse true states from lifted states
    Xt = EDMD.C*Z;
    x = Xt(1:3); dx = Xt(4:6); 
    q = Xt(7:10);
    wb = Xt(11:13); 

    % use pid dynamics for ode45
    % Xt = [x; dx in world frame; quartornions; body frame angular
    % velocities]
    Xt = [x;dx;q;wb];    
     
    [t,X_pid] = ode45(@(t,s) quadEOM_readonly(t, s, Ut(1), Ut(2:end), quad_params),[tstart,tend],Xt);
    X = parse_edmd_pid(t,X_pid);
    
    %% --- update ---
    Xt = X(end,:)'; % Xt for EDMD (in body frame)

    % addd to buffer for online update
    buffer_size = 100; % buffer size
    if(size(mpc.X_buffer,2)>=buffer_size)
        mpc.X_buffer = [mpc.X_buffer(:,2:end), Xt];
        mpc.X_ref_buffer = [mpc.X_ref_buffer(:,2:end), X_ref(:,ii)];
        mpc.U_buffer = [mpc.U_buffer(:,2:end), Ut];
    else
        mpc.X_buffer = [mpc.X_buffer, Xt];
        mpc.X_ref_buffer = [mpc.X_ref_buffer, X_ref(:,ii)];
        mpc.U_buffer = [mpc.U_buffer, Ut];
    end

    % get lifted states at t=0
    Z = get_basis(Xt,EDMD.n_basis);
    
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