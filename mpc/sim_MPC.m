function mpc = sim_MPC(EDMD,Z0,Z_ref,X_ref,params)
% --- logging ---
dt_sim = params.simTimeStep;
N = params.predHorizon;
tstart = 0;
tend = dt_sim;

[tout,Xout,Uout,Xdout] = deal([]);

%% --- simulation ----
h_waitbar = waitbar(0,'Calculating...');
tic
Z = Z0;
for ii = 1:params.MAX_ITER
    % --- time vector ---
    %t_ = dt_sim * (ii-1) + params.Tmpc * (0:N-1);

    %% --- MPC ----      
    %form QP using explicit matrices
    z_ref = Z_ref(:,ii:ii+N-1);

    if(params.use_casadi)
        [zval] = casadi_MPC(EDMD,Z,Z_ref,N,params);
    else
        [f, G, A, b] = get_QP(EDMD,Z,z_ref,N,params);
        % solve QP using quadprog     
        [zval] = quadprog(G,f,[],[],[],[],[],[]);
    end

    Ut = zval(1:4);

    %% --- simulate without any external disturbances ---
    %parse true states from lifted states
    Xt = EDMD.C*Z;
    x = Xt(1:3); dx = Xt(4:6); 
    R = reshape(Xt(7:15),[3,3])';
    wb_hat = reshape(Xt(16:24),[3,3]); % body frame
    wb = vee_map(wb_hat');
    Xt = [x;dx;R(:);wb;];  
    [t,X] = ode45(@(t,X)dynamics_SRB(t,X,Ut,params),[tstart,tend],Xt);
    
    %% --- update ---
    Xt = X(end,:)';
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

    waitbar(ii/params.MAX_ITER,h_waitbar,'Calculating...');
end
close(h_waitbar)
fprintf('Calculation Complete!\n')
toc

mpc.t = tout;
mpc.X = Xout;
mpc.U = Uout;
mpc.X_ref = Xdout;