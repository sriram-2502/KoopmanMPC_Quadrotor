function mpc = sim_MPC(EDMD,Z0,Z_ref,X_ref,params,U_ref_mpc,mass_change_flag,update_edmd_flag)
% --- logging ---
dt_sim = params.simTimeStep;
N = params.predHorizon;
tstart = 0;
tend = dt_sim;
quad_params = sys_params;
quad_params.flag='mpc';
[tout,Xout,Uout,Xdout] = deal([]);

%% --- simulation ----
h_waitbar = waitbar(0,'Calculating...');
tic
Z = Z0;
mpc = [];
if update_edmd_flag
    mpc.buffer_size = 500;
    mpc.X_buffer = [];
    mpc.U_buffer = [];
    mpc.X_ref_buffer = [];
end

for ii = 1:params.MAX_ITER
    % --- time vector ---
    %t_ = dt_sim * (ii-1) + params.Tmpc * (0:N-1);

    %% --- MPC ----      
    %form QP using explicit matrices
    z_ref = Z_ref(:,ii:ii+N-1);

    if(params.use_casadi)
        [zval] = casadi_MPC(EDMD,Z,Z_ref,N,params);
    else
        tic
        [f, G, A, b, EDMD] = get_QP(EDMD,Z,z_ref,N,mpc,update_edmd_flag);
        if any([any(isinf(f)), any(isnan(f)), any(isinf(G)), any(isnan(G)), any(isinf(A)), any(isnan(A)), any(isinf(b)), any(isnan(b))])
            fprintf('MATRICES ARE NOT REAL VALUED')
        elseif isreal(f) && isreal(G) && isreal(A) && isreal(b)
            % solve QP using quadprog
            options = optimoptions('quadprog','MaxIterations',1e4);
            [zval,f_val] = quadprog(G,f,[],[],[],[],[],[],[],options);
            toc
        end
    end
    
    if EDMD.update_flag
        zz = Z0;
        for eval_idx=1:size(U_ref_mpc,2)
            zz_next = EDMD.A*zz(:,end) + EDMD.B*U_ref_mpc(:,eval_idx);
            zz = [zz, zz_next];
        end
        xx_pred = EDMD.C*zz(:,2:end);
        xx_true = EDMD.C*Z_ref;
        eval_traj_len = size(xx_true,2);
        show_plot = true;
        RMSE = rmse(xx_pred,xx_true,eval_traj_len,show_plot)

        fprintf('EDMD evaluated in MPC after online update')
    end
    % apply control from the quadprog
    Ut = zval(1:4);

    %% --- simulate without any external disturbances ---
    % parse true states from lifted states
    Xt = EDMD.C*Z;

    % Geometric dynamcis for actual trajectory
    [t, X_geometric] = ode45(@(t, XR) eom_mpc(t, XR, Ut, params, mass_change_flag), ...
        [tstart,tend], Xt', ...
    odeset('RelTol', 1e-6, 'AbsTol', 1e-6));
 
    
    %% --- update ---
    Xt = X_geometric(end,:)'; % Xt for EDMD (in body frame)

    if update_edmd_flag
        % add to buffer for online update
        if(size(mpc.X_buffer,2)>=mpc.buffer_size)
            mpc.X_buffer = [mpc.X_buffer(:,2:end), Xt];
            mpc.X_ref_buffer = [mpc.X_ref_buffer(:,2:end), X_ref(:,ii)];
            mpc.U_buffer = [mpc.U_buffer(:,2:end), Ut];
        else
            mpc.X_buffer = [mpc.X_buffer, Xt];
            mpc.X_ref_buffer = [mpc.X_ref_buffer, X_ref(:,ii)];
            mpc.U_buffer = [mpc.U_buffer, Ut];
        end
    end

    % get lifted states at t=0
    Z = get_basis(Xt,EDMD.n_basis);
    
    tstart = tend;
    tend = tstart + dt_sim;
    
    %% --- log ---  
    lent = length(t(2:end));
    tout = [tout;t(2:end)];
    Xout = [Xout;X_geometric(2:end,:)];
    Uout = [Uout;repmat(Ut',[lent,1])];
    Xdout = [Xdout;repmat(X_ref(:,ii)',[lent,1])];

    waitbar(ii/params.MAX_ITER,h_waitbar,'Calculating...');
end
F = findall(0,'type','figure','tag','TMWWaitbar');
delete(F);
fprintf('Calculation Complete!\n')
toc

mpc.t = tout;
mpc.X = Xout;
mpc.U = Uout;
mpc.X_ref = Xdout;
mpc.EDMD_updated = EDMD.update_flag;