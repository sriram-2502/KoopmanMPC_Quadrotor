function [t_out, s_out, xtraj_edmd, u_out] = simulation_3d(trajhandle, controlhandle, train_edmd, show_plot,flag)
% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** QUADROTOR SIMULATION *****************

% *********** YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW **********

addpath('utils');

% real-time
real_time = false;

% max time
% max_time = 50;
max_time = 500;

% parameters for simulation
params = sys_params;
params.flag=flag;

video = false;
if video
  video_writer = VideoWriter('helice.avi', 'Uncompressed AVI');
  open(video_writer);
end

%% **************************** FIGURES *****************************
if(show_plot)
    disp('Initializing figures...');
    h_fig = figure;
    h_3d = gca;
    axis equal
    grid on
    view(3);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
    quadcolors = lines(1);
    set(gcf,'Renderer','OpenGL')
end

%% *********************** INITIAL CONDITIONS ***********************
disp('Setting initial conditions...');
tstep    = 0.001; % this determines the time step at which the solution is given
cstep    = 0.05; % image capture time interval
max_iter = max_time/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
err = []; % runtime errors

% Get start and stop position
des_start = trajhandle(0, []);
des_stop  = trajhandle(inf, []);
stop_pos  = des_stop.pos;
x0      = init_state(des_start.pos, 0); 
xtraj   = zeros(max_iter*nstep, length(x0));
if train_edmd
    x0_edmd = init_state_edmd(des_start.pos, 0);
    xtraj_edmd   = zeros(max_iter*nstep, length(x0_edmd)); 
end

u0 = [0;0;0;0];
utraj   = zeros(max_iter*nstep, length(u0));
x_init  = x0;
ttraj   = zeros(max_iter*nstep, 1);
x       = x_init;        % state
pos_tol = 0.01;
vel_tol = 0.01;

% get normalized white noise for each state
params.noise = 0.05*randn([max_iter,size(x0)]);

%% ************************* RUN SIMULATION *************************
disp('Simulation Running....');
% Main loop
for iter = 1:max_iter

    timeint = time:tstep:time+cstep;

    tic;

    % Initialize quad plot
    if(show_plot)
        if iter == 1
            QP = QuadPlot(1, x_init, 0.1, 0.04, quadcolors(1,:), max_iter, h_3d);
            current_state = stateToQd(x);
            desired_state = trajhandle(time, current_state);
            QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time);
            h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
        end
    end

    % Run simulation (PID)
    params.noise_idx = iter;
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x);
    
    if train_edmd
    % parse x_pid to get x_EDMD and usave (to use in EDMD training and MPC) 
        [x_edmd, usave] = parse_edmd_pid(tsave, xsave, controlhandle, trajhandle, params);
    else
    % use xsave to get usave 
        usave = zeros(length(tsave),length(u0));
    end
    x    = xsave(end, :)';

    % Save to traj
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
    if train_edmd
        xtraj_edmd((iter-1)*nstep+1:iter*nstep,:) = x_edmd(1:end-1,:);
    end
    utraj((iter-1)*nstep+1:iter*nstep,:) = usave(1:end-1,:);
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

    % Update quad plot
    if(show_plot)
        current_state = stateToQd(x);
        desired_state = trajhandle(time + cstep, current_state);
        QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time + cstep);
        set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    end

    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*500)
        err = 'Ode45 Unstable';
        break;
    end
  if video
    writeVideo(video_writer, getframe(h_fig));
  end
    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x, time, stop_pos, pos_tol, vel_tol, max_time)
        break
    end
end

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
xtraj = xtraj(1:iter*nstep,:);
if train_edmd
    xtraj_edmd = xtraj_edmd(1:iter*nstep,:);
else
    xtraj_edmd = [];
end

utraj = utraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);

% Truncate saved variables
if(show_plot)
    QP.TruncateHist();
end
% Plot position
% h_pos = figure('Name', ['Quad position']);
% plot_state(h_pos, QP.state_hist(1:3,:), QP.time_hist, 'pos', 'vic');
% plot_state(h_pos, QP.state_des_hist(1:3,:), QP.time_hist, 'pos', 'des');
% % Plot velocity
% h_vel = figure('Name', ['Quad velocity']);
% plot_state(h_vel, QP.state_hist(4:6,:), QP.time_hist, 'vel', 'vic');
% plot_state(h_vel, QP.state_des_hist(4:6,:), QP.time_hist, 'vel', 'des');

if(~isempty(err))
    error(err);
end

disp('finished.')
if video
  close(video_writer);
end
t_out = ttraj;
s_out = xtraj;
u_out = utraj;

end
