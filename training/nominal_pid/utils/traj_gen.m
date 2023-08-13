function traj_params = traj_gen(traj_params,parameter)
% function to generate waypoints for PID control
% inputs
% traj_prams                : structure of parameters
% parameter                 : scalar value for height or raidus

if strcmp(traj_params.traj_type,'hover')
    % get traj for hover
    traj_params.traj_type = 'hover';
    height = parameter;
    % waypoints [p1,p2,p3,...] where p1 = [x;y;z]
    waypoints = zeros(3);
    waypoints(end,:) = linspace(0,height,3);
    traj_params.waypoints = waypoints;

elseif strcmp(traj_params.traj_type,'circle')
    % get traj for circle
    traj_params.traj_type = 'circle';
    radius = parameter;
    traj_params.direction = 1; % direction = +1/-1 for anticlockwise/clockwise
    traj_params.center = [0; 0]; %% TODO - make it as user input

    % waypoints [p1,p2,p3,...] where p1 = [x;y;z]      
    precision = 0.1;
    thetas = 0:precision:2*pi;
    % parametric expression for circle in XY-plane
    x_data = radius*cos(thetas) + traj_params.center(1);
    y_data = traj_params.direction*radius*sin(thetas) + traj_params.center(2);
    waypoints = [x_data; y_data; ones(size(x_data))];
    traj_params.waypoints = waypoints;

elseif strcmp(traj_params.traj_type,'line')
    % % get traj for slanted line
    traj_params.traj_type = 'line';
    traj_params.endPoints = 0.5*[[0;0;0],[1;1;1]];
    traj_params.n_traj = size(traj_params.endPoints,2)/2; 

%%%%%%%%%%%%% TODO: add traj for random control inputs %%%%%%%%%%%%%%%%%%%
% elseif strcmp(traj_params.traj_type,'random')
% get traj for random control input
% n_control = 100; % number of random controls to apply
% t_traj = 0:dt:t_span; % traj length to simulate (s)
% show_plot = true; 
% flag = 'train';
% [X, U, X1, X2, U1] = get_rnd_trajectories(X0,n_control,t_traj,show_plot,flag);

end

end