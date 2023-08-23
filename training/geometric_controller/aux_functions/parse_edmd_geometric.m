function [x_edmd, usave] = parse_edmd_geometric(tsave, xsave, trajhandle, controlhandle, k, param)
%function [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x)
% inputs: 
% tsave = vector of time
% x_save = vector of states
% [xdot; vdot; Wdot; reshape(Rdot,9,1); ei_dot; eI_dot] - from geometric control;

parse_control = true;
if nargin<3
    parse_control = false;
    controlhandle=[];
    trajhandle = [];
    param = [];
end

% [tsave, xsave] = ode45(f_handle, timeint, x0);
x_edmd = [];
usave = [];

for i=1:length(tsave) 
    % get states for edmd
    x = xsave(i,1:3);
    dx_b = xsave(i,4:6); 
    wRb = xsave(i,10:18);
    wb = xsave(i,7:9);

    if parse_control
        param.R = reshape(wRb,[3,3]);
        % get control outputs for edmd
        desired = trajhandle(tsave(i),param);
        [F, M] = controlhandle(xsave(i,:)', desired, k, param);
        % assemble u
        u = [F; M];
        usave = [usave, u(:)];
    end

    

    % x_edmd = [x, xdot_body, R(:), wb] -- 18 states    
    x_edmd = [x_edmd; [x,dx_b,wRb,wb]];
end

usave = usave';
