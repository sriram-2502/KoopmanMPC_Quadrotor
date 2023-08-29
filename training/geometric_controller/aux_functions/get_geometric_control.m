function usave = get_geometric_control(tsave, xsave, trajhandle, controlhandle, k, param)
% inputs: 
% tsave = vector of time
% x_save = vector of states
% trajhandle = function handle to generate desired waypoints
% controlhandle = function handle to get the required control inputs
% [xdot; vdot; Wdot; reshape(Rdot,9,1); ei_dot; eI_dot] - from geometric control;
% outputs:
% usave = vector of control inputs corresponding to the states

usave = [];

for i=1:length(tsave) 
        wRb = xsave(i,10:18);
        param.R = reshape(wRb,[3,3]);
        % get control outputs for edmd
        desired = trajhandle(tsave(i),param);
        [F, M] = controlhandle(xsave(i,:)', desired, k, param);
        % assemble u
        u = [F; M];
        usave = [usave, u(:)];
end

usave = usave';
