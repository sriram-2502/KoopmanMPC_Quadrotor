function desired = command_line(t,param)

height = param.height;
desired.x = [1, 1, -height]';
desired.v = [0, 0, 0]';
desired.x_2dot = [0, 0, 0]';
desired.x_3dot = [0, 0, 0]';
desired.x_4dot = [0, 0, 0]';

w = 0 * 2 * pi / 10;
desired.b1 = [cos(w), sin(w), 0]';
desired.b1_dot = w * [-sin(w), cos(w), 0]';
desired.b1_2dot = w^2 * [-cos(w), -sin(w), 0]';

end