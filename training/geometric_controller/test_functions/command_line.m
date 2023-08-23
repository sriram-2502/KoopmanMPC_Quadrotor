function desired = command_line(t,param)

height = param.height;
R = param.R;
desired.x = R*[0.5 * t, 0, -height]';
desired.v = R'*[0.5 * 1, 0, 0]';
desired.x_2dot = [0, 0, 0]';
desired.x_3dot = [0, 0, 0]';
desired.x_4dot = [0, 0, 0]';

w = 2 * pi / 10;
desired.b1 = [cos(w * t), sin(w * t), 0]';
desired.b1_dot = w * [-sin(w * t), cos(w * t), 0]';
desired.b1_2dot = w^2 * [-cos(w * t), -sin(w * t), 0]';

end