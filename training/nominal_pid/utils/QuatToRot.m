function bRw = QuatToRot(q)
%QuatToRot Converts a Quaternion to Rotation matrix
%   written by Daniel Mellinger
% Sriram: returns world_to_body rotation matrix bRw
% normalize q
q = q./sqrt(sum(q.^2));

qahat(1,2) = -q(4);
qahat(1,3) = q(3);
qahat(2,3) = -q(2);
qahat(2,1) = q(4);
qahat(3,1) = -q(3);
qahat(3,2) = q(2);

bRw = eye(3) + 2*qahat*qahat + 2*q(1)*qahat;

end