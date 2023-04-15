function vector = vee_map(matrix)
% Inputs
% matrix:   n x n skew-symmetric matrix 
% Outputs
% vector:   3D vector extracted from the skew-symmetric matrix


vector = zeros(3,1);

vector(1) = -matrix(2,3);
vector(2) =  matrix(1,3);
vector(3) = -matrix(1,2);