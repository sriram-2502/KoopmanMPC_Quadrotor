function vector = vectorize(matrix)

% Inputs
% matrix:   n x n matrix of elements with col c1 c2 ... cn
% Outputs
% vector:   column wise vector of elements [c1;c2;...;cn]

vector = reshape(matrix,[numel(matrix),1]);

end