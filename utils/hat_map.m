function matrix = hat_map(vector)
% Inputs
% vector:   3D vector with elements [v1 v2 v3]
% Outputs
% matrix:   skew summetrix matrix formed with the given vector

matrix=[ 0            -vector(3)       vector(2);
         vector(3)     0              -vector(1);
        -vector(2)     vector(1)       0];

end