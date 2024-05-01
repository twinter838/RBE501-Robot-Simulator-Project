function twist_inB = adjoint(V,T)
% twist_inB  Transforms a 6x1 screw axis by a given transformation matrox.
%   S = 6x1 matrix of robot screw axes
%   T= Transformation matrix between the 2 screw axes
%
% Rotational matrix
R=T(1:3,1:3);
% Translation vector
P=T(1:3,4);
% skew matrix for vector
skewP=[0 -P(3) P(2);
P(3) 0 -P(1);
-P(2) P(1) 0];
% translation in new reference
twist_inB=[R zeros(3,3);skewP*R R]*V;
end

