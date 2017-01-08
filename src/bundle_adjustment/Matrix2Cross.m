% MATRIX2CROSS  Compute 3D vector corresponding to an antisymmetric matrix
%
% Computes the 3D vector x corresponding to an antisymmetric matrix M such
% that M*y = cross(x,y) for all 3D vectors y.
%
% Input: 
%   - M(3,3) : antisymmetric matrix
%
% Output: 
%   - x(3,1) : column vector
%
% See also CROSS2MATRIX

function x = Matrix2Cross(M)

x = [-M(2,3); M(1,3); -M(1,2)];

end