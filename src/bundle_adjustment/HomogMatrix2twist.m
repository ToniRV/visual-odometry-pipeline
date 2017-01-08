function twist = HomogMatrix2twist(H)

% HomogMatrix2twist Convert 4x4 homogeneous matrix to twist coordinates
%
% Input:
% -H(4,4): Euclidean transformation matrix (rigid body motion)
%
% Output:
% -twist(6,1): twist coordinates. Stack linear and angular parts [v;w]
%
% Observe that the same H might be represented by different twist vectors
% Here, twist(4:6) is a rotation vector with norm in [0,pi]

se_matrix = logm(H);

% careful for rotations of pi; the top 3x3 submatrix of the returned
% se_matrix by logm is not skew-symmetric (bad).

v = se_matrix(1:3,4);

w = Matrix2Cross(se_matrix(1:3,1:3));

twist = [v; w];

end