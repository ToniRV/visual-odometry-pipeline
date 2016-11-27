function M = estimatePoseDLT(p, P, K)
% Estimates the pose of a camera using a set of 2D-3D correspondences and a
% given camera matrix
%
% p: [nx2] vector containing the undistorted coordinates of the 2D points
% P: [nx3] vector containing the 3D point positions
% K: [3x3] camera matrix
%
% M: [3x4] projection matrix under the form M=[R|t] where R is a rotation
%    matrix. M encodes the transformation that maps points from the world
%    frame to the camera frame

% Convert 2D points to normalized coordinates
p_normalized = (K \ [p ones(length(p),1)]')';

% Build the measurement matrix Q
num_corners = length(p_normalized);
Q = zeros(2*num_corners, 12);

for i=1:num_corners
    u = p_normalized(i,1);
    v = p_normalized(i,2);
    
    Q(2*i-1,1:3) = P(i,:);
    Q(2*i-1,4) = 1;
    Q(2*i-1,9:12) = -u * [P(i,:) 1];
    
    Q(2*i,5:7) = P(i,:);
    Q(2*i,8) = 1;
    Q(2*i,9:12) = -v * [P(i,:) 1];
end

% Solve for Q.M = 0 subject to the constraint ||M||=1
[~,~,V] = svd(Q);
M = V(:,end);

M = reshape(M, 4, 3)';

%% Extract [R|t] with the correct scale from M ~ [R|t]

if det(M(:,1:3)) < 0
    M = -M;
end

R = M(:,1:3);

% Find the closest orthogonal matrix to R
% https://en.wikipedia.org/wiki/Orthogonal_Procrustes_problem
[U,~,V] = svd(R);
R_tilde = U*V';

% Normalization scheme using the Frobenius norm:
% recover the unknown scale using the fact that R_corr is a true rotation matrix
alpha = norm(R_tilde, 'fro')/norm(R, 'fro');

% Build M with the corrected rotation and scale
M = [R_tilde alpha * M(:,4)];


end

