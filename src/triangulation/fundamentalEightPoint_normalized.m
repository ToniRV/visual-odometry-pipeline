
function F = fundamentalEightPoint_normalized(p1, p2)
% Estimates the Fundamental matrix F given keypoint correspondences, and
% the intrinsic parameter matrix K.
%
% INPUT: Keypoint correspondences
%         --> p1(3,N): Homogeneous coordinates of 2D keypoints in image 1
%         --> p2(3,N): Homogeneous coordinates of 2D keypoints in image 2
%
% OUTPUT: Fundamental matrix F; 3x3 matrix
%

% Normalize each set of keypoints so that centroid of each set is at the 
% origin and mean distance from origin is sqrt(2).
[x1_nh,T1] = normalise2dpts(p1);
[x2_nh,T2] = normalise2dpts(p2);

% Apply the 8-point algorithm to estimate the Fundamental matrix
F = fundamentalEightPoint(x1_nh,x2_nh);

% Undo the normalization to obtain F for the unnormalized keypoints
F = (T2.') * F * T1;

end
