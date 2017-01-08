function p_W_estimate_aligned = alignEstimateToGroundTruth(...
    p_W_GT, p_W_estimate)
% Align the VO trajectory estimate to the ground truth using NLLS
% optimization. 
%
% INPUT:
%
%    *   p_W_GT: Ground truth trajectory; 3xN matrix; N = Number of frames/
%        estimated poses; frame origins only. 
%    *   p_W_estimate: Estimated trajectory by the VO pipeline; 3xN matrix
%
% OUTPUT:
%
%    *   p_W_estimate_aligned: Estimated trajectory aligned with the ground
%        truth
%
% Define initial guesses for transform (twist vector) and scale
% factor:
twist_guess = HomogMatrix2twist(eye(4));
scale_guess = 1;

% lsqnonlin optimizes x to minimize the error_terms.
x = [twist_guess; scale_guess];
error_terms = @(x) alignError(x, p_W_GT, p_W_estimate);
options = optimoptions(@lsqnonlin, 'Display', 'iter');
% Optimal similarity transformation parameters (including scale factor):
x_optim = lsqnonlin(error_terms, x, [], [], options);

% Convert twist vector to homogeneous transformation matrix:
T_old_new = twist2HomogMatrix(x_optim(1:6));
scale_old_new = x_optim(7);

num_frames = size(p_W_estimate, 2);
% Apply similarity transformation to obtain the aligned trajectory:
p_W_estimate_aligned = scale_old_new * T_old_new(1:3, 1:3) * p_W_estimate ...
    + repmat(T_old_new(1:3, 4), [1 num_frames]);

end

