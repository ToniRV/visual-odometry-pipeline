function [projected_points] = projectPoints(points_3d, K, D)
% Projects 3d points to the image plane (3xN), given the camera matrix (3x3) and
% distortion coefficients (4x1).

% if distortion vector D is missing, assume zero distortion
if nargin <= 2
    D = zeros(4,1);
end

num_points = size(points_3d,2);

% get normalized coordinates
xp = points_3d(1,:) ./ points_3d(3,:);
yp = points_3d(2,:) ./ points_3d(3,:);

% apply distortion
x_d = distortPoints([xp;yp],D);
xpp = x_d(1,:); ypp = x_d(2,:);

% convert to pixel coordinates
projected_points = K * [xpp; ypp; ones(1, num_points)];
projected_points = projected_points(1:2, :);

end

