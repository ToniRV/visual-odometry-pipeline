close all;
clear all;

K = load('../data/K.txt');

p_W_corners = 0.01 * load('../data/p_W_corners.txt');

num_corners = length(p_W_corners);

% Load the 2D projected points (detected on the undistorted image)
all_pts2d = load('../data/detected_corners.txt');

num_images = 210;
translations = zeros(num_images,3);
quaternions = zeros(num_images,4);
for img_index=1:num_images

    pts2d = reshape(all_pts2d(img_index,:), 2, 12)';

    % Now that we have the 2D <-> 3D correspondences (pts2d+normalized <-> p_W_corners),
    % let's find the camera pose with respect to the world using DLT
    M = estimatePoseDLT(pts2d, p_W_corners, K);
    
    R_C_W = M(1:3,1:3);
    t_C_W = M(1:3,4);
    rotMat = R_C_W';
    quaternions(img_index,:) = rotMatrix2Quat(rotMat);
    translations(img_index,:) = -R_C_W' * t_C_W;

end

%% Generate video of the camera motion

fps = 30;
plotTrajectory3D(fps, translations', quaternions', p_W_corners');
