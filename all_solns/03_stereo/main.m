clear all;
close all;

% Scaling down by a factor of 2, otherwise too slow.
left_img = imresize(imread('../data/left/000000.png'), 0.5);
right_img = imresize(imread('../data/right/000000.png'), 0.5);
K = load('../data/K.txt');
K(1:2, :) = K(1:2, :) / 2;

poses = load('../data/poses.txt');

% Given by the KITTI dataset:
baseline = 0.54;

% Carefully tuned by the TAs:
patch_radius = 5;
min_disp = 5;
max_disp = 50;
xlims = [7 20];
ylims = [-6 10];
zlims = [-5 5];

%% Parts 1, 2 and 4: Disparity on one image pair

tic;
disp_img = getDisparity(...
    left_img, right_img, patch_radius, min_disp, max_disp);
toc
figure(1);
imagesc(disp_img);
axis equal;
axis off;

%% Optional (only if fast enough): Disparity movie
figure(2);
for i = 0:99
    l = imresize(imread(sprintf('../data/left/%06d.png',i)), 0.5);
    r = imresize(imread(sprintf('../data/right/%06d.png',i)), 0.5);
    disp_img_i = getDisparity(...
        l, r, patch_radius, min_disp, max_disp);
    imagesc(disp_img_i);
    axis equal;
    axis off;
    pause(0.01);
end

%% Part 3: Create point cloud for first pair

[p_C_points, intensities] = disparityToPointCloud(...
    disp_img, K, baseline, left_img);
% From camera frame to world frame:
p_F_points = [0 -1 0; 0 0 -1; 1 0 0]^-1 * p_C_points(:, 1:10:end);

figure(3);
scatter3(p_F_points(1, :), p_F_points(2, :), p_F_points(3, :), ...
    20 * ones(1, length(p_F_points)), ...
    repmat(single(intensities(1:10:end))'/255, [1 3]), 'filled');
axis equal;
axis([0 30 ylims zlims]);
axis vis3d;
grid off;
xlabel('X');
ylabel('Y');
zlabel('Z');

%% Part 4: Accumulate point clouds over sequence and write PLY

all_points = [];
all_intensities = [];

% Shorten to get a point cloud faster.
image_range = 0:99;

h = waitbar(0, 'Accumulating point clouds...');

for i = image_range
    l = imresize(imread(sprintf('../data/left/%06d.png',i)), 0.5);
    r = imresize(imread(sprintf('../data/right/%06d.png',i)), 0.5);
    disp_img = getDisparity(...
        l, r, patch_radius, min_disp, max_disp);
    [p_C_points, intensities] = disparityToPointCloud(...
        disp_img, K, baseline, l);
    % From camera frame to world frame:
    R_C_frame = [0 -1 0; 0 0 -1; 1 0 0];
    p_F_points = R_C_frame^-1 * p_C_points;
    % Use only points within limits:
    filter = ...
        (p_F_points(1, :) > xlims(1)) & (p_F_points(1, :) < xlims(2)) & ...
        (p_F_points(2, :) > ylims(1)) & (p_F_points(2, :) < ylims(2)) & ...
        (p_F_points(3, :) > zlims(1)) & (p_F_points(3, :) < zlims(2));
    p_F_points = p_F_points(:, filter);
    intensities = intensities(:, filter);
    
    T_W_C = reshape(poses(i+1, :), 4, 3)';
    T_W_F = T_W_C * [R_C_frame zeros(3, 1); zeros(1, 3) 1];
    all_points = [all_points ...
        (T_W_F(1:3, 1:3) * p_F_points + T_W_F(1:3, end))];
    all_intensities = [all_intensities intensities];
    
    if 0
        figure(4);
        scatter3(all_points(1, :), all_points(2, :), all_points(3, :), ...
            20 * ones(1, length(all_points)), ...
            repmat(single(all_intensities)'/255, [1 3]), 'filled');
        axis equal;
        axis vis3d;
        grid off;
        pause(0.01);
    end
    
    waitbar(i/image_range(end), h);
end

close(h);

file = fopen('points.ply', 'w');
fprintf(file, ['ply\nformat ascii 1.0\nelement vertex %d\nproperty '...
    'double x\nproperty double y\nproperty double z\nproperty uchar red'...
    '\nproperty uchar green\nproperty uchar blue\nend_header\n'], ...
    length(all_points));
fclose(file);
dlmwrite('points.ply', ...
    [all_points' repmat(single(all_intensities)', [1 3])], ...
    '-append', 'delimiter', ' ');