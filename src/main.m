%% Tabula rasa
clear all;
close all;
clc;

addpath('2d2d_correspondences');
addpath('src/Init');

kitti_path='/home/tonirv/Downloads/kitti';
malaga_path='/home/tonirv/Downloads/';
parking_path='/home/tonirv/Documents/Vision Algorithms for Mobile Robotics/VO - Project/parking';
%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking
stereo = 1;

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    % Try  to find better bootstrap frames for initialisation
    bootstrap_frames = [0, 1];
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    % Try  to find better bootstrap frames for initialisation
    bootstrap_frames = [1, 2]; % This guy should be >0 because of the way the images are parsed I think.
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
    % Try  to find better bootstrap frames for initialisation
    bootstrap_frames = [0, 1]; 
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
if stereo
    sprintf('Stereo initialisation, bootstrapping images taken from kitti database');
    % Given by the KITTI dataset:
    baseline = 0.54;
else
    if ds == 0
        img0 = imread([kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frames(1))]);
        img1 = imread([kitti_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frames(2))]);
    elseif ds == 1
        img0 = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frames(1)).name]));
        img1 = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frames(2)).name]));
    elseif ds == 2
        img0 = rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
        img1 = rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
    else
        assert(false);
    end
end

%% Bootstrap stereo, remove when monocular init is ready
stereo_bootstrap_frames = [0 0];
% left image
    img_left = imread([kitti_path '/00/image_left/' ...
        sprintf('%06d.png',stereo_bootstrap_frames(1))]);
% right image
    img_right = imread([kitti_path '/00/image_right/' ...
        sprintf('%06d.png',stereo_bootstrap_frames(2))]);

[ points_2D_matlab_triangulation, points_3D_matlab_triangulation ] = ...
    stereo_initialisation( img_left, img_right , K, baseline, 'matlab_triangulation');

%% Continuous operation
p_W_landmarks = points_3D_matlab_triangulation;
keypoints = points_2D_matlab_triangulation;
range = (bootstrap_frames(2)+1):last_frame;
figure(5);
subplot(1, 3, 3);
scatter3(p_W_landmarks(1, :), p_W_landmarks(2, :), p_W_landmarks(3, :), 5);
set(gcf, 'GraphicsSmoothing', 'on');
view(0,0);
axis equal;
axis vis3d;
axis([-15 10 -10 5 -1 40]);
prev_img = img_left;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        % should we use image_left or right? before it was image_0 but
        % ain't nothing like that in the given folder
        image = imread([kitti_path '/00/image_left/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    % Makes sure that plots refresh.    
    pause(0.01);
          tic;
            [R_C_W, t_C_W, query_keypoints, all_matches, inlier_mask] = ...
            ransacLocalization(image, prev_img,  keypoints, ...
            p_W_landmarks, K);
            toc
            matched_query_keypoints = query_keypoints(:, all_matches > 0);
            corresponding_matches = all_matches(all_matches > 0);

            % Distinguish success from failure.
            if (numel(R_C_W) > 0)
                subplot(1, 3, 3);
                plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2);
                disp(['Frame ' num2str(i) ' localized with ' ...
                    num2str(nnz(inlier_mask)) ' inliers!']);
                view(0,0);
            else
                disp(['Frame ' num2str(i) ' failed to localize!']);
            end

            subplot(1, 3, [1 2]);
            imshow(image);
            hold on;
            plot(matched_query_keypoints(2, (1-inlier_mask)>0), ...
                matched_query_keypoints(1, (1-inlier_mask)>0), 'rx', 'Linewidth', 2);
            if (nnz(inlier_mask) > 0)
                plot(matched_query_keypoints(2, (inlier_mask)>0), ...
                    matched_query_keypoints(1, (inlier_mask)>0), 'gx', 'Linewidth', 2);
            end
            plotMatches(corresponding_matches(inlier_mask>0), ...
                matched_query_keypoints(:, inlier_mask>0), ...
                keypoints);
            hold off;
            title('Inlier and outlier matches');
            pause(0.01);
    prev_img = image;
end
