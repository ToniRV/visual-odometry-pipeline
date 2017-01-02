%% Tabula rasa
clear all;
clc;
close all;

% Add required files to MATLAB search path:
addpath('../2d2d_correspondences');

% Insert the paths to the corresponding frame sets:
kitti_path='/Users/amadeus/Documents/MATLAB/Frame_Sets/Kitti';
malaga_path='/Users/amadeus/Documents/MATLAB/Frame_Sets/Malaga_07';
parking_path='/Users/amadeus/Documents/MATLAB/Frame_Sets/Parking';

% At this point, the camera is initialized by stereo initialization using (only) the KITTI
% frame set.

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: Parking

if ds == 0
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    % Given by the KITTI dataset:
    baseline = 0.54;

    elseif ds == 1
        assert(exist('malaga_path', 'var') ~= 0);
        images = dir([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        last_frame = length(left_images);
        K = [621.18428 0 404.0076
            0 621.18428 309.05989
            0 0 1];

    elseif ds == 2
        % Path containing images, depths and all...
        assert(exist('parking_path', 'var') ~= 0);
        last_frame = 598;
        K = load([parking_path '/K.txt']);
        ground_truth = load([parking_path '/poses.txt']);
        ground_truth = ground_truth(:, [end-8 end]);
    
    else
        assert(false);
end

%% Bootstrap
% Need to set bootstrap_frames
% For stereo initialization: Choose left and right images taken by the
% stereo camera at the same time
bootstrap_frames = [0 0];

if ds == 0
    % Left image
    img_left = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    % Right image
    img_right = imread([kitti_path '/00/image_1/' ...
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

%% Triangulation
% The triangulation output varies depending on the used algorithm
% It seems like the one given in ex5 is doing something different at
% certain points compared with the other two.

[points_2D_ex_5_triangulation, points_3D_ex_5_triangulation] = ...
    stereo_initialisation(img_left, img_right, K, baseline, 'ex_5_triangulation');
[points_2D_matlab_triangulation, points_3D_matlab_triangulation] = ...
    stereo_initialisation(img_left, img_right, K, baseline, 'matlab_triangulation');
[points_2D_disparity_triangulation, points_3D_disparity_triangulation] = ...
    stereo_initialisation(img_left, img_right, K, baseline, 'disparity_triangulation');

%% Continuous Operation
range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
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
    
    prev_img = image;
end