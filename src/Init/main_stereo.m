%Tabula rasa
clear all;
clc;
close all;

addpath('../2d2d_correspondences');

kitti_path='/home/tonirv/Downloads/kitti';
malaga_path='/home/tonirv/Downloads/';
parking_path='/home/tonirv/Documents/Vision Algorithms for Mobile Robotics/VO - Project/parking';
%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    % Given by the KITTI dataset:
    baseline = 0.54;

    % Assuming identical K for each camera, and world frame identical to
    % left camera frame.
    M_left = K * [eye(3), [0; 0; 0]];
    M_right = K * [eye(3), [-baseline; 0; 0]]; % TODO check that this is correct, it actually depends on the Kitti coords.
                                                                                % According to the coords in this paper http://www.mrt.kit.edu/z/publ/download/2013/GeigerAl2013IJRR.pdf
elseif ds == 1
    % Path containing the many files of Malaga 7.
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
% need to set bootstrap_frames
% left and right images to take, obviously both have to be at the same time
% since we are doing stereo initialisation.
bootstrap_frames = [0 0];

if ds == 0
    % left image
    img_left = imread([kitti_path '/00/image_left/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    % right image
    img_right = imread([kitti_path '/00/image_right/' ...
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

tic;
% !!!!!!!!!!!!!!!!WARNING keypoints are in (row, col) coordinates of the image which might differ from
% (u, v) coordinates, depending on whether u representes rows or columns!!!!!!!!!!!!!!!!!!!!!!
[keypoints_left, keypoints_right] = correspondences_2d2d(img_left, img_right);
fprintf('It took %ds to compute correspondences \n', toc);

%Matlab convention, I have to flip keypoints :O
figure(10); showMatchedFeatures(img_left, img_right, flipud(keypoints_left)', flipud(keypoints_right)', 'montage');

% Homogeneous coordinates of 2d-2d correspondences
homo_keypoints_left = [keypoints_left ; ones(1, size(keypoints_left,2))]; % TODO not sure if this zeros should instead 
homo_keypoints_right = [keypoints_right ; ones(1, size(keypoints_right,2))]; %be a SCALE factor or something of the kind

% Given by the KITTI dataset:
baseline = 0.54;

% Assuming identical K for each camera
M_left = K * [eye(3), [0; 0; 0]];
M_right = K * [eye(3), [-baseline; 0; 0]]; % TODO check that this is correct, it actually depends on the Kitti coords.
                                                                                % According to the coords in this paper http://www.mrt.kit.edu/z/publ/download/2013/GeigerAl2013IJRR.pdf
                                                                                
toc


