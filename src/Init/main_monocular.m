% Tabula rasa
clear all;
clc;
close all;

% Insert the paths to the corresponding frame sets:
kitti_path = '/Users/mgrimm/Documents/Studium/9_Semester/VisionAlgoMobileRobotics/kitti';
malaga_path = '/Users/mgrimm/Documents/Studium/9_Semester/VisionAlgoMobileRobotics/malaga';
parking_path = '/Users/mgrimm/Documents/Studium/9_Semester/VisionAlgoMobileRobotics/parking';

%% Setup
ds = 2; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
    % Given by the KITTI dataset:
    % baseline = 0.54;
    
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
if ds == 0
    bootstrap_frames = [1 3]; % Frames suggested in the project description
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
    
elseif ds == 1
    bootstrap_frames = [0 2];
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
    
elseif ds == 2
    bootstrap_frames = [0 2]; % [1 3]
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

%% Monocular initialisation
[state, T_cw] = monocular_initialisation(img0, img1, K);