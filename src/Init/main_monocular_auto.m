% Tabula rasa
clear all;
clc;
close all;

kitti_path = '/Users/mgrimm/Documents/Studium/9_Semester/VisionAlgoMobileRobotics/kitti';
malaga_path = '/Users/mgrimm/Documents/Studium/9_Semester/VisionAlgoMobileRobotics/malaga';
parking_path = '/Users/mgrimm/Documents/Studium/9_Semester/VisionAlgoMobileRobotics/parking';

%% Setup
ds = 2; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
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
if ds == 0
    initial_frame = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',0)]);
elseif ds == 1
    initial_frame = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(0).name]));
elseif ds == 2
    initial_frame = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',0)]));
else
    assert(false);
end
    
max_num_auto_frames = 15;
inliers = zeros(max_num_auto_frames, 1);
errors = zeros(max_num_auto_frames, 2);
costs = zeros(max_num_auto_frames, 2);
for i = 1:max_num_auto_frames
    if ds == 0
        current_frame = imread([kitti_path '/00/image_0/' ...
            sprintf('%06d.png',i)]);
    elseif ds == 1
        current_frame = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        current_frame = rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)]));
    else
        assert(false);
    end
    
    % Automatically choosing frames
    [img0, img1, kp_database, state, T_cw, descriptors_query, ...
        reprojection_errors, dist_epi_costs] = auto_init_frames(...
        initial_frame, current_frame, K);
    
    inliers(i,:) = size(state.matches_2d, 2);
    errors(i,:) = reprojection_errors;
    costs(i,:) = dist_epi_costs;
end

inliers
errors
costs
    
%% Monocular initialisation
%[state, T_cw] = monocular_initialisation(img0, img1, K);