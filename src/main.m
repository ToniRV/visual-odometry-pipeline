%% Tabula rasa
clear all;
close all;
%clc;

% REMOVE
rng(1);

addpath('2d2d_correspondences');
addpath('Init');
% Replace the following with the path to your keypoint matcher code:
addpath('../../00_camera_projection/code');

% Set to true if you want to perform bundle adjustment:
BA = true;

kitti_path='/Users/amadeus/Documents/MATLAB/Frame_Sets/Kitti';
malaga_path='/Users/amadeus/Documents/MATLAB/Frame_Sets/Malaga_07';
parking_path='/Users/amadeus/Documents/MATLAB/Frame_Sets/Parking';

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
    img_left = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',stereo_bootstrap_frames(1))]);
% right image
    img_right = imread([kitti_path '/00/image_1/' ...
        sprintf('%06d.png',stereo_bootstrap_frames(2))]);

[ points_2D_matlab_triangulation, points_3D_matlab_triangulation ] = ...
    stereo_initialisation( img_left, img_right , K, baseline, 'matlab_triangulation');

%% Continuous operation
% p_W_landmarks = points_3D_matlab_triangulation;
% keypoints = points_2D_matlab_triangulation;
%%% GROUND TRUTH initialisation
keypoints = load('/Users/amadeus/Documents/MATLAB/VAFMR_Exercises/Exercise 6 - Localization using RANSAC and EPnP/data/keypoints.txt');
keypoints = keypoints';
p_W_landmarks = load('/Users/amadeus/Documents/MATLAB/VAFMR_Exercises/Exercise 6 - Localization using RANSAC and EPnP/data/p_W_landmarks.txt');
p_W_landmarks = p_W_landmarks';

[keypoints_1, inlier_mask] = KLT(keypoints, img_left, img_right);
    
%%% MAKE SURE WE START with the right frames at the beginning, in stereo it
%%% differs than in monocular
% Monocular init
%range = (bootstrap_frames(2)+1):last_frame;
% Stereo init

% range = (bootstrap_frames(2)):last_frame;
range = (bootstrap_frames(2)):30;

figure(5);
subplot(1, 3, 3);
%scatter3(p_W_landmarks(1, :), p_W_landmarks(2, :), p_W_landmarks(3, :), 5);
set(gcf, 'GraphicsSmoothing', 'on');
view(0,0);
axis equal;
axis vis3d;
grid on;
prev_img = img_left;

%% State Initialization
% Success field determines whether we localized or failed
S_i0 = struct('success', 0,...
    'keypoints_correspondences', keypoints,... % 2xL
    'p_W_landmarks_correspondences', p_W_landmarks,... % 3xL
    'first_obs_candidate_keypoints', zeros(2,0),... % 2xM First observed candidate keypoints
    'first_obs_candidate_transform', zeros(12,0),... % 16xM Transformation matrices of each ofthe candidates
    'last_obs_candidate_keypoints', zeros(2,0),... % 2xM Last keypoint matched corresponding to initial candidate
    'K', K);

% Bundle Adjustment initialization:
if (BA == true)
    % !!!!!!!!!! HAVE TO ADD POSE FROM INITIALIZATION HERE !!!!!!!!!!
    poses_W_hist = [];
    R_abs = eye(3);
    t_abs = zeros(3,1);
end

%% Continuous Operation
% Store performance by counting number of inlier keypoints per frame
num_inliers = zeros(1, last_frame-(bootstrap_frames(2)+1));
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    
    % Read in next frame:
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
    
    % State and pose update:
    [S_i1, T_i1, inlier_mask] = processFrame(image, prev_img, S_i0, i);
    
    % BUNDLE ADJUSTMENT
    % Refines camera poses and 3D landmark positions of the last m frames.
    if (BA == true)
        tic;
        m = 20;
        % Store absolute (!) pose history (w.r.t. first image frame) as 
        % twist vectors of the last m frames:
        R_abs = R_abs * T_i1(1:3,1:3);
        t_abs = t_abs + T_i1(1:3,4);
        % Convert pose to 6x1 twist column vector:
        tau_i1 = HomogMatrix2twist([R_abs, t_abs; zeros(1,3) 1]);
       
        % Make sure that the first m frames passed before calling BA:
        if (i <= range(m))
            poses_W_hist = [poses_W_hist; tau_i1];
        else
            % Only need to store the poses of the last m frames:
            % IS THIS POSSIBLE WITHOUT COPYING THE WHOLE ARRAY?
            poses_W_hist = [poses_W_hist(7:120,1); tau_i1];
            
            % bundle_adjustment(hidden_state, observations, m, K);
            sprintf('Time needed: Perform Bundle Adjustment: %f seconds', toc)
        end
    end    
    % Store the number of inliers per frame
    num_inliers(i) = nnz(inlier_mask);
    
    % Idea: Update keypoints and 3D landmarks and switch to new keyframe every 5
    % frames, that's why there is a mod there .... 
%     if (mod(i,5) == 0)
         S_i0 = S_i1;
         prev_img = image;
%     end
end
