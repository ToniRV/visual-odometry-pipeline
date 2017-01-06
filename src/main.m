%% Tabula rasa
clear all;
close all;

% REMOVE
rng(1);

addpath('2d2d_correspondences');
addpath('Init');
% Replace the following with the path to your keypoint matcher code:
addpath('../../00_camera_projection/code');

% Dataset paths
kitti_path_ = '/home/tonirv/Downloads/kitti';
malaga_path_ = '/home/tonirv/Downloads/';
parking_path_ = '/home/tonirv/Documents/Vision Algorithms for Mobile Robotics/VO - Project/parking';
%% Setup

%%% Select dataset to run:
dataset_ = 'Kitti';                                             % 'Kitti', 'Malaga', 'Parking'
%%% Select initialisation method to run:
initialisation_ = 'Monocular';                             % 'Monocular', 'Stereo', 'Ground Truth'

% Parameters
baseline_  = 0;
gound_truth_pose_ = 0;
last_frame_ = 0;
left_images_ = 0;

switch dataset_
    case 'Kitti'
        % need to set kitti_path to folder containing "00" and "poses"
        assert(exist('kitti_path_', 'var') ~= 0);
        gound_truth_pose_ = load([kitti_path_ '/poses/00.txt']);
        gound_truth_pose_ = gound_truth_pose_(:, [end-8 end]);
        baseline_ = 0.54; % Given by the KITTI dataset:
        last_frame_ = 4540;
        K = [7.188560000000e+02 0 6.071928000000e+02
            0 7.188560000000e+02 1.852157000000e+02
            0 0 1];
    case 'Malaga'
        % Path containing the many files of Malaga 7.
        assert(exist('malaga_path_', 'var') ~= 0);
        images = dir([malaga_path_ ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images_ = images(3:2:end);
        last_frame_ = length(left_images_);
        K = [621.18428 0 404.0076
            0 621.18428 309.05989
            0 0 1];
    case 'Parking'
        % Path containing images, depths and all...
        assert(exist('parking_path_', 'var') ~= 0);
        last_frame_ = 598;
        K = load([parking_path_ '/K.txt']);
        ground_truth = load([parking_path_ '/poses.txt']);
        ground_truth = ground_truth(:, [end-8 end]);
    otherwise
        disp(' No correct dataset specified');
        assert(false);
end

%% Bootstrap frames

bootstrap_frames_ = 0;
img0_ = 0;
img1_ = 0;
range_ = 0;
switch dataset_
    case 'Kitti'
        switch initialisation_
            case 'Monocular'
                bootstrap_frames_ = [0, 1];
                range_ = (bootstrap_frames_(2)+1):last_frame_;
            case 'Stereo'
                bootstrap_frames_ = [0, 0];
                range_ = (bootstrap_frames_(1)+1):last_frame_;
            case 'Ground Truth'
                bootstrap_frames_ = [0, 0];
                range_ = (bootstrap_frames_(1)+1):last_frame_;
            otherwise
                disp(['Wrong initialisation ', initialisation_,' method for dataset: ', dataset_]);
                assert(false);
        end
        [img0_, img1_] = parseKittiImages(kitti_path_, bootstrap_frames_, initialisation_);
    case 'Malaga'
        switch initialisation_
            case 'Monocular'
                bootstrap_frames_ = [1, 2];
                range_ = (bootstrap_frames_(2)+1):last_frame_;
            otherwise
                disp(['Wrong initialisation ', initialisation_,' method for dataset: ', dataset_]);
                assert(false);
        end
        [img0_, img1_] = parseMalagaImages(malaga_path_, left_images_, bootstrap_frames_);
    case 'Parking'
        switch initialisation_
            case 'Monocular'
                bootstrap_frames_ = [0, 1];
                range_ = (bootstrap_frames_(2)+1):last_frame_;
            otherwise
                disp(['Wrong initialisation ', initialisation_,' method for dataset: ', dataset_]);
                assert(false);
        end
        [img0_, img1_] = parseParkingImages(parking_path_, bootstrap_frames_);
    otherwise
        disp(['Wrong dataset: ', dataset_]);
        assert(false);
end

%% Initialisation
params_match_features_matlab = struct(...
    'Method', 'Exhaustive',...
    'MatchThreshold', 10.0,...
    'MaxRatio', 0.6,...
    'Metric', 'SSD',...
    'Unique', true);
params_correspondences_2D2D = struct(...
    'debug_verbose', false,...
    'flag_harris_matlab', true,...
    'descriptor_radius', 9,... % A total of 361 pixels per descriptor patch    % Only used if flag_harris_matlab is false
    'match_lambda', 4,... % Trades of false positives and false negatives    % Only used if flag_harris_matlab is false
    'harris_patch_size', 9,...                                                                         % Only used if flag_harris_matlab is false
    'harris_kappa', 0.08,... % Typical values between 0.04 - 0.15                 % Only used if flag_harris_matlab is false
    'num_keypoints', 1000,...                                                                       % Only used if flag_harris_matlab is false
    'nonmaximum_suppression_radius', 2,...                                               % Only used if flag_harris_matlab is false
    'filter_size', 3,...                                                                                      % Only used if flag_harris_matlab is true
    'min_quality', 0.00001,...
    'match_features', params_match_features_matlab);                                                                          % Only used if flag_harris_matlab is true
params_ransac = struct(...
    'num_iterations', 500);
    
params_stereo = struct(...
    'triangulation_algorithm', 'matlab_triangulation',...
    'debug_with_figures', false,...
    'baseline', baseline_,...
    'K', K,...
    'correspondences_2D2D', params_correspondences_2D2D);
params_mono = struct(...
    'debug_verbose', false,...
    'num_keypoints', 1000,...
    'K', K,...
    'correspondences_2D2D', params_correspondences_2D2D, ...
    'ransac', params_ransac);

init_parameters = struct('stereo', params_stereo, 'mono', params_mono);


keypoints_ = zeros(0);
p_W_landmarks_ = zeros(0);
switch initialisation_
    case 'Monocular'
        mono_init = makeMonoInit(init_parameters.mono);
        [state, ~] = mono_init(img0_, img1_);
        keypoints_ = state.matches_2d(1:2,:);
        p_W_landmarks_ = state.landmarks(1:3,:);
    case 'Stereo'
        stereoInit = makeStereoInit(init_parameters.stereo);
        [keypoints_, p_W_landmarks_] = stereoInit(img0_, img1_);
    case 'Ground Truth'
        %%% GROUND TRUTH initialisation
        if (strcmp(dataset_, 'Kitti'))
            keypoints_ = load('~/Documents/Vision Algorithms for Mobile Robotics/Exercise 6 - Localization using RANSAC and EPnP/data/keypoints.txt');
            keypoints_ = keypoints_';
            p_W_landmarks_ = load('~/Documents/Vision Algorithms for Mobile Robotics/Exercise 6 - Localization using RANSAC and EPnP/data/p_W_landmarks.txt');
            p_W_landmarks_ = p_W_landmarks_';
        else
            disp('There is no ground truth for the dataset specified');
        end
    otherwise
        disp('No correct initialisation specified');
end

if (isempty(keypoints_) || isempty(p_W_landmarks_))
    disp(' Initialisation did not succeed');
    assert('false');
end

%% Continuous operation

% Plotting
figure(5);
subplot(1, 3, 3);
%scatter3(p_W_landmarks(1, :), p_W_landmarks(2, :), p_W_landmarks(3, :), 5);
set(gcf, 'GraphicsSmoothing', 'on');
view(0,0);
axis equal;
axis vis3d;
grid on;

S_i0 = struct(...
    'keypoints_correspondences', keypoints_,...                     % 2xL
    'p_W_landmarks_correspondences', p_W_landmarks_,... % 3xL
    'first_obs_candidate_keypoints', zeros(2,0),...                  % 2xM First observed candidate keypoints
    'first_obs_candidate_transform', zeros(12,0),...               % 12xM Transformation matrices of each ofthe candidates
    'last_obs_candidate_keypoints', zeros(2,0),...                   % 2xM Last keypoint matched corresponding to initial candidate
    'K', K);

num_inliers = zeros(1, last_frame_-(bootstrap_frames_(2)+1));

% Store Image_i0, aka previous image to kickstart continuous operation.
prev_image_ = 0;
i_ = bootstrap_frames_(2);
switch dataset_
    case 'Kitti'
        prev_image_ = imread([kitti_path_ '/00/image_0/' sprintf('%06d.png',i_)]);
    case 'Malaga'
        prev_image_ = rgb2gray(imread([malaga_path_ ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images_(i_).name]));
    case 'Parking'
        prev_image_ = im2uint8(rgb2gray(imread([parking_path_ ...
            sprintf('/images/img_%05d.png',i_)])));
    otherwise
        disp(['Wrong dataset: ', dataset_]);
        assert(false);
end

prev_img = prev_image_;

params_harris_matlab = struct(...
    'MinQuality', 0.01,...
    'FilterSize', 5);
params_harris_lecture = struct(...
    'harris_patch_size', 9,...
    'harris_kappa', 0.08,...
    'nonmaximum_supression_radius', 8);
params_harris_detector = struct(...
    'debug_with_figures', 'false',...
    'num_keypoints', 70,...
    'cols', 4,...
    'rows', 3,...
    'algorithm', 1,...
    'harris_matlab', params_harris_matlab,...
    'harris_lecture', params_harris_lecture);

cont_op_parameters = struct(...
    'params_harris_detector', params_harris_detector,...
    'triangulation_angle_threshold', 35);
    
processFrame = makeProcessFrame(cont_op_parameters);
    
for i = range_
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    switch dataset_
        case 'Kitti'
            image = imread([kitti_path_ '/00/image_0/' sprintf('%06d.png',i)]);
        case 'Malaga'
            image = rgb2gray(imread([malaga_path_ ...
                '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                left_images_(i).name]));
        case 'Parking'
            image = im2uint8(rgb2gray(imread([parking_path_ ...
                sprintf('/images/img_%05d.png',i)])));
        otherwise
            disp(['Wrong dataset: ', dataset_]);
            assert(false);
    end
    
    [S_i1, T_i1, inlier_mask] = processFrame(image, prev_img, S_i0, i);
%      subplot(1, 3, 3);
%      scatter3(S_i1.p_W_landmarks_correspondences(1, :), ...
%          S_i1.p_W_landmarks_correspondences(2, :), ...
%          S_i1.p_W_landmarks_correspondences(3, :), 5);

    
    % Store the number of inliers per frame
    num_inliers(i) = nnz(inlier_mask);
    
    % Idea: Update keypoints and 3D landmarks and switch to new keyframe every 5
    % frames, that's why there is a mod there .... 
%     if (mod(i,5) == 0)
         S_i0 = S_i1;
         prev_img = image;
%     end
end


