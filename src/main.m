%% Tabula rasa
clear all;
close all;

% REMOVE
rng(1);

% Modify paths.txt with your paths to the datasets and the
% visual-odometry-pipeline folder
% An example of file is provided under Parameters folder 
% with the name paths_example.txt, copy it under
% paths.txt with your paths.
type Parameters/paths.txt
fileID = fopen('Parameters/paths.txt','r');
formatSpec = '%s';
paths = textscan(fileID,formatSpec, 'Delimiter', '"');
fclose(fileID);

% Dataset paths
vo_path = paths{1}{2};
addpath(genpath(vo_path));
kitti_path_ = paths{1}{4};
malaga_path_ = paths{1}{6};
parking_path_ = paths{1}{8};
smartphone_path_ = '/home/tonirv/Documents/visual-odometry-pipeline/src/Smartphone/';

%% Setup
%%% Select dataset to run:
dataset_ = 'Kitti';                                             % 'Kitti', 'Malaga', 'Parking', 'Smartphone'
%%% Select initialisation method to run:
initialisation_ = 'Monocular';     % 'Monocular', 'Stereo', 'Ground Truth'
%%% Select bundle adjustment method:
BA_ = 'Offline';                   % 'Offline', 'Online', 'None'
%%% Select if initialisation frames should be picked automatically
is_auto_frame_monocular_initialisation_ = false;
%%% Select if relocalisation should be turned on
is_relocalisation = false;

% Parameters
baseline_  = 0;
gound_truth_pose_ = 0;
last_frame_ = 0;
left_images_ = 0;
m_on_ = 30; % Set number of frames used for full online BA
m_off_ = 150; % Set number of frames used for full offline BA

switch dataset_
    case 'Kitti'
        % need to set kitti_path to folder containing "00" and "poses"
        assert(exist('kitti_path_', 'var') ~= 0);
        ground_truth_pose_ = load([kitti_path_ '/poses/00.txt']);
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
    case 'Smartphone'
        load('./Smartphone/data/cameraParams.mat');
        K = cameraParams.IntrinsicMatrix';
        last_frame_ = 12;
    otherwise
        disp(' No correct dataset specified');
        assert(false);
end

%% Initialisation
params_match_features_matlab = struct(...
    'Method', 'Exhaustive',...
    'MatchThreshold', 10.0,...
    'MaxRatio', 0.6,...
    'Metric', 'SSD',...
    'Unique', false);
params_extract_features = struct(...
    'Method', 'FREAK');
params_correspondences_2D2D = struct(...
    'debug_verbose', false,...
    'flag_harris_matlab', false,...
    'descriptor_radius', 9,... % A total of 361 pixels per descriptor patch    % Only used if flag_harris_matlab is false
    'match_lambda', 4,... % Trades of false positives and false negatives    % Only used if flag_harris_matlab is false
    'harris_patch_size', 9,...                                                                         % Only used if flag_harris_matlab is false
    'harris_kappa', 0.08,... % Typical values between 0.04 - 0.15                 % Only used if flag_harris_matlab is false
    'num_keypoints', 1000,...                                                                       % Only used if flag_harris_matlab is false
    'nonmaximum_suppression_radius', 2,...                                               % Only used if flag_harris_matlab is false
    'filter_size', 3,...                                                                                      % Only used if flag_harris_matlab is true
    'min_quality', 0.00001,...
    'match_features', params_match_features_matlab,...
    'extract_features', params_extract_features);                                                                          % Only used if flag_harris_matlab is true
params_ransac = struct(...
    'num_iterations', 1000);
    
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

img0_ = 0;
img1_ = 0;
range_ = 0;
i_ = 0;
keypoints_ = zeros(0);
p_W_landmarks_ = zeros(0);

% Initialize parameters
switch initialisation_
    case 'Monocular'
        monoInit = makeMonoInit(init_parameters.mono);
    otherwise
        disp('Autoframes ONLY with MONOCULAR');
end

% Automatically choose the best initialisation frames 
if (is_auto_frame_monocular_initialisation_)
    % Retrieve the initial image
    idx_initial_image = 1;
    img0_ = getImage(dataset_, idx_initial_image, ...
        kitti_path_, malaga_path_, parking_path_, smartphone_path_, left_images_);
        
    max_num_auto_frames = 10;
    min_num_inliers = 30;
    smallest_error = Inf;
    for i = (idx_initial_image+1):(max_num_auto_frames+idx_initial_image)
        % Seed the random generator every time
        rng(1);
        
        % Retrieve the current image
         current_image = getImage(dataset_, i, kitti_path_, ...
             malaga_path_, parking_path_, smartphone_path_, left_images_);

        % Monocular initialisation with repro errors
        [state_i, T_i0, reprojection_errors, ~] = ...
            monoInit(img0_, current_image);

        % Check if number of inliers is big enough
        if (size(state_i.matches_2d, 2) < min_num_inliers)
            break;
        end

        % Search for smallest reprojection error
        if (sum(reprojection_errors) < smallest_error)            
            smallest_error = sum(reprojection_errors);
            img1_ = current_image;
            keypoints_ = state_i.matches_2d(1:2,:);
            p_W_landmarks_ = state_i.landmarks(1:3,:);
            i_ = i;
        end
    end
    sprintf('Automatically choose Fram %i and %i for intialisation', ...
        idx_initial_image, i_)
    
    range_ = (i_+1):last_frame_;
else
    %% Bootstrap
    bootstrap_frames_ = 0;
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
                    bootstrap_frames_ = [0, 2];
                    range_ = (bootstrap_frames_(2)+1):last_frame_;
                otherwise
                    disp(['Wrong initialisation ', initialisation_,' method for dataset: ', dataset_]);
                    assert(false);
            end
            [img0_, img1_] = parseParkingImages(parking_path_, bootstrap_frames_);
        case 'Smartphone'
            switch initialisation_
                case 'Monocular'
                    bootstrap_frames_ = [1, 2];
                    range_ = (bootstrap_frames_(2)+1):last_frame_;
                otherwise
                    disp(['Wrong initialisation ', initialisation_,' method for dataset: ', dataset_]);
                    assert(false);
            end
            img0_ = parseSmartphoneImages(smartphone_path_, bootstrap_frames_(1));
            img1_ = parseSmartphoneImages(smartphone_path_, bootstrap_frames_(2));
        otherwise
            disp(['Wrong dataset: ', dataset_]);
            assert(false);
    end
    
    switch initialisation_
        case 'Monocular'
            [state, T_i0, ~, ~] = monoInit(img0_, img1_);
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
    
    i_ = bootstrap_frames_(2);
end

if (isempty(keypoints_) || isempty(p_W_landmarks_))
    disp(' Initialisation did not succeed');
    assert('false');
end

%% Continuous operation
S_i0 = struct(...
    'keypoints_correspondences', keypoints_,...          % 2xL
    'p_W_landmarks_correspondences', p_W_landmarks_,...  % 3xL
    'first_obs_candidate_keypoints', zeros(2,0),...      % 2xM First observed candidate keypoints
    'first_obs_candidate_transform', zeros(12,0),...     % 12xM Transformation matrices of each ofthe candidates
    'last_obs_candidate_keypoints', zeros(2,0)...        % 2xM Last keypoint matched corresponding to initial candidate
    );

params_harris_matlab = struct(...
    'MinQuality', 0.1,...
    'FilterSize', 5);
params_harris_lecture = struct(...
    'harris_patch_size', 9,...
    'harris_kappa', 0.08,...
    'nonmaximum_supression_radius', 8);
params_harris_detector = struct(...
    'debug_with_figures', false,...
    'num_keypoints', 100,...
    'cols', 4,...
    'rows', 4,...
    'algorithm', 1,...
    'harris_matlab', params_harris_matlab,...
    'harris_lecture', params_harris_lecture);
params_ransac_localization = struct(...
    'K', K,...
    'num_iterations', 500,...
    'pixel_tolerance', 5);

cont_op_parameters = struct(...
    'K', K,...
    'harris_detector', params_harris_detector,...
    'triangulation_angle_threshold', 3,...
    'suppression_radius', 3,...
    'reprojection_error_threshold', 100,...
    'ransac_localization', params_ransac_localization);

% Bundle Adjustment initialization:
if (strcmp(BA_,'None') == 0)
    [index_mask_, index_hist_m_, poses_W_hist_, poses_W_opt_,...
        plot_pose_hist_, plot_opt_pose_hist_, landmarks_hist_, observation_hist_] = ...
        BA_init(S_i0, T_i0, initialisation_, m_on_);
end

prev_img = getImage(dataset_, i_, kitti_path_, malaga_path_, parking_path_, smartphone_path_, left_images_);
% Initialize Parmeters
processFrame = makeProcessFrame(cont_op_parameters);

num_frames_plotting = 80;
cam_center1 = zeros(3,num_frames_plotting);
cam_center_all = [];
nnz_inlier_masks = zeros(1,num_frames_plotting);
T_i1_prev = eye(4);

for i = range_
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    image = getImage(dataset_, i, kitti_path_, malaga_path_, parking_path_, smartphone_path_, left_images_);
    % State and pose update:
    [S_i1, T_i1, inlier_mask, validity_mask, new_3D, new_2D] = ...
        processFrame(image, prev_img, S_i0, i);
    image = getImage(dataset_, i, kitti_path_, malaga_path_, parking_path_, left_images_);
    % BUNDLE ADJUSTMENT
    if (numel(T_i1) == 0)
        if (is_relocalisation)  
            % Relocalization
            bootstrap_frames_ = [i-2, i];
            range_ = (bootstrap_frames_(2)+1):last_frame_;
            img0_ = prev_img;
            img1_ = image;

            switch initialisation_
                case 'Monocular'
                    monoInit = makeMonoInit(init_parameters.mono);
                    [state, T_i0, ~, ~] = monoInit(img0_, img1_);
                    keypoints_ = state.matches_2d(1:2,:);
                    p_W_landmarks_ = state.landmarks(1:3,:);
                otherwise
                    disp('Only monocular for relocalisation');
            end

            if (isempty(keypoints_) || isempty(p_W_landmarks_))
                disp('Relocalisation did not succeed');
                assert('false');
                break;
            end

            T_diff = T_i1_prev2 - T_i1_prev;
            T_i0(1:3,4) = T_i0(1:3,4)*norm(T_diff);
            T_final_after_reloc_h = T_i1_prev*T_i0;
            T_i1 = T_final_after_reloc_h(1:3,:);
        else
            % Go to Offline BundleAdjustment
            break;
        end
    elseif (strcmp(BA_,'None') == 0)
        if (strcmp(BA_,'Offline') == 1)
            [poses_W_hist_, landmarks_hist_, observation_hist_, ...
                index_mask_] = BA_offline_hist_update(S_i0, T_i1, ...
                validity_mask, inlier_mask, index_mask_, new_3D, new_2D,...
                poses_W_hist_, landmarks_hist_, observation_hist_);
            % Stop VO pipeline if enough frames to perform offline BA have
            % been recorded:
            if (i == range_(m_off_-1))
                break;
            end
        elseif (strcmp(BA_,'Online') == 1)
            [poses_W_hist_, landmarks_hist_, observation_hist_,...
                index_mask_, index_hist_m_] = BA_online_hist_update(...
                S_i0, T_i1, validity_mask, inlier_mask, index_mask_,...
                index_hist_m_, new_3D, new_2D, poses_W_hist_,...
                landmarks_hist_, observation_hist_, range_, m_on_, i);
            if (i >= range_(m_on_-1))
                [poses_W_opt_, landmarks_hist_] = runBA_online(...
                   poses_W_hist_, landmarks_hist_, index_hist_m_,...
                   observation_hist_, K, m_on_);
                % Use optimized landmarks for next pose update:
                State_i1.p_W_landmarks_correspondences =...
                landmarks_hist_(:, index_mask_);
            end
            %BA_plot_update(poses_W_opt_, T_i1, T_i0,...
            %    ground_truth_pose_, i, range_, i);
            [plot_pose_hist_, plot_opt_pose_hist_] = BA_plot_update(...
                poses_W_opt_, T_i1, T_i0, ground_truth_pose_, m_on_,...
                range_, i, landmarks_hist_, plot_pose_hist_,...
                plot_opt_pose_hist_);            
        else
            disp(['Unidentified BA method: ', BA_]);
            assert(false);
        end
    end
    % Update variables
    S_i0 = S_i1;
    prev_img = image;
    T_i0 = T_i1;
    T_i1_prev(1:3,:) = T_i1;
    T_i1_prev2 = T_i1_prev;

    %% PLOTTING
    cam_help = -T_i1(:,1:3)'*T_i1(:,4);
    cam_center1 = [cam_help cam_center1(:, 1:end-1)];
    cam_center_all(:, end+1) = cam_help;
    nnz_inlier_masks = [nnz_inlier_masks(1, 2:end) nnz(inlier_mask)];
    
    plot_main(image, S_i1, inlier_mask, ...
        nnz_inlier_masks, T_i1(:,1:3), T_i1(:,4), 20, i, ...
        cam_center1, cam_center_all);
end

%% Offline Bundle Adjustment
disp('Dataset finished to localize');
if (strcmp(BA_,'Offline') == 1)
   [poses_W_opt_, landmarks_opt_] = runBA_offline(poses_W_hist_,...
        landmarks_hist_, observation_hist_, ground_truth_pose_, K, m_off_);
end
