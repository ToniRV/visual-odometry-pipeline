% Tabula rasa
clear all;
clc;
close all;

kitti_path = '/Users/mgrimm/Documents/Studium/9_Semester/VisionAlgoMobileRobotics/kitti';
malaga_path = '/Users/mgrimm/Documents/Studium/9_Semester/VisionAlgoMobileRobotics/malaga';
parking_path = '/Users/mgrimm/Documents/Studium/9_Semester/VisionAlgoMobileRobotics/parking';
rng(1);

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

%% Initialisation
is_auto_frame_monocular_initialisation = false;

if (is_auto_frame_monocular_initialisation) 
    % Automatically choose the best initialisation frames
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
    min_num_inliers = 30;
    Transformations_cw = zeros(4, 4, max_num_auto_frames);
    inliers = zeros(max_num_auto_frames, 1);
    errors = zeros(max_num_auto_frames, 2);
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
        [state_i, T_cw_i, reprojection_errors, ~] = ...
            monocular_initialisation(initial_frame, current_frame, K);

        if (size(state_i.matches_2d, 2) < min_num_inliers)
            break;
        end

        states(i,:) = state_i;
        Transformations_cw(:,:, i) = T_cw_i; 
        inliers(i,:) = size(state_i.matches_2d, 2);
        errors(i,:) = reprojection_errors;
    end

    smallest_error = Inf;
    for i = 1:nnz(inliers)
        error = sum(errors(i,:))/2;

        if (error < smallest_error)
            smallest_error = error;

            if ds == 0
                img1 = imread([kitti_path '/00/image_0/' ...
                    sprintf('%06d.png',i)]);
            elseif ds == 1
                img1 = rgb2gray(imread([malaga_path ...
                    '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
                    left_images(i).name]));
            elseif ds == 2
                img1 = rgb2gray(imread([parking_path ...
                    sprintf('/images/img_%05d.png',i)]));
            else
                assert(false);
            end

            state = states(i, :);
            T_cw = Transformations_cw(:, :, i);
        end
    end
else
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
    
    % Monocular initialisation
    [state, T_cw] = monocular_initialisation(img0, img1, K); 
end