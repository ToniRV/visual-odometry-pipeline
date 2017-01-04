function [keypoints, keypoints_2, descriptors, descriptors_2, matches] = ...
    correspondences_2d2d(img0, img1, N)

%   INPUT:
    %
    %   *   database_img: Corresponds to the left image / previous frame.
    %   * 	query_img:    Corresponds to the right image / current frame.
    %   *   N:            Number of keypoints to be obtained
%
%   OUTPUT:
    %
    %   *   keypoints: The N keypoints in the database image with the 
    %       highest Harris scores.
    %   *   keypoints_2: The N keypoints in the query image with the 
    %       highest Harris scores.
    %   *   descriptors: The descriptors corresponding to the keypoints 
    %       in the database image.
    %   *   descriptors_2: The descriptors corresponding to the keypoints 
    %       in the query image.

    % Set to 'true' if you want debugging figures to be shown
    debug_verbose = false;

    % TODO: IMPROVE PERFORMANCE BY TWEAKING THE PARAMETERS.
    % Parameter values taken from exercise 3:
    harris_patch_size = 9;
    harris_kappa = 0.08; % Typical values between 0.04 - 0.15
    num_keypoints = N;
    nonmaximum_suppression_radius = 2;
    descriptor_radius = 9; % A total of 361 pixels per descriptor patch
    match_lambda = 4; % Trades of false positives and false negatives

%% Part 1 - Harris scores
% Calculate the Harris scores of the database image
harris_scores = harris(img0, harris_patch_size, harris_kappa);
assert(min(size(harris_scores) == size(img0)));

if (debug_verbose)
    figure(1);
    subplot(2, 1, 1);
    imshow(img0);
    subplot(2, 1, 2);
    imagesc(harris_scores);
    axis equal;
    axis off;
end

%% Part 2 - Keypoint Selection
% Selects the N pixels with the highest Harris scores while performing 
% non-maximum suppression and stores them in a 2xN matrix; Harris score 
% decreases for increasing column index
keypoints = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_suppression_radius);

if (debug_verbose) 
    figure(2);
    imshow(img0);
    hold on;
    plot(keypoints(2, :), keypoints(1, :), 'rx', 'Linewidth', 2);
end

%% Part 3 - Keypoint Descriptors
% Obtain the keypoint descriptors, i.e. the intensity values within the patch
% around each keypoint and show the keypoint descriptors of the 16
% keypoints with the highest Harris scores.
descriptors = describeKeypoints(img0, keypoints, descriptor_radius);

if (debug_verbose) 
    figure(3);
    for i = 1:16
        subplot(4, 4, i);
        patch_size = 2 * descriptor_radius + 1;
        imagesc(uint8(reshape(descriptors(:,i), [patch_size patch_size])));
        axis equal;
        axis off;
    end
end

%% Part 4 - Keypoint Matching

% Calculate the Harris scores of the query image
harris_scores_2 = harris(img1, harris_patch_size, harris_kappa);
% Select the N stongest keypoints in the query image
keypoints_2 = selectKeypoints(...
    harris_scores_2, num_keypoints, nonmaximum_suppression_radius);
% Obtain the corresponding keypoint descriptors
descriptors_2 = describeKeypoints(img1, keypoints_2, descriptor_radius);

% Returns a 1xN row vector; i-th element contains the column index of the
% database keypoint matched to the i-th query keypoint.
matches = matchDescriptors(descriptors_2, descriptors, match_lambda);

if (debug_verbose)
    figure(4);
    imshow(img1);
    hold on;
    plot(keypoints_2(2, :), keypoints_2(1, :), 'rx', 'Linewidth', 2);
    plotMatches(matches, keypoints_2, keypoints);
end
end
