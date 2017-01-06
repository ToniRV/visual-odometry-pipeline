function [kp_homo_database_matched, kp_homo_query_matched] = ...
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
    flag_harris_matlab = true;

    if (flag_harris_matlab)
        %% Part 1 - Harris Score Calculation & Keypoint Selection
        % Selects the N pixels with the highest Harris scores while performing
        % non-maximum suppression and stores them in a 2xN matrix; Harris score
        % decreases for increasing column index

        % keypoints = selectKeypoints(...
        %     harris_scores, num_keypoints, nonmaximum_suppression_radius);

        % Detect Harris corners in the database image and extract their
        % coordinates.
        %   --> MinQuality: Pixels with a higher score than this fraction of the
        %                   highest Harris score in the image are accepted as corners.
        %   --> FilterSize: Defines the filter size of the Gaussian filter used to
        %                   smooth the gradient of the input image.
        min_quality = 0.00001; %0.001
        corners_1 = detectHarrisFeatures(img0,'FilterSize', 3, ...
            'MinQuality', min_quality);

        if (debug_verbose)
            keypoints_1 = [corners_1.Location];
            keypoints_1 = round(flipud(keypoints_1.'));
            figure(2);
            imshow(img0);
            hold on;
            plot(keypoints_1(2, :), keypoints_1(1, :), 'rx', 'Linewidth', 2);
        end

        %% Keypoint Matching

        % Calculate the Harris scores of the query image
        % harris_scores_2 = harris(img1, harris_patch_size, harris_kappa);
        % Select the N stongest keypoints in the query image
        % keypoints_2 = selectKeypoints(...
        %     harris_scores_2, num_keypoints, nonmaximum_suppression_radius);

        % Detect the Harris corners of the query image and extract their
        % coordinates:
        corners_2 = detectHarrisFeatures(img1,'FilterSize', 3, ...
            'MinQuality', min_quality);

        % Extract the features
        [features1, valid_points1] = extractFeatures(img0, corners_1, ...
            'Method', 'Block');
        [features2, valid_points2] = extractFeatures(img1, corners_2, ...
            'Method', 'Block');

        % Find the matches
        indexPairs = matchFeatures(features1, features2, 'Unique', true);
        
        % Extract the matched corners
        matchedPoints1 = valid_points1(indexPairs(:,1),:);
        matchedPoints2 = valid_points2(indexPairs(:,2),:);

        % Extract matched keypoint locations and round to integer
        keypoints_1 = [matchedPoints1.Location];
        keypoints_1 = round(flipud(keypoints_1.'));
        keypoints_2 = [matchedPoints2.Location];
        keypoints_2 = round(flipud(keypoints_2.'));

        if (debug_verbose)
            figure(4);
            imshow(img1);
            hold on;
            showMatchedFeatures(img0, img1, matchedPoints1, matchedPoints2);
        end

        %% OUTPUT
        % Homogeneous Keypoint Coordinates
        kp_homo_database_matched = ...
        [keypoints_1; ones(1, size(keypoints_1, 2))];
        kp_homo_query_matched = ...
        [keypoints_2; ones(1, size(keypoints_2, 2))];

    else
        % TODO: IMPROVE PERFORMANCE BY TWEAKING THE PARAMETERS.
        % Parameter values taken from exercise 3:
        descriptor_radius = 9; % A total of 361 pixels per descriptor patch
        match_lambda = 4; % Trades of false positives and false negatives
        harris_patch_size = 9;
        harris_kappa = 0.08; % Typical values between 0.04 - 0.15
        num_keypoints = N;
        nonmaximum_suppression_radius = 2;

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
        keypoints_1 = selectKeypoints(...
        harris_scores, num_keypoints, nonmaximum_suppression_radius);

        if (debug_verbose) 
            figure(2);
            imshow(img0);
            hold on;
            plot(keypoints_1(2, :), keypoints_1(1, :), 'rx', 'Linewidth', 2);
        end

        %% Part 3 - Keypoint Descriptors
        % Obtain the keypoint descriptors, i.e. the intensity values within the patch
        % around each keypoint and show the keypoint descriptors of the 16
        % keypoints with the highest Harris scores.
        descriptors = describeKeypoints(img0, keypoints_1, descriptor_radius);

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
            plotMatches(matches, keypoints_2, keypoints_1);
        end

        %% OUTPUT
            %% Extraction of Keypoint Correspondences
        [~, query_indices, match_indices] = find(matches);
        kp_matches_database = keypoints_1(:, match_indices);
        kp_matches_query = keypoints_2(:, query_indices);

        %% Homogeneous Keypoint Coordinates
        % Express detected keypoints in homogeneous coordinates
        kp_homo_database_matched = ...
            [kp_matches_database; ones(1, size(kp_matches_database, 2))];
        kp_homo_query_matched = ...
            [kp_matches_query; ones(1, size(kp_matches_query, 2))];
    end
end

