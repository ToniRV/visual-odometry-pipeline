function [database_keypoints_matched, keypoints_query_image_matched, database_descriptors_matched, ...
    query_image_descriptors_matched] = correspondences_2d2d(database_img, query_img)
%
%   INPUT:
    %
    %   *   database_img: Corresponds to the left image / last frame.
    %   * 	query_img: Corresponds to the right image / current frame.
%
%   OUTPUT:
    %
    %   *   database_keypoints_matched: Only those keypoints in the
    %       database image that have been matched.
    %   *   keypoints_query_image_matched: Only those keypoints in the
    %       query image that have been matched.
    %   *   database_descriptors_matched: The descriptors of only those
    %       keypoints in the database image that have been matched.
    %   *   query_descriptors_matched: The descriptors of only those
    %       keypoints in the query image that have been matched.

    % Set to 'true' if you want debugging figures to be shown
    debug_with_figures = true;

    % TODO: IMPROVE PERFORMANCE BY TWEAKING THE PARAMETERS.
    % Parameter values taken from exercise 3:
    harris_patch_size = 9;
    harris_kappa = 0.08; % Typical values between 0.04 - 0.15
    num_keypoints = 200;
    nonmaximum_supression_radius = 8;
    descriptor_radius = 9;
    match_lambda = 4; % Trades of false positives and false negatives

%% Part 1 - Calculation of Harris scores

% Take the left image of the stereo images and calculate the Harris score
% for each pixel in the image.
database_harris_scores = harris(database_img, harris_patch_size, harris_kappa);
assert(min(size(database_harris_scores) == size(database_img)));

if (debug_with_figures)
    figure(1);
    subplot(2, 1, 1);
    imshow(database_img);
    subplot(2, 1, 2);
    imagesc(database_harris_scores);
    axis equal;
    axis off;
end

%% Part 2 - Keypoint selection

% Select the num_keypoints pixels with the highest Harris scores and store
% them in a 2xnum_keypoints matrix; Harris score decreases for increasing
% column index
database_keypoints = selectKeypoints(...
    database_harris_scores, num_keypoints, nonmaximum_supression_radius);

if (debug_with_figures)
    figure(2);
    imshow(database_img);
    hold on;
    plot(database_keypoints(2, :), database_keypoints(1, :), 'rx', 'Linewidth', 2);
    hold off;
end

%% Part 3 - Keypoint descriptors
database_descriptors = describeKeypoints(database_img, database_keypoints, descriptor_radius);

if (debug_with_figures)
    figure(3);
    hold on;
    for i = 1:16
        subplot(4, 4, i);
        patch_size = 2 * descriptor_radius + 1;
        imagesc(uint8(reshape(database_descriptors(:,i), [patch_size patch_size])));
        axis equal;
        axis off;
    end
    hold off;
end

%% Part 4 - Match descriptors between first two images

% Calculate Harris scores and select keypoints for the query / right image:
query_harris_scores = harris(query_img, harris_patch_size, harris_kappa);
% harris_scores_query_image = harris(query_img, harris_patch_size, harris_kappa);
keypoints_query_image = selectKeypoints(...
    query_harris_scores, num_keypoints, nonmaximum_supression_radius);
query_descriptors = describeKeypoints(query_img, keypoints_query_image, descriptor_radius);
% descriptors_query_image = describeKeypoints(query_img, keypoints_query_image, descriptor_radius);

matches = matchDescriptors(query_descriptors, database_descriptors, match_lambda); % 1xQ row vector

[~, query_indices, match_indices] = find(matches);
database_keypoints_matched = database_keypoints(:, match_indices);
keypoints_query_image_matched = keypoints_query_image(:, query_indices);
database_descriptors_matched = database_descriptors(:, match_indices);
query_image_descriptors_matched = descriptors_query_image(:, query_indices);

if (debug_with_figures)
    figure(4);
    imshow(database_img);
    hold on;
    title('First image');
    plot(database_keypoints(2, :), database_keypoints(1, :), 'rx', 'Linewidth', 2);
    plot(database_keypoints_matched(2,:), database_keypoints_matched(1, :), 'bx', 'Linewidth', 2);
    legend('Non-matched keypoints', 'Matched keypoints');
    hold off;

    figure(5);
    imshow(query_img);
    hold on;
    title('Second image');
    plot(keypoints_query_image(2, :), keypoints_query_image(1, :), 'rx', 'Linewidth', 2);
    plot(keypoints_query_image_matched(2, :), keypoints_query_image_matched(1, :), 'bx', 'Linewidth', 2);
    legend('Non-matched keypoints','Matched keypoints');
    plotMatches(matches, keypoints_query_image, database_keypoints);
    hold off;
end

end
