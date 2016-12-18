function [database_keypoints_matched, keypoints_query_image_matched, database_descriptors_matched, ...
    query_image_descriptors_matched] = correspondences_2d2d(database_img, query_img)
%
%   INPUT:
    %
    %   - database_img:
    %   - query_img:
%
%   OUTPUT:
    %
    %   - database_keypoints_matched: Only the matched keypoints in the
    %   database image.
    %   - keypoints_query_image_matched: Only the matched keypoints in the
    %   query image.
    %   - database_descriptors_matched: Only the descriptors of the
    %   keypoints matched in the database image.
    %    - query_descriptors_matched: Only the descriptors of the
    %   keypoints matched in the query image.
debug_with_figures = true;

% Randomly chosen parameters that seem to work well - can you find better
% ones?
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

%% Part 1 - Calculate Harris scores

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

%% Part 2 - Select keypoints

database_keypoints = selectKeypoints(...
    database_harris_scores, num_keypoints, nonmaximum_supression_radius);

if(debug_with_figures)
    figure(2);
    imshow(database_img);
    hold on;
    plot(database_keypoints(2, :), database_keypoints(1, :), 'rx', 'Linewidth', 2);
    hold off;
end
%% Part 3 - Describe keypoints and show 16 strongest keypoint descriptors

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

%% Part 4 - Match descriptors rsbetween first two images

harris_scores_query_image = harris(query_img, harris_patch_size, harris_kappa);
keypoints_query_image = selectKeypoints(...
    harris_scores_query_image, num_keypoints, nonmaximum_supression_radius);
descriptors_query_image = describeKeypoints(query_img, keypoints_query_image, descriptor_radius);

matches = matchDescriptors(descriptors_query_image, database_descriptors, match_lambda);

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
