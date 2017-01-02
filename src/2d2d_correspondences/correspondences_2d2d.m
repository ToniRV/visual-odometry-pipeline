function [keypoints, keypoints_2, descriptors, descriptors_2, matches] = correspondences_2d2d(img0, img1)

debug_verbose = false;

% Randomly chosen parameters that seem to work well - can you find better
% ones?
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 1000;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

img = img0;

%% Part 1 - Calculate Harris scores

harris_scores = harris(img, harris_patch_size, harris_kappa);
assert(min(size(harris_scores) == size(img)));

if (debug_verbose)
    figure(1);
    subplot(2, 1, 1);
    imshow(img);
    subplot(2, 1, 2);
    imagesc(harris_scores);
    axis equal;
    axis off;
end

%% Part 2 - Select keypoints

keypoints = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius);

if (debug_verbose) 
    figure(2);
    imshow(img);
    hold on;
    plot(keypoints(2, :), keypoints(1, :), 'rx', 'Linewidth', 2);
end

%% Part 3 - Describe keypoints and show 16 strongest keypoint descriptors

descriptors = describeKeypoints(img, keypoints, descriptor_radius);
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

%% Part 4 - Match descriptors between first two images
img_2 = img1;

harris_scores_2 = harris(img_2, harris_patch_size, harris_kappa);
keypoints_2 = selectKeypoints(...
    harris_scores_2, num_keypoints, nonmaximum_supression_radius);
descriptors_2 = describeKeypoints(img_2, keypoints_2, descriptor_radius);

matches = matchDescriptors(descriptors_2, descriptors, match_lambda);

if (debug_verbose)
    figure(4);
    imshow(img_2);
    hold on;
    plot(keypoints_2(2, :), keypoints_2(1, :), 'rx', 'Linewidth', 2);
    plotMatches(matches, keypoints_2, keypoints);
end
end
