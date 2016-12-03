function [matches, keypoints1, keypoints2] = correspondences_2d2d(img1, img2)

% Randomly chosen parameters that seem to work well - can you find better
% ones?
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

img_1 = img1;

%% Part 1 - Calculate Harris scores

harris_scores = harris(img_1, harris_patch_size, harris_kappa);
assert(min(size(harris_scores) == size(img_1)));
figure(1);
subplot(2, 1, 1);
imshow(img_1);
subplot(2, 1, 2);
imagesc(harris_scores);
axis equal;
axis off;

%% Part 2 - Select keypoints

keypoints_1 = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius);
figure(2);
imshow(img_1);
hold on;
plot(keypoints_1(2, :), keypoints_1(1, :), 'rx', 'Linewidth', 2);

%% Part 3 - Describe keypoints and show 16 strongest keypoint descriptors

descriptors_1 = describeKeypoints(img_1, keypoints_1, descriptor_radius);
figure(3);
for i = 1:16
    subplot(4, 4, i);
    patch_size = 2 * descriptor_radius + 1;
    imagesc(uint8(reshape(descriptors_1(:,i), [patch_size patch_size])));
    axis equal;
    axis off;
end

%% Part 4 - Match descriptors rsbetween first two images
img_2 = img2;

harris_scores_2 = harris(img_2, harris_patch_size, harris_kappa);
keypoints_2 = selectKeypoints(...
    harris_scores_2, num_keypoints, nonmaximum_supression_radius);
descriptors_2 = describeKeypoints(img_2, keypoints_2, descriptor_radius);

matches = matchDescriptors(descriptors_2, descriptors_1, match_lambda);
keypoints1 = keypoints_1;
keypoints2 = keypoints_2;

figure(4);
imshow(img_2);
hold on;
plot(keypoints_2(2, :), keypoints_2(1, :), 'rx', 'Linewidth', 2);
plotMatches(matches, keypoints_2, keypoints_1);

end
