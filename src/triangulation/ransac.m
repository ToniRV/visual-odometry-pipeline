function inlier_mask = ransac(database_image, query_image, K, keypoints, ...
    p_W_landmarks)
% keypoints: of database (img2)
rng(1);

% % Create data for parts 1 and 2
% num_inliers = 20;
% num_outliers = 10;
% noise_ratio = 0.1;
% poly = rand(3, 1); % random second-order polynomial
% extremum = -poly(2)/(2*poly(1));
% xstart = extremum - 0.5;
% lowest = polyval(poly, extremum);
% highest = polyval(poly, xstart);
% xspan = 1;
% yspan = highest - lowest;
% max_noise = noise_ratio * yspan;
% x = rand(1, num_inliers) + xstart;
% y = polyval(poly, x);
% y = y + (rand(size(y))-.5) * 2 * max_noise;
% data = [x (rand(1, num_outliers) + xstart)
%     y (rand(1, num_outliers) * yspan + lowest)];

% Data for parts 3 and 4
%K = load('../data/K.txt');
%keypoints = load('../data/keypoints.txt')';
%p_W_landmarks = load('../data/p_W_landmarks.txt')';

% Data for part 4
%database_image = imread('../data/000000.png');

% Dependencies
addpath('plot');
% Replace the following with the path to your DLT code:
addpath('../../all_soln/01_pnp/code');
% Replace the following with the path to your keypoint matcher code:
addpath('../../all_soln/02_detect_describe_match/code');

%% Parts 2 and 3 - Localization with RANSAC + DLT/P3P
%query_image = imread('../data/000001.png');

[R_C_W, t_C_W, query_keypoints, all_matches, inlier_mask, ...
    max_num_inliers_history] = ...
    ransacLocalization(query_image, database_image,  keypoints, ...
    p_W_landmarks, K);

disp('Found transformation T_C_W = ');
disp([R_C_W t_C_W; zeros(1, 3) 1]);
disp('Estimated inlier ratio is');
disp(nnz(inlier_mask)/numel(inlier_mask));

matched_query_keypoints = query_keypoints(:, all_matches > 0);
corresponding_matches = all_matches(all_matches > 0);

figure(4);
subplot(3, 1, 1);
imshow(query_image);
hold on;
plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
plotMatches(all_matches, query_keypoints, keypoints);
title('All keypoints and matches');

subplot(3, 1, 2);
imshow(query_image);
hold on;
plot(matched_query_keypoints(2, (1-inlier_mask)>0), ...
    matched_query_keypoints(1, (1-inlier_mask)>0), 'rx', 'Linewidth', 2);
plot(matched_query_keypoints(2, (inlier_mask)>0), ...
    matched_query_keypoints(1, (inlier_mask)>0), 'gx', 'Linewidth', 2);
plotMatches(corresponding_matches(inlier_mask>0), ...
    matched_query_keypoints(:, inlier_mask>0), ...
    keypoints);
hold off;
title('Inlier and outlier matches');
subplot(3, 1, 3);
plot(max_num_inliers_history);
title('Maximum inlier count over RANSAC iterations.');


end