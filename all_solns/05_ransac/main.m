clear all;
close all;
rng(1);

% Create data for parts 1 and 2
num_inliers = 20;
num_outliers = 10;
noise_ratio = 0.1;
poly = rand(3, 1); % random second-order polynomial
extremum = -poly(2)/(2*poly(1));
xstart = extremum - 0.5;
lowest = polyval(poly, extremum);
highest = polyval(poly, xstart);
xspan = 1;
yspan = highest - lowest;
max_noise = noise_ratio * yspan;
x = rand(1, num_inliers) + xstart;
y = polyval(poly, x);
y = y + (rand(size(y))-.5) * 2 * max_noise;
data = [x (rand(1, num_outliers) + xstart)
    y (rand(1, num_outliers) * yspan + lowest)];

% Data for parts 3 and 4
K = load('../data/K.txt');
keypoints = load('../data/keypoints.txt')';
p_W_landmarks = load('../data/p_W_landmarks.txt')';

% Data for part 4
database_image = imread('../data/000000.png');

% Dependencies
addpath('plot');
% Replace the following with the path to your DLT code:
addpath('../../01_pnp/code');
% Replace the following with the path to your keypoint matcher code:
addpath('../../02_detect_describe_match/code');

%% Part 1 - RANSAC with parabola model
[best_guess_history, max_num_inliers_history] = ...
    parabolaRansac(data, max_noise);

% Compare with full data fit.
full_fit = polyfit(data(1, :), data(2, :), 2);

figure(2);
subplot(1, 2, 1);
scatter(data(1,:), data(2, :), 'b');
hold on;
x = xstart:0.01:(xstart+1);
for i = 1:length(best_guess_history)-1
    guess_plot = plot(x, polyval(best_guess_history(:, i), x), 'b');
end
truth_plot = plot(x, polyval(poly, x), 'g', 'LineWidth', 2);
best_plot = ...
    plot(x, polyval(best_guess_history(:, end), x), 'r', 'LineWidth', 2);
fit_plot = ...
    plot(x, polyval(full_fit, x), 'r--', 'LineWidth', 2);
axis([xstart xstart+1 lowest-max_noise highest+max_noise]);
legend([truth_plot, best_plot, fit_plot, guess_plot], 'ground truth', ...
    'RANSAC result', 'full data fit' , 'RANSAC guesses', 'Location', ...
    'North');
hold off;
title('RANSAC VS full fit');
subplot(1, 2, 2);
plot(max_num_inliers_history);
title('Max num inliers over iterations');

disp('RMS of full fit =');
x = (0:0.01:1) + xstart;
disp(rms(polyval(poly, x) - polyval(full_fit, x)));
disp('RMS of RANSAC =');
x = (0:0.01:1) + xstart;
disp(rms(polyval(poly, x) - polyval(best_guess_history(:, end), x)));


%% Parts 2 and 3 - Localization with RANSAC + DLT/P3P
query_image = imread('../data/000001.png');

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

%% Part 4 - Same, for all frames

figure(5);
subplot(1, 3, 3);
scatter3(p_W_landmarks(1, :), p_W_landmarks(2, :), p_W_landmarks(3, :), 5);
set(gcf, 'GraphicsSmoothing', 'on');
view(0,0);
axis equal;
axis vis3d;
axis([-15 10 -10 5 -1 40]);
for i = 0:9
    query_image = imread(sprintf('../data/%06d.png',i));
    
    [R_C_W, t_C_W, query_keypoints, all_matches, inlier_mask] = ...
    ransacLocalization(query_image, database_image,  keypoints, ...
    p_W_landmarks, K);

    matched_query_keypoints = query_keypoints(:, all_matches > 0);
    corresponding_matches = all_matches(all_matches > 0);

    % Distinguish success from failure.
    if (numel(R_C_W) > 0)
        subplot(1, 3, 3);
        plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2);
        disp(['Frame ' num2str(i) ' localized with ' ...
            num2str(nnz(inlier_mask)) ' inliers!']);
        view(0,0);
    else
        disp(['Frame ' num2str(i) ' failed to localize!']);
    end
    
    subplot(1, 3, [1 2]);
    imshow(query_image);
    hold on;
    plot(matched_query_keypoints(2, (1-inlier_mask)>0), ...
        matched_query_keypoints(1, (1-inlier_mask)>0), 'rx', 'Linewidth', 2);
    if (nnz(inlier_mask) > 0)
        plot(matched_query_keypoints(2, (inlier_mask)>0), ...
            matched_query_keypoints(1, (inlier_mask)>0), 'gx', 'Linewidth', 2);
    end
    plotMatches(corresponding_matches(inlier_mask>0), ...
        matched_query_keypoints(:, inlier_mask>0), ...
        keypoints);
    hold off;
    title('Inlier and outlier matches');
    pause(0.01);
end