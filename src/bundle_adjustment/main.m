clear all;

hidden_state = load('hidden_state.txt');
observations = load('observations.txt');
num_frames = 150;
K = load('/data/K.txt');
poses = load('/data/poses.txt');
p_W_GT_original = poses(:, [4 8 12])';

[hidden_state, observations, p_W_GT_original] = cropProblem(...
    hidden_state, observations, p_W_GT_original, num_frames);
[cropped_hidden_state, cropped_observations, ~] = cropProblem(...
    hidden_state, observations, p_W_GT_original, 4);

%% Compare trajectory to ground truth.
T_W_frames = reshape(hidden_state(1:num_frames*6), 6, []);
p_W_estimate = zeros(3, num_frames);
for i = 1:num_frames
    T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
    p_W_estimate(:, i) = T_W_frame(1:3, end);
end

figure(1);
plot(p_W_GT_original(3, :), -p_W_GT_original(1, :));
hold on;
plot(p_W_estimate(3, :), -p_W_estimate(1, :));
hold off;
axis equal;
axis([-5 95 -30 10]);
legend('Ground truth', 'Estimate', 'Location', 'SouthWest');

%% Align estimate to ground truth.
p_W_estimate_aligned = alignEstimateToGroundTruth(...
    p_W_GT_original, p_W_estimate);

figure(2);
plot(p_W_GT_original(3, :), -p_W_GT_original(1, :));
hold on;
plot(p_W_estimate(3, :), -p_W_estimate(1, :));
plot(p_W_estimate_aligned(3, :), -p_W_estimate_aligned(1, :));
hold off;
axis equal;
axis([-5 95 -30 10]);
legend('Ground truth', 'Original estimate', 'Aligned estimate', ...
    'Location', 'SouthWest');

%% Plot the state before bundle adjustment
figure(1);
plotMap(cropped_hidden_state, cropped_observations, [0 20 -5 5]);
title('Cropped problem before bundle adjustment');

%% Run BA and plot
cropped_hidden_state = runBA(...
    cropped_hidden_state, cropped_observations, K);
figure(2);
plotMap(cropped_hidden_state, cropped_observations, [0 20 -5 5]);
title('Cropped problem after bundle adjustment');

%% Full problem
figure(1);
plotMap(hidden_state, observations, [0 40 -10 10]);
title('Full problem before bundle adjustment');
optimized_hidden_state = runBA(hidden_state, observations, K);
figure(2);
plotMap(optimized_hidden_state, observations, [0 40 -10 10]);
title('Full problem after bundle adjustment');

%% Verify better performance

T_W_frames = reshape(optimized_hidden_state(1:num_frames*6), 6, []);
p_W_estimate = zeros(3, num_frames);
for i = 1:num_frames
    T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
    p_W_estimate(:, i) = T_W_frame(1:3, end);
end

p_W_optim_estimate_aligned = alignEstimateToGroundTruth(...
    p_W_GT_original, p_W_estimate);

figure(3);
plot(p_W_GT_original(3, :), -p_W_GT_original(1, :));
hold on;
plot(p_W_estimate_aligned(3, :), -p_W_estimate_aligned(1, :));
plot(p_W_optim_estimate_aligned(3, :), -p_W_optim_estimate_aligned(1, :));
hold off;
axis equal;
axis([-5 95 -30 10]);
legend('Ground truth', 'Original estimate','Optimized estimate', ...
    'Location', 'SouthWest');