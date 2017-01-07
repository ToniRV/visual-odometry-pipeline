function [poses_W_opt_, landmarks_opt_] = runBA_online(poses_W_hist_,...
        landmarks_hist_, observation_hist_, ground_truth_pose_, K, m_on_)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

p_W_GT = ground_truth_pose_(1:m_on_, [4 8 12])';
    
% Define current hidden_state, i.e. poses and landmarks of last m_on_ frames: 
hidden_state = [poses_W_hist_; landmarks_hist_(:)];
opt_hidden_state = runBA_0(hidden_state, cast(observation_hist_,'double'), K, m_on_);
poses_W_opt_ = opt_hidden_state(1:6*m_on_);
landmarks_opt_ = reshape(opt_hidden_state(6*m_on_+1:end), 3, []);

%% Comparision Estimate - Aligned Estimate - Ground truth 
% Compare the estimated and aligned estimate trajectory to the ground truth 
% Reshape twist vectors in hidden_state to 6 x m_offline_ matrix:
T_W_frames = reshape(hidden_state(1:m_on_*6), 6, []);
% Extract pose estimates as 3 x m_offline_ matrix:
p_W_estimate = zeros(3, m_on_);
for i = 1:m_on_
    % Need current (homogeneous) transformation only temporarily to
    % calculate current p_W_estimate(:, i):
    T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
    p_W_estimate(:,i) = T_W_frame(1:3,4);
end
% Align the estimate without BA to the ground truth for performance
% evaluation.
p_W_estimate_aligned = alignEstimateToGroundTruth(...
    p_W_GT, p_W_estimate);

figure(1);
plot(p_W_GT(3, :), -p_W_GT(1, :));
hold on;
plot(p_W_estimate(3, :), -p_W_estimate(1, :));
plot(p_W_estimate_aligned(3, :), -p_W_estimate_aligned(1, :));
hold off;
axis equal;
axis([-10 100 -40 20]);
legend('Ground truth', 'Original estimate', 'Aligned estimate', ...
    'Location', 'SouthWest');

%% Full problem
figure(2);
subplot(1,2,1);
plotMap(hidden_state, [-10 100 -40 20], m_on_);
subplot(1,2,2);
plotMap(opt_hidden_state, [-10 100 -40 20], m_on_);

%% Evaluation of BA Performance
p_W_optim_estimate_aligned = alignEstimateToGroundTruth(...
    p_W_GT, p_W_estimate);

figure(3);
plot(p_W_GT(3, :), -p_W_GT(1, :));
hold on;
plot(p_W_estimate_aligned(3, :), -p_W_estimate_aligned(1, :));
plot(p_W_optim_estimate_aligned(3, :), -p_W_optim_estimate_aligned(1, :));
hold off;
axis equal;
axis([-5 95 -30 10]);
legend('Ground truth', 'Original estimate','Optimized estimate', ...
    'Location', 'SouthWest');

T_W_frames = reshape(opt_hidden_state(1:m_on_*6), 6, []);
% Extract pose estimates as 3 x m_offline_ matrix:
p_W_opt_estimate = zeros(3, m_on_);
for i = 1:m_on_
    % Need current (homogeneous) transformation only temporarily to
    % calculate current p_W_estimate(:, i):
    T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
    p_W_opt_estimate(:,i) = T_W_frame(1:3,4);
end
% Align the estimate without BA to the ground truth for performance
% evaluation.
p_W_opt_estimate_aligned = alignEstimateToGroundTruth(...
    p_W_GT, p_W_opt_estimate);

figure(3);
plot(p_W_GT(3, :), -p_W_GT(1, :));
hold on;
plot(p_W_estimate_aligned(3, :), -p_W_estimate_aligned(1, :));
plot(p_W_opt_estimate_aligned(3, :), -p_W_opt_estimate_aligned(1, :));
hold off;
axis equal;
axis([-10 100 -40 20]);
legend('Ground truth', 'Original (aligned) estimate',...
    'Optimized (aligned) estimate', 'Location', 'SouthWest');


end

