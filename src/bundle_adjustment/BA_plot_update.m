function [plot_pose_hist_] = BA_plot_update(poses_W_opt_, T_i1, T_i0,...
    ground_truth_pose_, m_on_, range_, i, landmarks_hist_, plot_pose_hist_0)
% Updates the plot of current estimated pose, BA optimized pose and 
% current landmarks.


if (i > range_(1))  
    figure(13);
    p_W_GT = ground_truth_pose_(1:600, [4 8 12])';
    % Plot ground truth for the first 600 frames:
    plot(p_W_GT(3, :), -p_W_GT(1, :),'--k');
    hold on;
    % plot(landmarks_hist_(3, :), -landmarks_hist_(1, :),'.r','MarkerSize', 4);
end

if (i == range_(1))        
    scrsz = get(groot, 'ScreenSize');
    fig = figure(13);
    fig.Position = [1 scrsz(4) scrsz(3) scrsz(4)];
    fig.Name = 'Optimized trajectory';
    fig.NumberTitle = 'off';
    hold on;
    p_W_GT = ground_truth_pose_(1:600, [4 8 12])';
    % Plot ground truth for the first 600 frames:
    plot(p_W_GT(3, :), -p_W_GT(1, :),'--k');
    % Plot first position obtained from initialization:
    p_W_estimate_0 = -T_i0(1:3,1:3)*T_i0(1:3, end);
    plot_pose_hist_ = p_W_estimate_0;
    plot(p_W_estimate_0(3, :), -p_W_estimate_0(1, :),'.r','MarkerSize', 4);
    plot(landmarks_hist_(3, :), -landmarks_hist_(1, :),'.r','MarkerSize', 4)
elseif (i < range_(m_on_-1))
    % Only plot estimates until enough frames passed to perform online BA:
    p_W_estimate = -T_i1(1:3,1:3)'*T_i1(1:3, end);
    plot_pose_hist_ = [plot_pose_hist_0, p_W_estimate];
    % plot(p_W_estimate(3, :), -p_W_estimate(1, :),'.g','MarkerSize', 4);
    plot(plot_pose_hist_(3, :), -plot_pose_hist_(1, :),'.g','MarkerSize', 4);
elseif (i == range_(m_on_-1))
    % Plot current estimate:
    p_W_estimate = -T_i1(1:3,1:3)'*T_i1(1:3, end);
    plot_pose_hist_ = [plot_pose_hist_0, p_W_estimate];
    % plot(p_W_estimate(3, :), -p_W_estimate(1, :),'.g','MarkerSize', 4);
    plot(plot_pose_hist_(3, :), -plot_pose_hist_(1, :),'.g','MarkerSize', 4);
    % First time online BA is performed; plot all optimized poses:    
    T_W_frames = reshape(poses_W_opt_, 6, []);
    p_W_opt_estimate = zeros(3, m_on_);
    for i = 1:m_on_
        T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
        p_W_opt_estimate(:,i) = T_W_frame(1:3,4);
    end
    plot(p_W_opt_estimate(3, :), -p_W_opt_estimate(1, :),'.b','MarkerSize', 4);
elseif (i >= range_(m_on_-1))
    % Plot current estimate:
    p_W_estimate = -T_i1(1:3,1:3)'*T_i1(1:3, end);
    plot(p_W_estimate(3, :), -p_W_estimate(1, :),'.g','MarkerSize', 4);
    % FOR NOW, ONLY PRINT CURRENT OPTIMIZED POSE:
    T_i1_opt = twist2HomogMatrix(poses_W_opt_(end-5:end));
    p_W_opt_estimate = T_i1_opt(1:3, end);
    plot(p_W_opt_estimate(3, :), -p_W_opt_estimate(1, :),'.b','MarkerSize', 4);
end
drawnow;
hold off;
    
% %% Comparision Estimate - Aligned Estimate - Ground truth 
% % Compare the estimated and aligned estimate trajectory to the ground truth 
% % Reshape twist vectors in hidden_state to 6 x m_offline_ matrix:
% T_W_frames = reshape(hidden_state(1:m_on_*6), 6, []);
% % Extract pose estimates as 3 x m_offline_ matrix:
% p_W_estimate = zeros(3, m_on_);
% for i = 1:m_on_
%     % Need current (homogeneous) transformation only temporarily to
%     % calculate current p_W_estimate(:, i):
%     T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
%     p_W_estimate(:,i) = T_W_frame(1:3,4);
% end
% % Align the estimate without BA to the ground truth for performance
% % evaluation.
% p_W_estimate_aligned = alignEstimateToGroundTruth(...
%     p_W_GT, p_W_estimate);
% 
% figure(1);
% plot(p_W_GT(3, :), -p_W_GT(1, :));
% hold on;
% plot(p_W_estimate(3, :), -p_W_estimate(1, :));
% plot(p_W_estimate_aligned(3, :), -p_W_estimate_aligned(1, :));
% hold off;
% axis equal;
% axis([-10 100 -40 20]);
% legend('Ground truth', 'Original estimate', 'Aligned estimate', ...
%     'Location', 'SouthWest');
% 
% %% Full problem
% figure(2);
% subplot(1,2,1);
% plotMap(hidden_state, [-10 100 -40 20], m_on_);
% subplot(1,2,2);
% plotMap(opt_hidden_state, [-10 100 -40 20], m_on_);
% 
% %% Evaluation of BA Performance
% p_W_optim_estimate_aligned = alignEstimateToGroundTruth(...
%     p_W_GT, p_W_estimate);
% 
% figure(3);
% plot(p_W_GT(3, :), -p_W_GT(1, :));
% hold on;
% plot(p_W_estimate_aligned(3, :), -p_W_estimate_aligned(1, :));
% plot(p_W_optim_estimate_aligned(3, :), -p_W_optim_estimate_aligned(1, :));
% hold off;
% axis equal;
% axis([-5 95 -30 10]);
% legend('Ground truth', 'Original estimate','Optimized estimate', ...
%     'Location', 'SouthWest');
% 
% T_W_frames = reshape(opt_hidden_state(1:m_on_*6), 6, []);
% % Extract pose estimates as 3 x m_offline_ matrix:
% p_W_opt_estimate = zeros(3, m_on_);
% for i = 1:m_on_
%     % Need current (homogeneous) transformation only temporarily to
%     % calculate current p_W_estimate(:, i):
%     T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
%     p_W_opt_estimate(:,i) = T_W_frame(1:3,4);
% end
% % Align the estimate without BA to the ground truth for performance
% % evaluation.
% p_W_opt_estimate_aligned = alignEstimateToGroundTruth(...
%     p_W_GT, p_W_opt_estimate);
% 
% figure(3);
% plot(p_W_GT(3, :), -p_W_GT(1, :));
% hold on;
% plot(p_W_estimate_aligned(3, :), -p_W_estimate_aligned(1, :));
% plot(p_W_opt_estimate_aligned(3, :), -p_W_opt_estimate_aligned(1, :));
% hold off;
% axis equal;
% axis([-10 100 -40 20]);
% legend('Ground truth', 'Original (aligned) estimate',...
%     'Optimized (aligned) estimate', 'Location', 'SouthWest');

end

