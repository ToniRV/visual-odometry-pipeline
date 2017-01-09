function [plot_pose_hist_, plot_opt_pose_hist_] = BA_plot_update(...
    poses_W_opt_, T_i1, T_i0, ground_truth_pose_, m_on_, range_, i,...
    landmarks_hist_, plot_pose_hist_0, plot_opt_pose_hist_0)
% Updates the plot of current estimated pose, BA optimized pose and 
% current landmarks.

if (i > range_(1))  
    fig = figure(13);
    p_W_GT = ground_truth_pose_(1:600, [4 8 12])';
    % Plot ground truth for the first 600 frames:
    plot(p_W_GT(3, :), -p_W_GT(1, :),'--k');
    hold on;
    plot(landmarks_hist_(3, :), -landmarks_hist_(1, :),'.c','MarkerSize', 3);
    plot_opt_pose_hist_ = plot_opt_pose_hist_0;
    axis equal;
    xlim([-50 300]);
    ylim([-80 40]);
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
    plot(landmarks_hist_(3, :), -landmarks_hist_(1, :),'.m','MarkerSize', 4);
    plot_opt_pose_hist_ = plot_opt_pose_hist_0;
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
    plot(plot_pose_hist_(3, :), -plot_pose_hist_(1, :),'.g','MarkerSize', 4);
    % First time online BA is performed; plot all optimized poses:    
    T_W_frames = reshape(poses_W_opt_, 6, []);
    p_W_opt_estimate = zeros(3, m_on_);
    for i = 1:m_on_
        T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
        p_W_opt_estimate(:,i) = T_W_frame(1:3,4);
    end
    plot_opt_pose_hist_ = [plot_opt_pose_hist_0, p_W_opt_estimate];
    plot(plot_opt_pose_hist_(3, :), -plot_opt_pose_hist_(1, :),'.r',...
        'MarkerSize', 5);
elseif (i >= range_(m_on_-1))
    % Plot current estimate:
    p_W_estimate = -T_i1(1:3,1:3)'*T_i1(1:3, end);
    plot_pose_hist_ = [plot_pose_hist_0, p_W_estimate];
    plot(plot_pose_hist_(3, :), -plot_pose_hist_(1, :),'.g','MarkerSize', 4);
    % Print current optimized estimate and update last m_on_ poses:
    T_i1_opt = twist2HomogMatrix(poses_W_opt_(end-5:end));
    p_W_opt_estimate = T_i1_opt(1:3, end);
    plot_opt_pose_hist_ = [plot_opt_pose_hist_0, p_W_opt_estimate];
    plot(plot_opt_pose_hist_(3, :), -plot_opt_pose_hist_(1, :),'.r',...
        'MarkerSize', 5);
end
drawnow;
hold off;

end

