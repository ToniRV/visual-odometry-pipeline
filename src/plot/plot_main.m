function plot_main(img1, state1, inlier_mask, ...
    nnz_inlier_masks, R_C_W, t_C_W, fig_num, i1, ...
    cam_center1_last_n, cam_center1_all)

    P = state1.p_W_landmarks_correspondences;
    kp_homo_query = state1.keypoints_correspondences;
    kp_candidates = state1.last_obs_candidate_keypoints;
    
    % Visualize the 3-D scene
    figure(fig_num);
    subplot(3,4,[1,2]);
    imshow(img1);
    title('Inlier and outlier matches');

    hold on;
    if (nnz(inlier_mask) > 0)
        plot(kp_candidates(2, :), kp_candidates(1, :), 'rx', 'Linewidth', 0.5);
        plot(kp_homo_query(2, :),kp_homo_query(1, :),'gx','Linewidth',2);
    end
    hold off;
    
    subplot(3,4,[5,9]);
    plot(nnz_inlier_masks, 'b-');
    title('# tracked landmarks over last 20 frames');
    grid on;

    subplot(3,4,[6,10]);
    %plotCoordinateFrame(R_C_W',cam_center1_all(:,end), 2);
    hold on;
    plot3(cam_center1_all(1,:)', cam_center1_all(2,:)', ...
        cam_center1_all(3,:)', 'b-', 'Linewidth', 3);
    hold off;
    axis equal;
    title('Full trajectory');
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    grid on;
    
    subplot(3,4,[3,4,7,8,11,12]);
    if (numel(R_C_W) > 0)
        num_inliers(i1) = nnz(inlier_mask);
        plot3(cam_center1_last_n(1,:)', cam_center1_last_n(2,:)', cam_center1_last_n(3,:)', 'g-x');
        title('Trajectory of last 20 frames and currently observed landmarks');
        disp(['Frame ' num2str(i1) ' localized with ' ...
            num2str(num_inliers(i1)) ' inliers!']);
    else
        disp(['Frame ' num2str(i1) ' failed to localize!']);
    end
    
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    grid on;
    
    % Visualize current 3d landmarks
    hold on;
    scatter3(P(1, :), P(2, :), P(3, :), 5, 'filled');
    hold off;   
    pause(0.001);
end

