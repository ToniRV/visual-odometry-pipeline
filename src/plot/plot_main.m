function plot_main(img1, state1, inlier_mask, ...
    nnz_inlier_masks, R_C_W, t_C_W, fig_num, i1, ...
    cam_center1_last_n, cam_center1_all)

    P = state1.p_W_landmarks_correspondences;
    kp_homo_query = state1.keypoints_correspondences;
    kp_candidates = state1.last_obs_candidate_keypoints;
    
    % Visualize the 3-D scene
    fig = figure(fig_num);
    scrsz = get(groot, 'ScreenSize');
    fig.Position = [1 scrsz(4) scrsz(3) scrsz(4)];
    subplot(3,4,[1,2]);
    imshow(img1);
    title('Inlier and outlier matches');

    hold on;
    if (nnz(inlier_mask) > 0)
        plot(kp_candidates(2, :), kp_candidates(1, :), 'yx', 'Linewidth', 0.5);
        plot(kp_homo_query(2, :),kp_homo_query(1, :),'gx','Linewidth',2);
    end
    hold off;
    
    subplot(3,4,[5,9]);
    plot(nnz_inlier_masks, 'b-');
    title('# tracked landmarks over last 20 frames');
    ylim([0 200]);
    grid on;

    subplot(3,4,[6,10]);
    %plotCoordinateFrame(R_C_W',cam_center1_all(:,end), 2);
    hold on;
    plot3(cam_center1_all(1,:)', cam_center1_all(2,:)', ...
        cam_center1_all(3,:)', 'b-', 'Linewidth', 3);
    hold off;
    axis equal;
%     xlim([-10 10]);
%     ylim([0 inf]);
    title('Full trajectory');
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    grid on;
    
    
    
    
    subplot(3,4,[3,4,7,8,11,12]);
    if (numel(R_C_W) > 0)
        num_inliers(i1) = nnz(inlier_mask);
        xlim('manual');
        ylim('manual');
        r_x = 10*abs(max(cam_center1_last_n(1,:))-min(cam_center1_last_n(1,:)))/2;
        r_z = 10*abs(max(cam_center1_last_n(3,:))-min(cam_center1_last_n(3,:)))/2;
        xlim([(cam_center1_last_n(1,1))-r_x, (cam_center1_last_n(1,1))+r_x]);
        ylim([(cam_center1_last_n(3,1))-r_z, (cam_center1_last_n(3,1))+r_z]);
        
        plot(cam_center1_last_n(1,:)', cam_center1_last_n(3,:)', 'g-x');
        
        title('Trajectory of last 20 frames and currently observed landmarks');
        disp(['Frame ' num2str(i1) ' localized with ' ...
            num2str(num_inliers(i1)) ' inliers!']);
    else
        disp(['Frame ' num2str(i1) ' failed to localize!']);
    end
    set(gcf, 'GraphicsSmoothing', 'on');
    grid on;    
    % Visualize current 3d landmarks
    P_avg = mean(P,2);
    P_dev = 2*std(P');
    x_limits = [P_avg(1)-P_dev(1), P_avg(1)+P_dev(1)];
    y_limits = [P_avg(2)-P_dev(2), P_avg(2)+P_dev(2)];
    z_limits = [P_avg(3)-P_dev(3), P_avg(3)+P_dev(3)];
    x_mask = P(1,:)>x_limits(1) & P(1,:)<x_limits(2);
    y_mask = P(2,:)>y_limits(1) & P(2,:)<y_limits(2);
    z_mask = P(3,:)>z_limits(1) & P(3,:)<z_limits(2);
    mask = x_mask & y_mask & z_mask;
    
    hold on;
    plot(P(1, mask), P(3, mask), '.k', 'MarkerSize', 4);
    hold off;
     
    pause(0.001);
    
    
    
    
    
%     
%     subplot(3,4,[3,4,7,8,11,12]);
%     if (numel(R_C_W) > 0)
%         num_inliers(i1) = nnz(inlier_mask);
%         plot3(cam_center1_last_n(1,:)', cam_center1_last_n(2,:)', cam_center1_last_n(3,:)', 'g-x');
%         title('Trajectory of last 20 frames and currently observed landmarks');
%         disp(['Frame ' num2str(i1) ' localized with ' ...
%             num2str(num_inliers(i1)) ' inliers!']);
%     else
%         disp(['Frame ' num2str(i1) ' failed to localize!']);
%     end
%     
%     set(gcf, 'GraphicsSmoothing', 'on');
%     view(0,0);
%     grid on;
%     
%     % Visualize current 3d landmarks
%     hold on;
%     scatter3(P(1, :), P(2, :), P(3, :), 5, 'filled');
%     hold off;   
%     pause(0.001);
end

