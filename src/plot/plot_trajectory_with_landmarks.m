function plot_trajectory_with_landmarks(img0, img1, kp_homo_database, ...
    kp_homo_query, P, R, T)
    % Visualize the 3-D scene
    figure(10);
    subplot(2,3,1)
    imshow(img0,[]);
    hold on
    plot(kp_homo_database(1,:), kp_homo_database(2,:), 'ys');
    title('Image 1')

    subplot(2,3,4)
    imshow(img1,[]);
    hold on
    plot(kp_homo_query(1,:), kp_homo_query(2,:), 'ys');
    title('Image 2')
    % Display camera pose
    plotCoordinateFrame(eye(3),zeros(3,1), 20);
    %text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

    subplot(2,3,[2,3,5,6])
    center_cam2_W = -R'*T;
    plotCoordinateFrame(R',center_cam2_W, 20);
    %text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
    
    % Landmarks
    hold on
    scatter3(P(1,:), P(2,:), P(3,:), 'o')
    view([0 -1 0])
end

