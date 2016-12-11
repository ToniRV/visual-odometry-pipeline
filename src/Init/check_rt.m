function [points_3d, num_good, M1, M2] = check_rt(R, t, K, ...
    keypoints_database, keypoints_query)
    % Check if in front of camera
    % lambda0*[u v 1]^T = K1 * [Xw Yw Zw]^T
    % lambda1*[u v 1]^T = K1 * R1* [Xw Yw Zw]^T + T
    
    % Retrieve intrinsics
    fx = K(1,1); fy = K(2,2);
    cx = K(1,3); cy = K(2,3);
    
    % Thresholds
    sigma = 1.0;
    sigma_squared = sigma^2;
    th_squared = 4.0*sigma_squared;
    
    % Camera 1 Projection Matrix (3x4) M1 = K[I|0]
    M1 = K*[eye(3) zeros(3,1)];
    O1 = zeros(3,1);
    
    % Camera 2 Projection Matrix (3x4) M2 = K[R|t]
    M2 = K*[R t];
    O2 = -R'*t;
    
    % Initialize Output
    points_3d = [];
    num_good = 0;
    
    for i=1:size(keypoints_database, 2)        
        % Matched keypoints
        kp1 = keypoints_database(:, i);
        kp2 = keypoints_query(:, i);
        
        % Triangulate 3d point in Camera 1
        %point_3d_from_cam1 = triangulate(kp1, kp2, M1, M2)
        point_3d_from_cam1 = linearTriangulation(kp1, kp2, M1, M2);
        point_3d_from_cam1 = point_3d_from_cam1(1:3);
        
        % Check if 3d point is finite
        %if(~isfinite(point_3d_from_cam1))
        %    continue;
        %end
        
        % Check parallax
        normal1 = point_3d_from_cam1 - O1;
        dist1 = norm(normal1);
        
        normal2 = point_3d_from_cam1 - O2;
        dist2 = norm(normal2);
        
        cos_parallax = dot(normal1, normal2)/(dist1*dist2);
        
        % Check depth in front of first camera (Only if enough parallax,
        % as "infinite" points can easily go to negative depth)
        %if ((point_3d_from_cam1(3) <= 0.0) & (cos_parallax < 0.99998))
        %    continue;
        %end
       
        % Check depth in front of first camera (Only if enough parallax,
        % as "infinite" points can easily go to negative depth)
        point_3d_from_cam2 = R*point_3d_from_cam1 + t;
        
        %if ((point_3d_from_cam2(3) <= 0.0) & (cos_parallax < 0.99998))
        %    continue;
        %end
        
        % Check reprojection error in first image
        inv_z1 = 1.0/point_3d_from_cam1(3);
        img1_x = fx*point_3d_from_cam1(1) * inv_z1 + cx;
        img1_y = fy*point_3d_from_cam1(2) * inv_z1 + cy;
        
        square_error1 = (img1_x-kp1(1))*(img1_x-kp1(1)) + ...
            (img1_y-kp1(2))*(img1_y-kp1(2));
        
        %if (square_error1 > th_squared)
        %    continue;
        %end
        
        % Check reprojection error in second image
        inv_z2 = 1.0/point_3d_from_cam2(3);
        img2_x = fx*point_3d_from_cam2(1) * inv_z2 + cx;
        img2_y = fy*point_3d_from_cam2(2) * inv_z2 + cy;
        
        square_error2 = (img2_x-kp2(1))*(img2_x-kp2(1)) + ...
            (img2_y-kp2(2))*(img2_y-kp2(2));
        
        %if (square_error2 > th_squared)
        %    continue;
        %end     
        
        % OUTPUT: points in 3d and number of points
        points_3d(:,i) = point_3d_from_cam1;
        num_good = num_good + 1;
    end
end

