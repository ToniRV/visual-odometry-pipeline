function [inliers] = check_fundamental(kp_database, kp_query, F, ...
    num_matches)

    % Initialize output variables
    % inliers = false(num_matches, 1);
      inliers = false(1, num_matches);
      
    for i=1:num_matches        
        kp1 = kp_database(:,i);
        kp2 = kp_query(:,i);

        % Epipolar-Line-to-point Distances
        square_dist_temp2= distPoint2EpipolarLine(F, kp1, kp2);
        
        % If error to epipolar line is too big --> outlier
        if (square_dist_temp2 > 0.1)            
            inliers(1,i) = false;
        else
            inliers(1,i) = true;
        end
    end
end