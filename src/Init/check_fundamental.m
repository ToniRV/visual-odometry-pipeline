function [inliers] = check_fundamental(kp_database, kp_query, F, ...
    num_matches)

    % Initialize output variables
    inliers = false(num_matches, 1);
    
    for i=1:num_matches        
        kp1 = kp_database(:,i);
        kp2 = kp_query(:,i);

        % Epipolar-Line-to-point Distances
        square_dist_temp2= distPoint2EpipolarLine(F, kp1, kp2);
        
        % If error to epipolar line is to big --> outlier
        if (square_dist_temp2 > 1)            
            inliers(i) = false;
        else
            inliers(i) = true;
        end
    end
end