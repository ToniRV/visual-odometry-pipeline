function [inliers] = check_fundamental(kp_database, kp_query, F, ...
    num_matches)

    % Initialize output variables
    inliers = false(num_matches, 1);
    
    for i=1:num_matches        
        % Epipolar-Line-to-point Distances
        square_dist_temp2= distPoint2EpipolarLine(F, ...
            kp_database(:,i), kp_query(:,i));
        
        % If error to epipolar line is too big --> outlier
        if (square_dist_temp2 < 0.1)            
            inliers(i) = true;
        end
    end
end