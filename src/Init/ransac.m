function [inlier_mask, F] = ransac(kp_database, kp_query, sigma)
    % Parameters
    num_iterations = 1000;
    num_matches = size(kp_database,2);
    top_score = 0.0;    
    
    % 1) Perform RANAC iterations and save solution with hightest score
    for i=1:num_iterations
        % Select a minimum set
        [p1i, idx] = datasample(kp_database, 8, 2, 'Replace', false);
        p2i = kp_query(:,idx);
        
        % 2) Compute the Fundamental matrix
        Fi = fundamentalEightPoint_normalized(p1i, p2i);
        
        % 3) Current inliers
        [current_inliers] = check_fundamental(kp_database, kp_query, ...
            Fi, num_matches);

        % 4) Choose Fundamental matrix with most inliers
        if (nnz(current_inliers) > top_score)
           F = Fi;
           inlier_mask = current_inliers;
           top_score = nnz(current_inliers);
        end
    end
end