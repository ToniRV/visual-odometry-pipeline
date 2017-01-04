function [inlier_mask, F] = ransac(kp_database, kp_query)
    % Parameters
    num_iterations = 1177; % See lecture slides
    num_matches = size(kp_database,2);
    top_score = 0.0;    
    kp_database = kp_database([2 1 3],:)
    kp_query = kp_query([2 1 3],:);
    
    % 1) Perform RANSAC iterations and save solution that achieves the
    %    highest number of inliers.
    for i=1:num_iterations
        % Within each iteration, randomly select 8 keypoint correspondences
        % from the set of all keypoint correspondences.
        [p1i, idx] = datasample(kp_database, 8, 2, 'Replace', false);
        p2i = kp_query(:,idx);
        
        % 2) Compute the Fundamental matrix based on the randomly selected
        %    keypoint correspondences.
        Fi = fundamentalEightPoint_normalized(p1i, p2i);
        
        % 3) Determine the current inliers
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