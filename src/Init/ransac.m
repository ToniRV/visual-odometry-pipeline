function [inlier_mask, F] = ransac(kp_database, kp_query, K, sigma)

    % Parameters
    num_iterations = 1000;
    num_matches = size(kp_database,2);
    top_score = 0.0;    

    % 1) Generate sets of 8 inidices for n - iterations
    Sets = [];
    
    for i=1:num_matches
        all_indices(i) = i;
    end
    
    
    for i=1:num_iterations
        available_indices = all_indices;
        
        % Select a minimum set
        for j=1:8
            randi = ceil(size(available_indices,2)*rand);
            idx = available_indices(randi);
            
            Sets(i, j) = idx;
            available_indices(randi) = available_indices(end);
            available_indices = available_indices(1:end-1);
        end 
    end
    
    % 2.1) Normalize coordinates?
    % 2.2) Perform RANAC iterations and save solution with hightest score
    p1i = []; p2i = [];
    for i=1:num_iterations
       % Select a minimum set
       for j=1:8
          idx = Sets(i,j);
          
          p1i(:,j) = kp_database(:,idx);
          p2i(:,j) = kp_query(:,idx);
       end

        % 3) Compute the Fundamental matrix
        Fi = fundamentalEightPoint_normalized(p1i, p2i);

        % 4) Current score and inliers
        [current_score, current_inliers] = check_fundamental(kp_database, ...
            kp_query, Fi, num_matches, sigma);
    
        if (current_score > top_score)
           F = Fi;
           inlier_mask = current_inliers;
           top_score = current_score;
        end
    end
end