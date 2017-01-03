function [img0, img1, kp_homo_database_matched, kp_homo_query_matched, ...
    descriptors_query_matched] = auto_init_frames(initial_frame, current_frame)
    %% Get keypoints in both frames, descriptors and matches
    tic;
    N = 1000;
    [keypoints_database, keypoints_query, ~, descriptors_query, matches] = ...
        correspondences_2d2d(initial_frame, current_frame, N);
    sprintf('Time needed: correspondences_2d2d: %f seconds', toc)
        
    %% Get matching keypoints
    [~, query_indices, match_indices] = find(matches);
    kp_database_matched = keypoints_database(:, match_indices);
    kp_query_matched = keypoints_query(:, query_indices);
    descriptors_query_matched = descriptors_query(:, query_indices);
    
    %% Homogenous fliped keypoint coordinates
    kp_homo_database_matched = ...
        [kp_database_matched; ones(1, size(kp_database_matched, 2))];
    kp_homo_query_matched = ...
        [kp_query_matched; ones(1, size(kp_query_matched, 2))];

end