function [img0, img1, kp_homo_initial_matched, kp_homo_current_matched, ...
    descriptors_current_matched] = auto_init_frames(initial_frame, current_frame)
    %% Get keypoints in both frames, descriptors and matches
    tic;
    N = 1000;
    [keypoints_initial_frame, keypoints_current_frame, ~, ...
        descriptors_current, matches] = ...
        correspondences_2d2d(initial_frame, current_frame, N);
    sprintf('Time needed: correspondences_2d2d: %f seconds', toc)
        
    %% Get matching keypoints
    [~, query_indices, match_indices] = find(matches);
    kp_initial_matched = keypoints_initial_frame(:, match_indices);
    kp_current_matched = keypoints_current_frame(:, query_indices);
    descriptors_current_matched = descriptors_current(:, query_indices);
    
    %% Homogenous fliped keypoint coordinates
    kp_homo_initial_matched = ...
        [kp_initial_matched; ones(1, size(kp_initial_matched, 2))];
    kp_homo_current_matched = ...
        [kp_current_matched; ones(1, size(kp_current_matched, 2))];

    %% Output images
    img0 = initial_frame;
    img1 = current_frame;
end