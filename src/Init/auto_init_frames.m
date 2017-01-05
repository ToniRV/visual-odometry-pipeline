function [state, T_cw, reprojection_errors, costs] = ...
    auto_init_frames(initial_frame, current_frame, K)

    % Verbose output for debug
    debug_verbose = true;
    
    %% Output Initialization
    N = 1000;
    state = struct('matches_2d', zeros(3, N), 'landmarks', zeros(4, N));
    T_cw = eye(4);
    
    %% Keypoint Detection & Matching
    % Detect keypoints in both frames, obtain descriptors and find matches
    tic;
    [kp_homo_initial_matched, keypoints_current_frame] = ...
        correspondences_2d2d(initial_frame, current_frame, N);
    sprintf('Time needed: correspondences_2d2d: %f seconds', toc)
        
    %% Flip keypoints for right input of functions from exercise 5
    kp_homo_database_fl = kp_homo_initial_matched([2 1 3], :);
    kp_homo_query_fl = keypoints_current_frame([2 1 3], :);
    
    % For later plotting
    if (debug_verbose)
        kp1h_matched_before_ransac = kp_homo_initial_matched;
        kp2h_matched_before_ransac = keypoints_current_frame;
    end
    
    %% Find fundamental matrix
    tic;
    [inlier_mask, F] = ransac(kp_homo_database_fl, kp_homo_query_fl);
    sprintf('Number of inliers after ransac: %i', nnz(inlier_mask))
    sprintf('Time needed: Find fundamental matrix: %f seconds', toc)
    
    %% Obtain inliers from inlier_mask
    kp_homo_initial_matched = kp_homo_initial_matched(:, inlier_mask);
    kp_homo_current_matched = keypoints_current_frame(:, inlier_mask);
    kp_homo_database_fl = kp_homo_database_fl(:, inlier_mask);
    kp_homo_query_fl = kp_homo_query_fl(:, inlier_mask);    
    
    %% Estimate Essential matrix
    tic;
    E = estimateEssentialMatrix(kp_homo_database_fl, kp_homo_query_fl, K, K);
    sprintf('Time needed: exercise estimateEssentialMatrix: %f seconds', toc)
    
    if (debug_verbose)
        tic;
        F_test = estimateFundamentalMatrix(kp_homo_database_fl(1:2,:)', ...
            kp_homo_query_fl(1:2,:)');
        E_test = K' * F_test * K;
        sprintf('Time needed: matlab estimateFundamentalMatrix: %f seconds', toc)
    end
    
    %% Get the hypotheses for the pose (rotation and translation)
    [R_hypo, u3]= decomposeEssentialMatrix(E);

    %% Check if in front of camera
    % lambda0*[u v 1]^T = K1 * [Xw Yw Zw]^T
    % lambda1*[u v 1]^T = K1 * R1* [Xw Yw Zw]^T + T
    [R, T, P, M1, M2] = disambiguateRelativePose(R_hypo, u3, ...
        kp_homo_database_fl, kp_homo_query_fl, K, K);
    
    %% ONLY TESTING
    % Rescale reprojection to homogenous coordinates again (u v 1)
    p_homo_database = zeros(3, size(P, 2));
    p_homo_query = zeros(3, size(P, 2));
    for i=1:size(P,2)
        p_homo_database(:,i) = M1*P(:,i);
        p_homo_database(:,i) = p_homo_database(:,i) ./p_homo_database(3,i);
        p_homo_query(:,i) = M2*P(:,i);
        p_homo_query(:,i) = p_homo_query(:,i) ./p_homo_query(3,i);
    end

    % Reprojection error
    % First image
    difference_db = kp_homo_database_fl - p_homo_database;
    errors_db = sum(difference_db.^2, 1);
    errordb = sum(sqrt(errors_db))/sqrt(size(P,2));
    % Second image
    difference_qu = kp_homo_query_fl - p_homo_query;
    errors_qu = sum(difference_qu.^2, 1);
    errorqu = sum(sqrt(errors_qu))/sqrt(size(P,2));
        
    % Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
    N = size(kp_homo_query_fl,2);
    cost_algebraic = norm( sum(kp_homo_query_fl.*(F*kp_homo_database_fl)) ) / sqrt(N);
    cost_dist_epi_line = distPoint2EpipolarLine(F,kp_homo_database_fl, ...
        kp_homo_query_fl);
    
    reprojection_errors = [errordb, errorqu]
    costs = [cost_algebraic, cost_dist_epi_line]
    
    %% PLOT
    % Plot matching features
    if (debug_verbose)
        figure(5); 
        subplot(2,1,1);
        showMatchedFeatures(initial_frame, current_frame, flipud(kp1h_matched_before_ransac(1:2,:))', ...
            flipud(kp2h_matched_before_ransac(1:2,:))', 'montage');       
        subplot(2,1,2);
        showMatchedFeatures(initial_frame, current_frame, flipud(kp_homo_initial_matched(1:2,:))', ...
            flipud(kp_homo_current_matched(1:2,:))', 'montage');
        
        % Plot 3d scene (trajectory and landmarks)
%         plot_trajectory_with_landmarks(initial_frame, current_frame, kp_homo_database_fl, ...
%             kp_homo_query_fl, P, R, T);
    end
    
    %% OUTPUT
    % State
    state.matches_2d = kp_homo_current_matched;
    state.landmarks = P;
    
    % Pose (4x4)
    T_cw(1:3,:) = [R, T]
end