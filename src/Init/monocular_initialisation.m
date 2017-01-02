function [state, T_cw] = moncular_initialisation(img0, img1, K)

%   INPUT:
    %   - img0: First image of monocular camera.
    %   - img1: Subsequent image of monocular camera 
    %           (not necessarily second image!)
    %   - K: 3x3 matrix with the intrinsics of the camera
    
%   OUTPUT:
    %   - state: First state S^(i_1) that contains:
        %       * First set of 2D-3D correnspondences
        %       * Keypoints of img1 for later tracking
        %       * Descriptors of img1 for later tracking
    %   - T_cw: First camera pose of img1 in wold coorinate frame 
    %           (frame of img0)
    
% ALGORITHM:
    % 1. Find 2d-2d correspondences
    % 2. 8-point RANSAC
        % --> Find Fundamental matrix F
        % --> Find Essential Matrix E (E = K^T*F*K)
    % 3. Find rotation and translation pose hypotheses
    % 4. Triangulate 2D-3D points
    
    % Verbose output for debug
    debug_verbose = true;
    
    %% Initialize Output
    N = 1000;
    state = struct('matches_2d', zeros(3, N), 'matches_3d', zeros(4, N), ...
        'keypoints', zeros(3, N), 'descriptors', zeros(361, N));
    T_cw = eye(4);
    
    %% Get keypoints in both frames, descriptors and matches
    tic;
    [keypoints_database, keypoints_query, ~, descritors_query, matches] = ...
        correspondences_2d2d(img0, img1, N);
    sprintf('Time needed: correspondences_2d2d: %f seconds', toc)
    
    %% Get matching keypoints
    [~, query_indices, match_indices] = find(matches);
    kp_matches_database = keypoints_database(:, match_indices);
    kp_matches_query = keypoints_query(:, query_indices);
    
    %% Homogenous fliped keypoint coordinates
    kp_homo_database = ...
        [kp_matches_database; ones(1, size(kp_matches_database, 2))];
    kp_homo_query = ...
        [kp_matches_query; ones(1, size(kp_matches_query, 2))];
        
    %% Plot matching features
    if (debug_verbose)
        figure(5); 
        showMatchedFeatures(img0, img1, flipud(kp_matches_database)', ...
            flipud(kp_matches_query)', 'montage');
    end
    
    %% Find fundamental matrix
    tic;
    [inlier_mask, F] = ransac(kp_homo_database, kp_homo_query);
    inliers_F = nnz(inlier_mask)
    sprintf('Time needed: Find fundamental matrix: %f seconds', toc)
    
    %% Get inlier from inlier_mask
    kp_homo_database = kp_homo_database(:, inlier_mask);
    kp_homo_query = kp_homo_query(:, inlier_mask);
        
    %% Estimate Essential matrix
    E = estimateEssentialMatrix(kp_homo_database, ...
        kp_homo_query, K, K)
    E = K' * F * K
    
    %% Plot matching features
    figure(6); 
    showMatchedFeatures(img0, img1, flipud(kp_homo_database(1:2,:))', ...
        flipud(kp_homo_query(1:2,:))', 'montage');

    %% Get the hypotheses for the pose (rotation and translation)
    %[rot, u3] = get_pose_hypotheses(E)
    [R, u3]= decomposeEssentialMatrix(E);

    %% Check if in front of camera
    % lambda0*[u v 1]^T = K1 * [Xw Yw Zw]^T
    % lambda1*[u v 1]^T = K1 * R1* [Xw Yw Zw]^T + T
    [R, T, P, M1, M2] = disambiguateRelativePose(R, u3, ...
        kp_homo_database, kp_homo_query, K, K);

    %% ONLY TESTING
    % Rescale reprojection to homogenous coordinates again (u v 1)
    for i=1:size(P,2)
        p_homo(:,i) = M1*P(:,i);
        p_homo(:,i) = p_homo(:,i) ./p_homo(3,i);
    end
    
    for i=1:size(P,2)
        p_homo2(:,i) = M2*P(:,i);
        p_homo2(:,i) = p_homo2(:,i) ./p_homo2(3,i);
    end
    
    % Reprojection error
    % First image
    difference_db = kp_homo_database - p_homo;
    errors_db = sum(difference_db.^2, 1);
    errordb = sum(sqrt(errors_db))/sqrt(size(P,2))
    % Second image
    difference_qu = kp_homo_query - p_homo2;
    errors_qu = sum(difference_qu.^2, 1);
    errorqu = sum(sqrt(errors_qu))/sqrt(size(P,2))
    
    % Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
    N = size(kp_homo_query,2);
    cost_algebraic = norm( sum(kp_homo_query.*(F*kp_homo_database)) ) / sqrt(N)
    cost_dist_epi_line = distPoint2EpipolarLine(F,kp_homo_database, ...
        kp_homo_query)
    
    %% OUTPUT
    % State
    state.matches_2d = kp_homo_query;
    state.matches_3d = P;
    state.keypoints = keypoints_query;
    state.descriptors = descritors_query;
    
    % Pose (4x4)
    T_cw(1:3,:) = [R, T]
end