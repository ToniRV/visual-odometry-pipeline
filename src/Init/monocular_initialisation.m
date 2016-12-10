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
    % 5. RANSAC to get rid of correspondence outliers
    
    % Initialize Output
    N = 1000;
    state = struct('matches_2d', zeros(3, N), 'matches_3d', zeros(4, N), ...
        'keypoints', zeros(3, N), 'descriptors', zeros(361, N));

    % Get keypoints in both frames, descriptors and matches
    [keypoints_database, keypoints_query, descriptors_database, ...
        descritors_query, matches] = correspondences_2d2d(img0, img1);

    % Get matching keypoints
    [~, query_indices, match_indices] = find(matches);
    kp_matches_database = keypoints_database(:, match_indices);
    kp_matches_query = keypoints_query(:, query_indices);
    
    % Plot matching features
    figure(5); 
    showMatchedFeatures(img0, img1, ...
        flipud(kp_matches_database)', flipud(kp_matches_query)', 'montage');
    
    % Homogenous fliped keypoint coordinates
    % TODO: FLIP OR NOT FLIP??
    kp_fliped_homo_database = ...
        [kp_matches_database; ones(1, size(kp_matches_database, 2))];
    kp_fliped_homo_query = ...
        [kp_matches_query; ones(1, size(kp_matches_query, 2))];
    
%     kp_fliped_homo_database = ...
%         [flipud(kp_matches_database); ones(1, size(kp_matches_database, 2))];
%     kp_fliped_homo_query = ...
%         [flipud(kp_matches_query); ones(1, size(kp_matches_query, 2))];

    % Estimate fundamental matrix
    % Call the 8-point algorithm on inputs x1,x2
    F = fundamentalEightPoint_normalized(kp_fliped_homo_database, ...
        kp_fliped_homo_query)
    E = estimateEssentialMatrix(kp_fliped_homo_database, ...
        kp_fliped_homo_query, K, K)

    % Decompose the matrix E by svd
    [u,s,v]=svd(E);

    % E = SR where S = [t]_x and R is the rotation matrix.
    % E can be factorized as:
    % s = u * z * u';
    w = [0 -1 0; 1 0 0; 0 0 1];
    z = [0 1 0; -1 0 0; 0 0 1];
    
    % Two possibilities:
    rot1 = u * w  * v';
    rot2 = u * w' * v';
    % Two possibilities:
    t1 = u(:,3) ./max(abs(u(:,3)));
    t2 = -u(:,3) ./max(abs(u(:,3)));

    % 4 possible choices of the camera matrix P2 based on the 2 possible
    % choices of R and 2 possible signs of t.
    rot(:,:,1) = rot1; 
    t(:,:,1) = t1;

    rot(:,:,2) = rot2; 
    t(:,:,2) = t2;

    rot(:,:,3) = rot1; 
    t(:,:,3) = t2;

    rot(:,:,4) = rot2; 
    t(:,:,4) = t1;
    
    %% Check if in front of camera
    % lambda0*[u v 1]^T = K1 * [Xw Yw Zw]^T
    % lambda1*[u v 1]^T = K1 * R1* [Xw Yw Zw]^T + T
    [P1, num_good(1)] = check_rt(rot(:,:,1), t(:,:,1), K, ...
        kp_fliped_homo_database, kp_fliped_homo_query);
    [P2, num_good(2)] = check_rt(rot(:,:,2), t(:,:,2), K, ...
        kp_fliped_homo_database, kp_fliped_homo_query);
    [P3, num_good(3)] = check_rt(rot(:,:,3), t(:,:,3), K, ...
        kp_fliped_homo_database, kp_fliped_homo_query);
    [P4, num_good(4)] = check_rt(rot(:,:,4), t(:,:,4), K, ...
        kp_fliped_homo_database, kp_fliped_homo_query);
    
    % Find best rotation and translation hypotheses
    [maximum, idx] = max(num_good)
    switch(idx)
        case 1 
            P = P1;
        case 2
            P = P2;
        case 3
            P = P3;
        case 4
            P = P4;
    end
    
    %repro0 = reprojectPoints(transpose(P(1:3,:)), M0, K);
    %repro1 = reprojectPoints(transpose(P(1:3,:)), M1, K);
    %inlier_mask = ransac(img0, img1, K, kp_fliped_homo_database, P)

    % Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
    N = size(kp_fliped_homo_query,2);
    cost_algebraic = norm( sum(kp_fliped_homo_query.*(F*kp_fliped_homo_database)) ) / sqrt(N)
    cost_dist_epi_line = distPoint2EpipolarLine(F,kp_fliped_homo_database, ...
        kp_fliped_homo_query)
    
    %% OUTPUT
    % State
    state.matches_2d = kp_fliped_homo_query;
    state.matches_3d = P;
    state.keypoints = keypoints_query;
    state.descriptors = descritors_query;
    
    % Pose (4x4)
    T_cw = eye(4); 
    T_cw(1:3, 1:3) = rot(:,:,idx); 
    T_cw(1:3,4) = t(:,:,idx);
end
