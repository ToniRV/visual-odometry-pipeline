function [state, T_cw] = moncular_initialisation(img0, img1, K)

%   INPUT:
    %   - img0: First image of monocular camera.
    %   - img1: Subsequent image of monocular camera (not necessarily second image!).
    %   - K: 3x3 matrix with the intrinsics of the camera
    
%   OUTPUT:
    %   - state: First state S^(i_1) that contains:
        %       * First set of 2D-3D correnspondences
        %       * Keypoints of img1 for later tracking
        %       * Descriptors of img1 for later tracking
    %   - T_cw: First camera pose of img1 in wold coorinate frame (frame of img0)

    % Get keypoints in both frames, descriptors and matches
    [keypoints_data, keypoints_query, descriptors_data, ...
        descritors_query, matches] = correspondences_2d2d(img0, img1);

    % Get matching keypoints
    [~, query_indices, match_indices] = find(matches);
    kp_matches_data = keypoints_data(:, match_indices);
    kp_matches_query = keypoints_query(:, query_indices);
    
    % Plot matching features
    figure(5); 
    showMatchedFeatures(img0, img1, ...
        flipud(kp_matches_data)', flipud(kp_matches_query)', 'montage');
    
    % Homogenous fliped keypoint coordinates
    kp_fliped_homo_data = ...
        [flipud(kp_matches_data); ones(1, size(kp_matches_data, 2))];
    kp_fliped_homo_query = ...
        [flipud(kp_matches_query); ones(1, size(kp_matches_query, 2))];

    % Estimate fundamental matrix
    % Call the 8-point algorithm on inputs x1,x2
    F = fundamentalEightPoint_normalized(kp_fliped_homo_data, ...
        kp_fliped_homo_query)
    E = estimateEssentialMatrix(kp_fliped_homo_data, ...
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
    for i=1:size(rot,3)
        M0 = K*[eye(3) zeros(3,1)];
        M1 = K*[rot(:,:,i) t(:,:,i)];

        P_test = linearTriangulation(kp_fliped_homo_data(:,1), ...
            kp_fliped_homo_query(:,1), M0, M1);

        px0 = M0*P_test;
        px1 = M1*P_test;

        % Check if point in front of both cameras
        if ((px0(3) > 0) && (px1(3) > 0))
            R = rot(:,:,i); 
            T = t(:,:,i);
            P = linearTriangulation(kp_fliped_homo_data, ...
                kp_fliped_homo_query, M0, M1);
            break
        end

        %x3d0(:,i) = inv(K)*p0(:,2);
        %kr = K*rot(:,:,i);
        %x3d1(:,i) = inv(kr)*(p1(:,2) - t(:,:,i));
    end

    repro0 = reprojectPoints(transpose(P(1:3,:)), M0, K);
    repro1 = reprojectPoints(transpose(P(1:3,:)), M1, K);
    %inlier_mask = ransac(img0, img1, K, keypoints1, P(1:3,:))

    % Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
    N = size(kp_fliped_homo_query,2);
    cost_algebraic = norm( sum(kp_fliped_homo_query.*(F*kp_fliped_homo_data)) ) / sqrt(N)
    cost_dist_epi_line = distPoint2EpipolarLine(F,kp_fliped_homo_data, ...
        kp_fliped_homo_query)
    
    %% OUTPUT
    state = []
    T_cw = []
end
