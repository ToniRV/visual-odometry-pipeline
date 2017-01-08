function p = makeMonoInit (parameters)
    
    p = @monocular_initialisation;
    
    % Retrieve parameters
    debug_verbose_ = parameters.debug_verbose;
    N_ = parameters.num_keypoints;
    K_ = parameters.K;
    parameters_correspondences_2D2D_ = parameters.correspondences_2D2D;
    parameters_ransac_ = parameters.ransac;

    function [state, T_cw, reprojection_errors, costs] = ...
            monocular_initialisation(img0, img1)

    %   INPUT:
        %   * img0: First image taken by monocular camera
        %   * img1: Subsequent image taken by monocular camera 
        %           (not necessarily second image!)
        %   * K:    3x3 matrix containing the intrinsic parameters of the camera

    %   OUTPUT:
        %   * state: First state S^(i_1) that contains:
        %               --> First set of 2D-3D correnspondences
        %               --> Keypoints in img1 for subsequent tracking
        %               --> Keypoint descriptors in img1 for subsequent tracking
        %   * T_cw:  First camera pose of img1 in wold coorinate frame 
        %            (frame of img0)

    %   ALGORITHM:
        %   1. Find 2d-2d correspondences
        %   2. 8-point RANSAC
        %       --> Find Fundamental matrix F
        %       --> Find Essential Matrix E (E = K^T*F*K)
        %   3. Find rotation and translation pose hypotheses
        %   4. Triangulate 2D-3D points

        %% Output Initialization   
        state = struct('matches_2d', zeros(3, N_), 'landmarks', zeros(4, N_));
        T_cw = eye(4);

        %% Keypoint Detection & Matching
        % Detect keypoints in both frames, obtain descriptors and find matches
        tic;
        corresp_2d2d = makeCorrespondences2D2D(parameters_correspondences_2D2D_);
        [kp_homo_database_matched, kp_homo_query_matched] = ...
            corresp_2d2d(img0, img1);
        sprintf('Time needed: correspondences_2d2d: %f seconds', toc)

        % For later plotting
        if (debug_verbose_)
            kp1h_matched_before_ransac = kp_homo_database_matched;
            kp2h_matched_before_ransac = kp_homo_query_matched;
        end

        %% Flip keypoints for right input of functions from exercise 5
        kp_homo_database_fl = kp_homo_database_matched([2 1 3], :);
        kp_homo_query_fl = kp_homo_query_matched([2 1 3], :);

        %% Find fundamental matrix
        tic;
        h_ransac = makeRansac(parameters_ransac_);
        [inlier_mask, F] = h_ransac(kp_homo_database_fl, kp_homo_query_fl);
        sprintf('Number of inliers after ransac: %i', nnz(inlier_mask))
        sprintf('Time needed: Find fundamental matrix: %f seconds', toc)

        %% Obtain inliers from inlier_mask
        kp_homo_database_matched = kp_homo_database_matched(:, inlier_mask);
        kp_homo_query_matched = kp_homo_query_matched(:, inlier_mask);
        kp_homo_database_fl = kp_homo_database_fl(:, inlier_mask);
        kp_homo_query_fl = kp_homo_query_fl(:, inlier_mask);    

        %% Estimate Essential matrix
        tic;
        E = estimateEssentialMatrix(kp_homo_database_fl, kp_homo_query_fl, K_, K_);
        sprintf('Time needed: exercise estimateEssentialMatrix: %f seconds', toc)

        %% Get the hypotheses for the pose (rotation and translation)
        [R_hypo, u3]= decomposeEssentialMatrix(E);

        %% Check if in front of camera
        % lambda0*[u v 1]^T = K1 * [Xw Yw Zw]^T
        % lambda1*[u v 1]^T = K1 * R1* [Xw Yw Zw]^T + T
        [R, T, P, M1, M2, inlier] = ...
            disambiguateRelativePose(R_hypo, u3, ...
            kp_homo_database_fl, kp_homo_query_fl, K_, K_);

        %% Remove outlier behind cameras
        P = P(:, inlier);
        kp_homo_database_matched = kp_homo_database_matched(:, inlier);
        kp_homo_query_matched = kp_homo_query_matched(:, inlier);
        kp_homo_database_fl = kp_homo_database_fl(:, inlier);
        kp_homo_query_fl = kp_homo_query_fl(:, inlier);  

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
        if (debug_verbose_)
            figure(15); 
            subplot(2,1,1);
            showMatchedFeatures(img0, img1, flipud(kp1h_matched_before_ransac(1:2,:))', ...
                flipud(kp2h_matched_before_ransac(1:2,:))', 'montage');       
            subplot(2,1,2);
            showMatchedFeatures(img0, img1, flipud(kp_homo_database_matched(1:2,:))', ...
                flipud(kp_homo_query_matched(1:2,:))', 'montage');

            % Plot 3d scene (trajectory and landmarks)
            plot_trajectory_with_landmarks(img0, img1, kp_homo_database_fl, ...
                kp_homo_query_fl, P, R, T, 10);
        end

        %% OUTPUT
        % State
        state.matches_2d = kp_homo_query_matched;
        state.landmarks = P;

        % Pose (4x4)
        T_cw(1:3,:) = [R, T]
    end
end