function [ State_i1, Transform_i1, inlier_mask] = processFrame(Image_i1, Image_i0, State_i0, i1)
%PROCESSFRAME Summary of this function goes here
%   Detailed explanation goes here

    % Run p3p+ransac to get transformation and new keypoints, both matched
    % and unmatched. Given the images and the correspondences 2D<->3D
    % correspondences and K.
    [R_C_W, t_C_W, query_keypoints, all_matches, inlier_mask] = ...
    ransacLocalization(Image_i1, Image_i0,  State_i0.keypoints_correspondences, ...
    State_i0.p_W_landmarks_correspondences, State_i0.K);

    %Retrieve transformation
    Transform_i1 = [R_C_W, t_C_W];

    % Get matched keypoints with both inliers and outliers
    matched_query_keypoints = query_keypoints(:, all_matches > 0);
    % Get matched keypoints with only inliers
    inlier_matched_query_keypoints =  matched_query_keypoints(:, inlier_mask>0);
    
    
    %% Step 3: trying to triang new landmarks
    % a) Store the new non-matched query keypoints as candidates for
    % triangulation in next iteration. We don't store keypoints that were
    % matched and are outliers... should we?
    non_matched_query_keypoints = query_keypoints(:, all_matches == 0);
    State_i1.candidate_keypoints = non_matched_query_keypoints;
    State_i1.candidates_transformation = Transform_i1;
    
    % These will be the new 2D-3D correspondences
    points_3D = 0;
    points_2D = 0;
    % b) Try to find correspondences with previous candidates
    if ((State_i0.candidate_keypoints ~= 0) &...
            (State_i0.success == 1) & numel(R_C_W)>0) % we only do it if we already have candidates, and valid cameraMatrices, aka transforns
    descriptor_radius = 9;
    match_lambda = 4;
    descriptors_i0_candidates = ...
        describeKeypoints(Image_i0, State_i0.candidate_keypoints, descriptor_radius);
    descriptors_i1_candidates = ...
        describeKeypoints(Image_i1, State_i1.candidate_keypoints, descriptor_radius);
    matches = matchDescriptors(descriptors_i1_candidates, descriptors_i0_candidates,...
        match_lambda);
    [~, query_indices, match_indices] = find(matches);
    matched_i0_candidate_keypoints = State_i0.candidate_keypoints(:, match_indices);
    matched_i1_candidate_keypoints = State_i1.candidate_keypoints(:, query_indices);
    
    % c) Try to find new 2D-3D correspondences
        % TODO check that the bearing vector is bigger than a certain
        % threshold
        
        % If ok, then triangulate
        M_i0 = State_i0.candidates_transformation;
        M_i1 = State_i1.candidates_transformation; % TODO check that this is correct, it actually depends on the Kitti coords.
                                                                                    % According to the coords in this paper http://www.mrt.kit.edu/z/publ/download/2013/GeigerAl2013IJRR.pdf

        % TODO can we actually use matlab's triangulate function?                                                                            
        [new_X, reprojectionErrors] = ...
            triangulate(flipud(matched_i1_candidate_keypoints)', flipud(matched_i0_candidate_keypoints)', M_i0', M_i1');
        fprintf('It took %ds to compute matlab_triangulation \n', toc);
        new_X = new_X';
        reprojectionErrors = reprojectionErrors';
        % TODO we should still remove 3d points with big reprojection error
        % (i.e. >0.5?) 
        reprojectionError_threshold = 0.10; % WARNING: this guy gets rid of MANY possible landmarks!
        valid_errors = reprojectionErrors < reprojectionError_threshold;
        valid_depth = new_X(3,:) > 0;
        valid_indices = valid_errors & valid_depth;
        
        points_3D = new_X(:, valid_indices);
        points_2D =  matched_i1_candidate_keypoints(:, valid_indices);
    end
    
    %% 
    % Get inlier matched landmarks
    % all_matches gives the matches and non matches of the query keypoints,
    % corresponding_matches gives the matched for only the ones that have been matched
    % including outliers (which are excluded using inlier_mask)
    corresponding_matches = all_matches(all_matches > 0);
    matched_corresponding_landmarks = ...
        State_i0.p_W_landmarks_correspondences(:, corresponding_matches);
    inlier_matched_corresponding_landmarks = matched_corresponding_landmarks(:, inlier_mask>0);
    
    % Store only the matched inlier keypoints of the Image_i1 plus the new
    % ones triangulated
     if (isempty(points_2D) | points_2D ==0)
         % WITH ONLY STORING INLIERS (!!this is the suggested method!!)
%         State_i1.keypoints_correspondences = inlier_matched_query_keypoints;
%         State_i1.p_W_landmarks_correspondences = inlier_matched_corresponding_landmarks;
         % STORING ALL MATCHES
        State_i1.keypoints_correspondences = matched_query_keypoints;
        State_i1.p_W_landmarks_correspondences = matched_corresponding_landmarks;
     else
         %%% WITH ONLY STORING INLIERS (!!this is the suggested method!!)
%            State_i1.keypoints_correspondences = [inlier_matched_query_keypoints, points_2D];
        % Keep the found 3D landmarks that match to the inlier keypoints plus the new ones triangulated. TODO
        % what about the rest of outlier landmarks, shouldn't we keep the
        % previous 2d-3d correspondence for other frames? Probably yes, this
        % way if there's an occlusion we can still track the landmark later...
%             State_i1.p_W_landmarks_correspondences = [inlier_matched_corresponding_landmarks, points_3D];
         %%% STORING ALL MATCHES
          State_i1.keypoints_correspondences = [matched_query_keypoints, points_2D];
          State_i1.p_W_landmarks_correspondences = [matched_corresponding_landmarks, points_3D];
           end
    State_i1.K = State_i0.K;
    

%% Plotting
% Distinguish success from failure.
    if (numel(R_C_W) > 0)
        subplot(1, 3, 3);
        num_inliers(i1) = nnz(inlier_mask);
        plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2);
        disp(['Frame ' num2str(i1) ' localized with ' ...
            num2str(num_inliers(i1)) ' inliers!']);
        view(0,0);
        % Since we are successful in localizing
        State_i1.success = 1;
    else
        % Since we are NOT successful in localizing
        State_i1.success = 0;
        disp(['Frame ' num2str(i1) ' failed to localize!']);
    end

    subplot(1, 3, [1 2]);
    imshow(Image_i1);
    hold on;
    plot(matched_query_keypoints(2, (1-inlier_mask)>0), ...
        matched_query_keypoints(1, (1-inlier_mask)>0), 'rx', 'Linewidth', 2);
    if (nnz(inlier_mask) > 0)
        plot(matched_query_keypoints(2, (inlier_mask)>0), ...
            matched_query_keypoints(1, (inlier_mask)>0), 'gx', 'Linewidth', 2);
    end
    plotMatches(corresponding_matches(inlier_mask>0), ...
        matched_query_keypoints(:, inlier_mask>0), ...
        State_i0.keypoints_correspondences);
    hold off;
    title('Inlier and outlier matches');
    pause(0.01);

end

