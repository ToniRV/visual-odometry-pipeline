function [ State_i1, Transform_i1, inlier_mask] = processFrame(Image_i1, Image_i0, State_i0, i1)
%PROCESSFRAME Summary of this function goes here
%   Detailed explanation goes here

    %% Step 1 & 2:
    % A) Run p3p+ransac to get transformation and new keypoints, both matched
    % and unmatched. Given the images and the correspondences 2D<->3D
    % correspondences and K.
    [R_C_W, t_C_W, valid_tracked_keypoints, valid_p_W_landmarks, validity_mask, inlier_mask] = ...
    ransacLocalization(Image_i1, Image_i0,  State_i0.keypoints_correspondences, ...
                                  State_i0.p_W_landmarks_correspondences, State_i0.K);
    % Detect new keypoints
    [query_keypoints, ~] = harrisDetector (Image_i1);
    
    % B) Retrieve transformation
    Transform_i1 = [R_C_W, t_C_W];
    isLocalized = numel(R_C_W)>0;
    
    keypoints_correspondences_i1 = valid_tracked_keypoints(:, inlier_mask > 0); % WARNING: should we round, ceil floor?
    p_W_landmarks_correspondences_i1 = valid_p_W_landmarks(:,inlier_mask > 0);
    
    
    %% Step 3: trying to triangulate new landmarks
    descriptor_radius = 9;
    match_lambda = 4;
    points_2D_global_var = 0;
    final_keypoints_correspondences_i1 = keypoints_correspondences_i1;
    final_p_W_landmarks_correspondences_i1 = p_W_landmarks_correspondences_i1;
          
    
    %First time we start:
    if (isempty(State_i0.first_obs_candidate_keypoints))
        new_first_obs_cand_kp_i1 = zeros(2, 0); % keypoints 
        new_first_obs_cand_tf_i1 = zeros(12, 0); % transform
        new_last_obs_cand_kp_i1 = zeros(2, 0); % keypoints
        if(isLocalized)
            new_first_obs_cand_kp_i1 = query_keypoints; % Non_matched_query_keypoints
            % TODO find better way to deal with the transform of the first
            % observed candidates
            new_first_obs_cand_tf_i1 = repmat(reshape(Transform_i1,12,1), 1, size(query_keypoints, 2)); % Store as 12xM transform
            new_last_obs_cand_kp_i1 = new_first_obs_cand_kp_i1; %First time we store them we also fill last_obs as first_obs for uniformity
        else
            fprintf('[INFO] Iteration %d: set of first_observed_candidate_keypoints is empty and we have not localized \n', i1);
        end
        
        % F) FINAL RESULT
        State_i1.first_obs_candidate_keypoints = new_first_obs_cand_kp_i1;
        State_i1.first_obs_candidate_transform = new_first_obs_cand_tf_i1;
        State_i1.last_obs_candidate_keypoints = new_last_obs_cand_kp_i1;
    else
        %Subsequent times:
        % A) Store input for this part:
        first_obs_cand_kp_i0 = State_i0.first_obs_candidate_keypoints; 
        first_obs_cand_tf_i0 = State_i0.first_obs_candidate_transform;

        last_obs_cand_kp_i0 = State_i0.last_obs_candidate_keypoints;

        % B) Try to match all last_obs_cand_kp_i0
        [tracked_last_obs_cand_kp_i1, cand_validity_mask] = KLT(last_obs_cand_kp_i0, Image_i1, Image_i0);
        

        % C) If successful match, update track with new last obs kp, if unsuccessful
        % discard track, aka delete last and first observed candidate kp together with transform.
        last_obs_cand_kp_i1 = tracked_last_obs_cand_kp_i1 (:, cand_validity_mask > 0);
        first_obs_cand_kp_i1 = first_obs_cand_kp_i0(:, cand_validity_mask > 0);
        first_obs_cand_tf_i1 = first_obs_cand_tf_i0(:, cand_validity_mask > 0);

        %If current pose is ok continue, if not do nothing.
        if (isLocalized)
            % D) For all the updated last_obs_cand_kp_i1 and corresponding
            % first kp and tf do:
            %%% I) Check which are suitable to triangulate:
            
            %random_generator = randi(2,1,size(last_obs_cand_kp_i1, 2));
            %is_triangulable = random_generator > 1;
            is_triangulable = checkTriangulability(last_obs_cand_kp_i1, Transform_i1, ...
                                                                      first_obs_cand_kp_i1, first_obs_cand_tf_i1, State_i0.K);
            fprintf('Number of triangulable points: %d \n', nnz(is_triangulable));
            triangulable_last_kp = last_obs_cand_kp_i1(:, is_triangulable);
            triangulable_last_tf = Transform_i1;
            triangulable_first_kp = first_obs_cand_kp_i1(:, is_triangulable);
            triangulable_first_tf = first_obs_cand_tf_i1(:, is_triangulable);

            %%% II) Triangulate
            % TODO can we actually use matlab's triangulate function?
            % A way to optimize the following would be to vectorize over sets with common
            % triangulable_first_tf...
            % These will be the new 2D-3D correspondences
            num_triang_kps = size(triangulable_last_kp, 2);
            X_s = zeros(3, num_triang_kps);
            list_reprojection_errors = zeros(1, num_triang_kps);
            K = State_i0.K;
            for i = 1:num_triang_kps
                [newX_cam_frame, reprojectionError] = ...
                triangulate(flipud(triangulable_last_kp(:, i))',...
                                  flipud(triangulable_first_kp(:, i))',...
                                  (K*triangulable_last_tf)',...
                                  (K*reshape(triangulable_first_tf(:,i), 3, 4))');
                newX_cam_frame = newX_cam_frame';
                X_s(:, i) = newX_cam_frame;
                list_reprojection_errors(i) = reprojectionError;
            end

            %%% III) Update state
            %%%% a) Store new 2D-3D correspondences which are valid
            reprojectionError_threshold = 2; % WARNING: this guy gets rid of MANY possible landmarks!
            valid_errors = list_reprojection_errors < reprojectionError_threshold;
            valid_depth = X_s(3,:) > 0;
            valid_indices = valid_errors & valid_depth;
            fprintf('Number of valid triangulated points: %d \n', nnz(valid_indices));
            points_3D_cam_frame = X_s(:, valid_indices);
            % IS THIS CORRECT?
            points_3D_W = points_3D_cam_frame;
            points_2D = triangulable_last_kp(:, valid_indices);
            
            %%%% b) Append to already known 2D-3D correspondences
            final_keypoints_correspondences_i1 = [keypoints_correspondences_i1,  points_2D];
            final_p_W_landmarks_correspondences_i1 = [p_W_landmarks_correspondences_i1, points_3D_W];
            
            points_2D_global_var = points_2D; % Only for plotting later...
            
            %%%% b) Clear triangulated tracks, both correctly and incorrectly
            %%%% triangulated
            last_obs_cand_kp_i1 = last_obs_cand_kp_i1(:, is_triangulable == 0);
            first_obs_cand_kp_i1 = first_obs_cand_kp_i1(:, is_triangulable == 0);
            first_obs_cand_tf_i1 = first_obs_cand_tf_i1(:, is_triangulable == 0);
                

            % E) For the non_matched_query_keypoints, append them as new candidates to current candidates.
            %%% I) Create values:
            new_first_obs_cand_kp_i1 = query_keypoints; % These guys haven't been matched twice.
            new_first_obs_cand_tf_i1 = repmat(reshape(Transform_i1, 12, 1), 1, size(new_first_obs_cand_kp_i1, 2));
            new_last_obs_cand_kp_i1 = new_first_obs_cand_kp_i1; %First time we store them we also fill last_obs as first_obs for uniformity

            %%% II) Append Results
            last_obs_cand_kp_i1 = [last_obs_cand_kp_i1, new_last_obs_cand_kp_i1];
            first_obs_cand_kp_i1 = [first_obs_cand_kp_i1, new_first_obs_cand_kp_i1];
            first_obs_cand_tf_i1 = [first_obs_cand_tf_i1, new_first_obs_cand_tf_i1];

            % F) FINAL RESULT
            State_i1.last_obs_candidate_keypoints = last_obs_cand_kp_i1;
            State_i1.first_obs_candidate_keypoints = first_obs_cand_kp_i1;
            State_i1.first_obs_candidate_transform = first_obs_cand_tf_i1;
        end
    end % end of if (isempty(State_i0.first_obs_candidate_keypoints))
    
    % F) FINAL RESULTS
    State_i1.keypoints_correspondences = final_keypoints_correspondences_i1;
    State_i1.p_W_landmarks_correspondences = final_p_W_landmarks_correspondences_i1;

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
    plot(valid_tracked_keypoints(2, (1-inlier_mask)>0), ...
        valid_tracked_keypoints(1, (1-inlier_mask)>0), 'rx', 'Linewidth', 2);
    if (nnz(inlier_mask) > 0)
        plot(keypoints_correspondences_i1(2, :), ...
            keypoints_correspondences_i1(1, :), 'gx', 'Linewidth', 2);
    end
    % Plot the new guys
    if (points_2D_global_var ~= 0)
        plot (points_2D_global_var(2, :), points_2D_global_var(1, :), 'bx');
    end
    valid_keypoints_correspondences_i0 = State_i0.keypoints_correspondences(:, validity_mask > 0);
    keypoints_correspondences_i0 = valid_keypoints_correspondences_i0(:, inlier_mask > 0);
    x_from = keypoints_correspondences_i1(1, :);
    x_to = keypoints_correspondences_i0(1, :);
    y_from = keypoints_correspondences_i1(2, :);
    y_to = keypoints_correspondences_i0(2, :);
    plot([y_from; y_to], [x_from; x_to], 'g-', 'Linewidth', 3);
    hold off;
    title('Inlier and outlier matches');
    pause(0.01);

end

function is_triangulable = checkTriangulability(last_kps, last_tf, first_kps, first_tfs, K)
%%% last_tf: transformation from World to Camera of the last kps
%%% first_tf: transformation from World to Camera of the first kps
    %%% Tune this threshold
    threshold = 20;
    %1) Compute last bearing vector in the World frame
    bearing_vector_last_kps = computeBearing(last_kps, last_tf, K);
    %2) Compute first bearing vector in the World frame
    bearing_vector_first_kps = zeros(3, size(first_tfs, 2));
    for i = 1:size(first_kps,2)
        first_tf = reshape(first_tfs(:, i), 3, 4);
        bearing_vector_first_kps(:, i) = computeBearing(first_kps(:, i), first_tf, K);
    end
    %3) Check which current kps are triangulable
    angles = atan2d(norm(cross(bearing_vector_last_kps, bearing_vector_first_kps)), ...
        dot(bearing_vector_last_kps, bearing_vector_first_kps));
    is_triangulable = angles > threshold;
end

function bearing_vector = computeBearing(kps, tfs, K)
    % Get bearings orientation in cam frame
    bearings = K\[kps; ones(1, size(kps,2))];
    % Get rot matrix from cam points to world
    R_C_W = tfs(:, 1:3);
    % Get bearings orientation in world frame
    bearings_in_world_frame = R_C_W*bearings;
    bearing_vector = bearings_in_world_frame;
end
