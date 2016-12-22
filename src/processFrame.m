function [ State_i1, Transform_i1, inlier_mask] = processFrame(Image_i1, Image_i0, State_i0, i1)
%PROCESSFRAME Summary of this function goes here
%   Detailed explanation goes here
    [R_C_W, t_C_W, query_keypoints, all_matches, inlier_mask] = ...
    ransacLocalization(Image_i1, Image_i0,  State_i0.keypoints, ...
    State_i0.p_W_landmarks, State_i0.K);

    % Matched keypoints with outliers
    matched_query_keypoints = query_keypoints(:, all_matches > 0);
    % all_matched gives the matches and non matches of the query keypoints,
    % corresponding_matches gives the matched for only the ones that have been matched
    % including outliers (which are excluded using inlier_mask)
    corresponding_matches = all_matches(all_matches > 0);
    matched_corresponding_landmarks = State_i0.p_W_landmarks(:, corresponding_matches);
    
    inlier_matched_corresponding_landmarks = matched_corresponding_landmarks(:, inlier_mask>0);
    
    Transform_i1 = [R_C_W, t_C_W];
    
    % Store only the matched inlier keypoints of the Image_i1
    State_i1.keypoints = matched_query_keypoints(:, inlier_mask>0);
    % Keep the found 3D landmarks that match to the inlier keypoints
    State_i1.p_W_landmarks = inlier_matched_corresponding_landmarks;
    
    State_i1.K = State_i0.K;

%% Plotting
% Distinguish success from failure.
    % Store number of inliers per iterations
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
        State_i0.keypoints);
    hold off;
    title('Inlier and outlier matches');
    pause(0.01);

end

