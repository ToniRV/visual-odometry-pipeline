function p = makeRansacLocalization (parameters)

p = @ransacLocalization;

K_ = parameters.K;
num_iterations_ = parameters.num_iterations;
pixel_tolerance_ = parameters.pixel_tolerance;

function [R_C_W, t_C_W, valid_tracked_keypoints, valid_p_W_landmarks, validity_mask, inlier_mask, ...
    max_num_inliers_history] = ransacLocalization(...
    query_image, database_image, database_keypoints, p_W_landmarks, K)
% query_keypoints should be 2x1000
% all_matches should be 1x1000 and correspond to the output from the
%   matchDescriptors() function from exercise 3.
% inlier_mask should be 1xnum_matched (!!!) and contain, only for the
%   matched keypoints (!!!), 0 if the match is an outlier, 1 otherwise.

use_p3p = true;

if use_p3p
    num_iterations = 200;
    pixel_tolerance = 10;
    k = 3;
else
    num_iterations = 2000;
    pixel_tolerance = 10;
    k = 6;
end

[tracked_keypoints, validity_mask] = KLT(database_keypoints, query_image, database_image);
valid_tracked_keypoints = tracked_keypoints(:, validity_mask > 0); % WARNING: should we round, or keep sub-pixel accuracy?
valid_p_W_landmarks = p_W_landmarks(:, validity_mask > 0);

% Debug
num_db_keypoints = size(database_keypoints, 2);
num_matches = nnz(validity_mask);
fprintf('Num of database keypoints: %d \n', num_db_keypoints);
fprintf('Num of matches: %d \n', num_matches);

% Store matched keypoints and landmarks correpondences ci<->Xi
matched_query_keypoints = valid_tracked_keypoints;
corresponding_landmarks = valid_p_W_landmarks;

% Initialize RANSAC.
inlier_mask = zeros(1, size(matched_query_keypoints, 2));
matched_query_keypoints = flipud(matched_query_keypoints); % ?????? flipud here?
max_num_inliers_history = zeros(1, num_iterations);
max_num_inliers = 0;

best_R = [];
best_t = [];
% RANSAC
for i = 1:num_iterations
    [landmark_sample, idx] = datasample(...
        corresponding_landmarks, k, 2, 'Replace', false);
    keypoint_sample = matched_query_keypoints(:, idx);
    
    if use_p3p
        normalized_bearings = K\[keypoint_sample; ones(1, 3)];
        for ii = 1:3
            normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
                norm(normalized_bearings(:, ii), 2);
        end
        poses = p3p(landmark_sample, normalized_bearings);
        R_C_W_guess = zeros(3, 3, 2);
        t_C_W_guess = zeros(3, 1, 2);
        for ii = 0:1
            R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
            t_W_C_ii = real(poses(:, (1+ii*4)));
            R_C_W_guess(:,:,ii+1) = R_W_C_ii';
            t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
        end
    else
        M_C_W_guess = estimatePoseDLT(...
            keypoint_sample', landmark_sample', K);
        R_C_W_guess = M_C_W_guess(:, 1:3);
        t_C_W_guess = M_C_W_guess(:, end);
    end
    
    % Count inliers:
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,1) * corresponding_landmarks) + ...
        repmat(t_C_W_guess(:,:,1), ...
        [1 size(corresponding_landmarks, 2)]), K);
    difference = matched_query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;
    
    if use_p3p
        projected_points = projectPoints(...
            (R_C_W_guess(:,:,2) * corresponding_landmarks) + ...
            repmat(t_C_W_guess(:,:,2), ...
            [1 size(corresponding_landmarks, 2)]), K);
        difference = matched_query_keypoints - projected_points;
        errors = sum(difference.^2, 1);
        alternative_is_inlier = errors < pixel_tolerance^2;
        if nnz(alternative_is_inlier) > nnz(is_inlier)
            is_inlier = alternative_is_inlier;
            tmp_R = R_C_W_guess(:,:,2);
            tmp_t = t_C_W_guess(:,:,2);
        else
            tmp_R = R_C_W_guess(:,:,1);
            tmp_t = t_C_W_guess(:,:,1);
        end
    end
    
    if nnz(is_inlier) > max_num_inliers && nnz(is_inlier) >= 6
        max_num_inliers = nnz(is_inlier);        
        inlier_mask = is_inlier;
        best_R = tmp_R;
        best_t = tmp_t;
    end
    
    max_num_inliers_history(i) = max_num_inliers;
end

if max_num_inliers == 0
    R_C_W = [];
    t_C_W = [];
else
    % TODO last DLT estimate has to be removed according to hints?? 
%     M_C_W = estimatePoseDLT(...
%         matched_query_keypoints(:, inlier_mask>0)', ...
%         corresponding_landmarks(:, inlier_mask>0)', K);
%     R_C_W = M_C_W(:, 1:3);
%     t_C_W = M_C_W(:, end);
    R_C_W = best_R;
    t_C_W = best_t;
end

end
end

