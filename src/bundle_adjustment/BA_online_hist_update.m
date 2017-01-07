function [poses_W_hist_, landmarks_hist_, observation_hist_, index_mask_, index_hist_m_] =...
    BA_online_hist_update(S_i0, T_i1, validity_mask, inlier_mask, index_mask_0, index_hist_m_0,...
    new_3D, new_2D, poses_W_hist_0, landmarks_hist_0, observation_hist_0, range_, m_on_, i)
% Stores poses and landmarks that have been observed in the last m_on_ frames.

% Store current pose as 6x1 twist vector:
tau_i1 = HomogMatrix2twist([T_i1(1:3,1:3)',...
    -T_i1(1:3,1:3)'*T_i1(1:3,4); zeros(1,3) 1]);

% Extract tracked keypoints from previous to current frame:
kp_tracked = S_i0.keypoints_correspondences(:, validity_mask > 0);
kp_inliers_tracked = kp_tracked(:, inlier_mask > 0);

% Update corresponding index mask of tracked keypoints:
index_temp_tracked = index_mask_0(:, validity_mask > 0);
index_mask_tracked = index_temp_tracked(:, inlier_mask > 0);

% If new landmarks are triangulated add them to the landmark history
% and update the corresponding index_mask:
if (isempty(new_3D) == 0)
    index_mask_new = (size(landmarks_hist_0,2)+1):...
        (size(landmarks_hist_0,2)+size(new_3D,2));
    landmarks_hist_ = [landmarks_hist_0, new_3D];
    index_mask_ = [index_mask_tracked, index_mask_new];
    kp2add = [kp_inliers_tracked(:); new_2D(:)];
    n_kp = size(kp_inliers_tracked,2) + size(new_2D,2);
else
    index_mask_ = index_mask_tracked;
    kp2add = kp_inliers_tracked(:);
    n_kp = size(kp_inliers_tracked,2);
end

if (i <= range_(m_on_-1)) % Make sure that the first m_on_ frames passed before calling BA
    index_hist_m_ = [index_hist_m_0, index_mask_];
    poses_W_hist_ = [poses_W_hist_0; tau_i1];
    % Update observation history:
    observation_hist_ = [observation_hist_0; n_kp; kp2add; index_mask_'];
else          
    % Track which landmarks have been observed in the last m frames
    % and extract these 3D points:
    num_remove = observation_hist_0(1);
    index_hist_m_ = [index_hist_m_0(num_remove+1:end), index_mask_];
    % Remove duplicates and sort:
    % index_hist_m_sorted = unique(index_hist_m_);

    poses_W_hist_ = [poses_W_hist_0(7:6*m_on_,1); tau_i1];
    observation_hist_ = [observation_hist_0(num_remove*3+2:end);...
        n_kp; kp2add; index_mask_'];
    % opt_hidden_state = runBA(hidden_state, cast(observation_hist_,'double'), K, m_on_);
end

end

