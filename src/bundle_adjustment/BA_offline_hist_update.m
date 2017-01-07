function [poses_W_hist_, landmarks_hist_, observation_hist_, index_mask_] =...
    BA_offline_hist_update(S_i0, T_i1, validity_mask, inlier_mask, index_mask_0,...
    new_3D, new_2D, poses_W_hist_0, landmarks_hist_0, observation_hist_0)
% Updates the history of computed poses, seen landmarks, corresponding
% keypoints, and index_mask for each iteration.

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
    landmarks_hist_ = landmarks_hist_0;
    index_mask_ = index_mask_tracked;
    kp2add = kp_inliers_tracked(:);
    n_kp = size(kp_inliers_tracked,2);
end

poses_W_hist_ = [poses_W_hist_0; tau_i1];
% Update observation history:
observation_hist_ = [observation_hist_0; n_kp; kp2add; index_mask_'];

end

