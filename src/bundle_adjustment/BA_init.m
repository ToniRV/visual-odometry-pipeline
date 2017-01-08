function [index_mask_, index_hist_m_, poses_W_hist_, poses_W_opt_,...
    landmarks_hist_, observation_hist_] = ...
    BA_init(S_i0, T_i0, initialisation_, m_on_)
% Initialize parameters according to the chosen bundle adjustment method.

% Initialize observation vector and index mask:
index_mask_ = (1:size(S_i0.p_W_landmarks_correspondences,2));
observation_hist_ = [size(S_i0.p_W_landmarks_correspondences,2);...
    S_i0.keypoints_correspondences(:); index_mask_'];

% Initialize global landmark vector with landmarks from initialization:
landmarks_hist_ = S_i0.p_W_landmarks_correspondences;

% Initialize index history for last m frames:
index_hist_m_ = index_mask_;

if (strcmp(initialisation_,'Monocular') == 1)
    tau_i0 = HomogMatrix2twist([T_i0(1:3,1:3)',...
        -T_i0(1:3,1:3)'*T_i0(1:3,4); zeros(1,3), 1]);
    else    
        R_init_ = eye(3);
        t_init_ = zeros(3,1);
        tau_i0 = HomogMatrix2twist([R_init_, t_init_; zeros(1,3) 1]);
end

% Initialize pose history and optimized poses:
poses_W_opt_ = zeros(6*m_on_,1);
poses_W_hist_ = tau_i0;
end

