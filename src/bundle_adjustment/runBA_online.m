function [poses_W_opt_, landmarks_hist_] = runBA_online(poses_W_hist_,...
          landmarks_hist_, index_hist_m_, observation_hist_, K, m_on_)
% Perform online bundle adjustment on the last m_on_ frames

% Set number of iterations for lsqnonlin solver:
n_iter = 20;

% Define current hidden_state, i.e. poses and landmarks of last m_on_ frames:
idx_m = unique(index_hist_m_);
landmarks_m = landmarks_hist_(:,idx_m);
hidden_state = [poses_W_hist_; landmarks_m(:)];

% Update keypoint indices to access corresponding landmarks in current
% landmarks_m vector:
observation_i = 1;
pointer_3D_m = zeros(1, size(landmarks_hist_,2));
pointer_3D_m(idx_m) = 1:size(idx_m);

for frame_i = 1:m_on_
    num_keypoints_in_frame = observation_hist_(observation_i);
    observation_hist_((observation_i+2*num_keypoints_in_frame+1:...
        observation_i+3*num_keypoints_in_frame)) = ...
        pointer_3D_m(observation_hist_(...
        observation_i+2*num_keypoints_in_frame+1:...
        observation_i+3*num_keypoints_in_frame));
    observation_i = observation_i + 1 + 3*num_keypoints_in_frame;
end

opt_hidden_state = runBA_0(hidden_state,...
    cast(observation_hist_,'double'), K, m_on_, n_iter);
poses_W_opt_ = opt_hidden_state(1:6*m_on_);


% Update optimized landmarks:
landmarks_opt = reshape(opt_hidden_state(6*m_on_+1:end), 3, []);
landmarks_hist_(:, idx_m) = landmarks_opt;

end

