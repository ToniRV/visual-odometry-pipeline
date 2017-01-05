function error_terms = baError(hidden_state, observations, K)

plot_debug = false;

num_frames = observations(1);
T_W_frames = reshape(hidden_state(1:num_frames*6), 6, []);
p_W_landmarks = reshape(hidden_state(num_frames*6+1:end), 3, []);

error_terms = [];
observation_i = 2;

for i = 1:num_frames
    T_W_frame = twist2HomogMatrix(T_W_frames(:, i));
    num_frame_observations = observations(observation_i + 1);
    
    keypoints = flipud(reshape(observations(observation_i+2:...
        observation_i+1+num_frame_observations * 2), 2, []));
    
    landmark_indices = observations(...
        observation_i+2+num_frame_observations * 2:...
        observation_i+1+num_frame_observations * 3);
    
    landmarks = p_W_landmarks(:, landmark_indices);
    
    T_frame_W = T_W_frame ^ -1;
    
    num_landmarks = size(landmarks, 2);
    p_C_landmarks = T_frame_W(1:3,1:3) * landmarks + ...
        repmat(T_frame_W(1:3, end), [1 num_landmarks]);
    
    projections = projectPoints(p_C_landmarks, K);
    
    if plot_debug
        figure(3);
        plot(projections(1, :), projections(2, :), 'x');
        hold on;
        plot(keypoints(1, :), keypoints(2, :), 'x');
        hold off;
        axis equal;
        pause(0.01);
    end
       
    error_terms = [error_terms keypoints-projections];
    
    observation_i = observation_i + num_frame_observations * 3 + 1;
end

end

